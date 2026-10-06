# 项目理解报告

请尽量使用自己的语言回答以下问题。可以引用少量关键代码或伪代码，但不要只粘贴实现。
完成一节后删除该节末尾的待填写标记；本地检查会拒绝仍有未完成章节的报告。

## 1. 图像生命周期与所有权

`ImageSequenceSource` 用一块成员 `buffer_` 模拟相机内部缓冲区：每次 `next()` 先把新图 `copyTo(buffer_)`，再交给 `Frame`。真实相机也是这样，采集环里只有有限几块内存，读下一帧就会覆盖上一块。

`cv::Mat` 的普通赋值是浅拷贝，只复制头信息并增加引用计数，像素仍指向同一块内存。原先写的是 `frame.image = buffer_`，于是入队的 `Frame` 和源里的 `buffer_` 共享像素。下一次 `next()` 再往 `buffer_` 里写，已经交给队列、甚至正在被 worker 处理的旧帧也会一起变掉，checksum 对不上，就会被记成 corrupted。

修改是 `frame.image = buffer_.clone()`。`clone()` 分配新的像素缓冲并拷贝数据，这一帧从此自己拥有图像。之后源再复用 `buffer_`，改的是源自己的那份，碰不到已经入队的 `Frame`。`expected_checksum` 在 clone 之前对 `buffer_` 算一次，和入队后的独立副本内容一致，worker 侧再校验就能通过。

## 2. 并发处理与恰好一次

`BlockingQueue` 的 `push` 在锁内把元素 `move` 进 `std::queue`，再 `notify_one`；`pop` 在锁内等到「已关闭或队列非空」，若队列空则返回 false，否则把队首 `move` 给调用者。一次 `pop` 只唤醒并交给一个 worker，同一帧不会被两个人取走。

producer 每成功 `next()` 一帧就 `onProduced()`，再 `push(std::move(frame))`。输入耗尽后调用 `queue_.close()`。close 只是把 `closed_` 置位并 `notify_all`，并不丢弃已经在队列里的元素。因此：

- 已入队但还没被取走的帧，worker 仍会 `pop` 出来处理，不会漏。
- 每个元素只会被 move 走一次，不会重复处理。
- 输入耗尽时，正在处理当前帧的 worker 做完本轮（校验、process、imwrite）后再次 `pop`；若队列已空且已 close，`pop` 返回 false，循环结束。
- 卡在 `pop` 上等待的 worker 被 `notify_all` 唤醒，发现队列空且已关闭，同样返回 false 退出。

所以「恰好一次」来自队列的互斥交付，而不是靠额外的帧 ID 去重。

## 3. 共享统计数据

producer 线程调用 `onProduced()`；每个 worker 调用 `onProcessed()`、`onSaved()`，校验失败时还调用 `onCorrupted()`；main 在 `wait()` 之后通过 `snapshot()` 读四个计数。这些调用会交错发生。

原实现把「读旧值 → 睡 100µs → 写成旧值+1」暴露成一段无锁代码。两个线程读到同一个旧值，后写的覆盖先写的，一次更新就丢了。`snapshot()` 无锁读四个 `int`，也可能读到只改了一半的组合。

现在 `Statistics` 内有一把 `mutable std::mutex`。所有 `onXxx()` 和 `snapshot()` 都在 `lock_guard` 里进行。同一时刻只有一个线程能改计数，慢递增仍然保留（题目用它放大竞争），但竞争被锁消掉了。`snapshot()` 也在同一把锁下拷贝四个字段，读到的是一致的一组值，不会出现 produced 已经加完、processed 还是旧值这种撕裂快照。

## 4. 线程关闭协议

`std::thread` 在仍然 `joinable()` 时析构会直接 `std::terminate`。因此所有已 `start()` 的线程必须在 `Pipeline` 销毁前被 join。

1. 显式 `wait()`：producer 读完图像后自己 `queue_.close()`，再结束；`wait()` 先 join producer，再 join 每个 worker。worker 侧 `pop` 在 close 且队列空时返回 false，循环退出，join 能返回。这条路径不会永久等待，因为 close 一定发生在 producer 退出之前。

2. 不调用 `wait()`，直接析构：`~Pipeline()` 先 `queue_.close()`，再调用 `wait()`。先 close 是为了叫醒还堵在 `pop` 上的 worker，以及让仍在跑的 producer 后续 `push` 直接丢弃（`push` 发现已关闭就 return）。然后 join 所有线程。这样不会留下 joinable 的 `std::thread`，也就不会 `terminate`；worker 也不会在对象销毁后还访问 `queue_` / `statistics_`。

`wait()` 对每个线程都判断了 `joinable()`，join 过的线程再 wait 或再进析构是空操作，因此析构里调用 `wait()` 是安全的。`start()` 没有“已启动”标志，重复调用会再创建一批线程，前置条件是整个生命周期只 `start()` 一次。`close()` 可以重复调用。
