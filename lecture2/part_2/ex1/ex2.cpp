#include <iostream>
class car
{
public:
  car();
  ~car();
  void run();
};
car::car() { std::cout << "car构造完成" << std::endl; }
car::~car() { std::cout << "car析沟完成" << std::endl; }
int main()
{
  {
    car car;
    car.run();
  }
  return 0;
}
void car::run() { std::cout << "这是一个car类" << std::endl; }