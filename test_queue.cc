#include <iostream>
#include "queue.h"


class Label: public LabelInterface<uint32_t>
{
 public:
  Label(uint32_t id, double cost) : id_(id), cost_(cost) {}
  uint32_t id() const {return id_;}
  double sortcost() const {return cost_;}

 private:
  uint32_t id_;
  double cost_;
};


bool operator==(const Label& lhs, const Label& rhs)
{
  return lhs.id() == rhs.id() && lhs.sortcost() == rhs.sortcost();
}


bool operator!=(const Label& lhs, const Label& rhs)
{
  return !(lhs == rhs);
}


void SimpleTestQueue()
{
  SPQueue<Label> queue;
  assert(queue.size() == 0 && queue.empty());

  queue.push(Label(1, 3));
  assert(queue.top() == Label(1, 3));
  assert(queue.size() == 1);

  queue.push(Label(2, 2));
  assert(queue.top() == Label(2, 2));
  assert(queue.size() == 2);

  queue.push(Label(2, 4));
  assert(queue.top() == Label(2, 2));
  assert(queue.size() == 2);

  queue.push(Label(2, 2));
  assert(queue.top() == Label(2, 2));
  assert(queue.size() == 2);

  queue.push(Label(1, 1));
  assert(queue.top() == Label(1, 1));
  assert(queue.size() == 2);

  queue.pop();
  assert(queue.top() == Label(2, 2));
  assert(queue.size() == 1);

  queue.pop();
  assert(queue.empty() && queue.size() == 0);
}


void TestQueue()
{
  const int N = 100000;
  SPQueue<Label> queue;

  for (int i=0; i<N; i++) {
    if (i%2 == 0) {
      queue.push(Label(i, i+1));
    }
  }

  for (int i=0; i<N; i++) {
    if (i%2 != 0) {
      queue.push(Label(i, i+1));
    }
  }

  assert(queue.size() == N);
  assert(!queue.empty());

  for (int i=0; i<N; i++) {
    queue.push(Label(i, i+2));
  }

  assert(queue.size() == N);
  assert(!queue.empty());

  for (int i=0; i<N; i++) {
    queue.push(Label(i, i));
  }

  assert(queue.size() == N);
  assert(!queue.empty());

  std::vector<Label> labels;
  while (!queue.empty()) {
    labels.push_back(queue.top());
    queue.pop();
  }

  assert(labels.size() == N);
  assert(queue.size() == 0);
  assert(queue.empty());

  int i = 0;
  for (const auto& label : labels) {
    assert(label.id() == i);
    assert(label.sortcost() == i);
    i++;
  }

  for (const auto& label : labels) {
    queue.push(label);
  }
  queue.clear();
  assert(queue.size()==0);
  assert(queue.empty());
}


int main(int argc, char *argv[])
{
  SimpleTestQueue();
  TestQueue();
  std::cout << "all tests passed" << std::endl;
  return 0;
}
