// -*- mode: c++ -*-
#include <iostream>
#include <cstdlib>
#include <limits>

#include "meili/priority_queue.h"


class Label
{
 public:
  using id_type = uint32_t;

  Label(id_type id, double cost)
      : id_(id), cost_(cost) {}

  id_type id() const
  { return id_; }

  double sortcost() const
  { return cost_; }

 private:
  id_type id_;
  double cost_;
};


bool operator>(const Label& lhs, const Label& rhs)
{ return lhs.sortcost() > rhs.sortcost(); }


bool operator<(const Label& lhs, const Label& rhs)
{ return lhs.sortcost() < rhs.sortcost(); }


bool operator==(const Label& lhs, const Label& rhs)
{ return lhs.sortcost() == rhs.sortcost(); }


bool operator!=(const Label& lhs, const Label& rhs)
{ return !(lhs == rhs); }


inline void
simple_assert(bool assertion, const std::string& message)
{
  if (!assertion) {
    throw std::logic_error(message);
  }
}


inline void
simple_assert(bool assertion)
{ simple_assert(assertion, "assertion failed"); }


void SimpleTestQueue()
{
  SPQueue<Label> queue;
  simple_assert(queue.size() == 0 && queue.empty());

  queue.push(Label(1, 3));
  simple_assert(queue.top() == Label(1, 3));
  simple_assert(queue.size() == 1);

  // test compressions
  simple_assert(queue.top() == Label(2, 3));
  simple_assert(queue.top() > Label(2, 2));
  simple_assert(queue.top() < Label(2, 4));
  simple_assert(queue.top() != Label(2, 4));

  queue.push(Label(2, 2));
  simple_assert(queue.top() == Label(2, 2));
  simple_assert(queue.size() == 2);

  queue.push(Label(2, 4));
  simple_assert(queue.top() == Label(2, 2));
  simple_assert(queue.size() == 2);

  queue.push(Label(2, 2));
  simple_assert(queue.top() == Label(2, 2));
  simple_assert(queue.size() == 2);

  queue.push(Label(1, 1));
  simple_assert(queue.top() == Label(1, 1));
  simple_assert(queue.size() == 2);

  queue.pop();
  simple_assert(queue.top() == Label(2, 2));
  simple_assert(queue.size() == 1);

  queue.pop();
  simple_assert(queue.empty() && queue.size() == 0);
}


void TestQueue()
{
  constexpr int N = 100000;
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

  simple_assert(queue.size() == N);
  simple_assert(!queue.empty());

  for (int i=0; i<N; i++) {
    queue.push(Label(i, i+2));
  }

  simple_assert(queue.size() == N);
  simple_assert(!queue.empty());

  for (int i=0; i<N; i++) {
    queue.push(Label(i, i));
  }

  simple_assert(queue.size() == N);
  simple_assert(!queue.empty());

  std::vector<Label> labels;
  while (!queue.empty()) {
    labels.push_back(queue.top());
    queue.pop();
  }

  simple_assert(labels.size() == N);
  simple_assert(queue.size() == 0);
  simple_assert(queue.empty());

  uint32_t i = 0;
  for (const auto& label : labels) {
    simple_assert(label.id() == i);
    simple_assert(label.sortcost() == i);
    i++;
  }

  for (const auto& label : labels) {
    queue.push(label);
  }
  queue.clear();
  simple_assert(queue.size()==0);
  simple_assert(queue.empty());
}


void TestSorting()
{
  SPQueue<Label> queue;
  constexpr Label::id_type N = 10000;

  for (Label::id_type id = 0; id < N; id++) {
    const double cost = static_cast<double>(std::rand());
    // id is insignificant in this test
    queue.push(Label(id, cost));
  }

  simple_assert(queue.size() == N);

  // Should be sorted
  Label previous_label(N + 1, -std::numeric_limits<double>::infinity());
  while (!queue.empty()) {
    const auto label = queue.top();
    queue.pop();
    simple_assert(previous_label.sortcost() <= label.sortcost());
    previous_label = label;
  }
}


int main(int argc, char *argv[])
{
  SimpleTestQueue();

  TestQueue();

  TestSorting();

  std::cout << "all tests passed" << std::endl;

  return 0;
}
