#include <cstdint>
// -*- mode: c++ -*-
#include <cstdlib>
#include <limits>

#include "meili/priority_queue.h"
#include "test.h"

class Label {
public:
  using id_type = uint32_t;

  Label(id_type id, double cost) : id_(id), cost_(cost) {
  }

  id_type id() const {
    return id_;
  }

  double sortcost() const {
    return cost_;
  }

private:
  id_type id_;
  double cost_;
};

bool operator>(const Label& lhs, const Label& rhs) {
  return lhs.sortcost() > rhs.sortcost();
}

bool operator<(const Label& lhs, const Label& rhs) {
  return lhs.sortcost() < rhs.sortcost();
}

bool operator==(const Label& lhs, const Label& rhs) {
  return lhs.sortcost() == rhs.sortcost();
}

bool operator!=(const Label& lhs, const Label& rhs) {
  return !(lhs == rhs);
}

void SimpleTestQueue() {
  SPQueue<Label> queue;
  test::assert_bool(queue.size() == 0 && queue.empty(), "initial queue should be empty");

  queue.push(Label(1, 3));
  test::assert_bool(queue.top() == Label(1, 3), "top should be <1 3>");
  test::assert_bool(queue.size() == 1, "queue should have one label");

  // test compressions
  test::assert_bool(queue.top() == Label(2, 3), "should be equal");
  test::assert_bool(queue.top() > Label(2, 2), "should be larger");
  test::assert_bool(queue.top() < Label(2, 4), "should be smaller");
  test::assert_bool(queue.top() != Label(2, 4), "should not be equal");

  queue.push(Label(2, 2));
  test::assert_bool(queue.top() == Label(2, 2), "top should be <2, 2>");
  test::assert_bool(queue.size() == 2, "queue should have 2 labels now");

  queue.push(Label(2, 4));
  test::assert_bool(queue.top() == Label(2, 2), "top should still be <2, 2>");
  test::assert_bool(queue.size() == 2, "queue size should not change");

  queue.push(Label(2, 2));
  test::assert_bool(queue.top() == Label(2, 2), "top should still be <2, 2>");
  test::assert_bool(queue.size() == 2, "nothing should change");

  queue.push(Label(1, 1));
  test::assert_bool(queue.top() == Label(1, 1), "top should be changed now");
  test::assert_bool(queue.size() == 2, "the old lable 1 should be replaced so size should be 2");

  queue.pop();
  test::assert_bool(queue.top() == Label(2, 2), "<2, 2> should be popped");
  test::assert_bool(queue.size() == 1, "now there should be only one label");

  queue.pop();
  test::assert_bool(queue.empty() && queue.size() == 0, "nothing should be left");
}

void TestQueue() {
  constexpr int N = 100000;
  SPQueue<Label> queue;

  for (int i = 0; i < N; i++) {
    if (i % 2 == 0) {
      queue.push(Label(i, i + 1));
    }
  }

  for (int i = 0; i < N; i++) {
    if (i % 2 != 0) {
      queue.push(Label(i, i + 1));
    }
  }

  test::assert_bool(queue.size() == N, "all should be pushed");
  test::assert_bool(!queue.empty(), "definitely should be non-empty");

  for (int i = 0; i < N; i++) {
    queue.push(Label(i, i + 2));
  }

  test::assert_bool(queue.size() == N, "size should not be changes since no new id introduced");
  test::assert_bool(!queue.empty(), "definitely should be non-empty");

  std::vector<Label> labels;
  while (!queue.empty()) {
    labels.push_back(queue.top());
    queue.pop();
  }

  test::assert_bool(labels.size() == N, "all labels should be popped");
  test::assert_bool(queue.size() == 0, "now queue should be empty");
  test::assert_bool(queue.empty(), "now queue should be empty");

  uint32_t i = 0;
  for (const auto& label : labels) {
    test::assert_bool(label.id() == i, "id should be matched");
    test::assert_bool(label.sortcost() == i + 1, "sortcost should be matched");
    i++;
  }

  for (const auto& label : labels) {
    queue.push(label);
  }
  queue.clear();
  test::assert_bool(queue.size() == 0, "nothing should be left");
  test::assert_bool(queue.empty(), "should be empty");
}

void TestSorting() {
  SPQueue<Label> queue;
  constexpr Label::id_type N = 10000;

  for (Label::id_type id = 0; id < N; id++) {
    const double cost = static_cast<double>(std::rand());
    // id is insignificant in this test
    queue.push(Label(id, cost));
  }

  test::assert_bool(queue.size() == N, "all labels should be pushed");

  // Should be sorted
  Label previous_label(N + 1, -std::numeric_limits<double>::infinity());
  while (!queue.empty()) {
    const auto label = queue.top();
    queue.pop();
    test::assert_bool(previous_label.sortcost() <= label.sortcost(), "should be sorted");
    previous_label = label;
  }
}

int main(int argc, char* argv[]) {
  test::suite suite("queue");

  suite.test(TEST_CASE(SimpleTestQueue));

  suite.test(TEST_CASE(TestQueue));

  suite.test(TEST_CASE(TestSorting));

  return suite.tear_down();
}
