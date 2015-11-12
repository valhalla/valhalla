#include "midgard/grid.h"
#include "midgard/point2.h"
#include "midgard/pointll.h"
#include "test.h"

#include <list>
#include <string>
#include <random>

using namespace valhalla::midgard;

namespace {

  void assert_answer(const grid<Point2>& g, const std::list<Point2>& l, const std::unordered_set<size_t>& cells, bool uncontained) {
    bool uc_answer;
    auto answer = g.intersect(l, uc_answer);
    if(uc_answer != uncontained)
      throw std::logic_error("Expected shape to " + std::string(uncontained ? "leave" : "stay in") + " the grid");
    if(answer != cells)
      throw std::logic_error("Expected a different set of intersecting cells");
  }

  void test_intersect_linestring() {
    grid<Point2> g(AABB2<Point2>{-1,-1,1,1}, 5);
    assert_answer(g, {}, std::unordered_set<size_t>{}, false);
    assert_answer(g, { {-.9,0}, {.9,0} }, {10,11,12,13,14}, false);
    assert_answer(g, { {-2,0}, {2,0} }, {10,11,12,13,14}, true);
    assert_answer(g, { {-.9,0}, {-2,0} }, {10}, true);
    assert_answer(g, { {-.9,.9} }, {20}, false);
    assert_answer(g, { {.9,-.9} }, {4}, false);
    assert_answer(g, { {.9,-1.1}, {.9, .9} }, {4, 9, 14, 19, 24}, true);
  }

  void test_intersect_circle() {
    grid<Point2> g(AABB2<Point2>{-1,-1,1,1}, 5);
    //TODO:
  }

  void test_random_linestring() {
    grid<Point2> g(AABB2<Point2>{-1,-1,1,1}, 5);
    std::default_random_engine generator;
    std::uniform_real_distribution<> distribution(-10, 10);
    for(int i = 0; i < 10000; ++i) {
      std::vector<Point2> linestring;
      for(int j = 0; j < 100; ++j)
        linestring.emplace_back(PointLL(distribution(generator), distribution(generator)));
      bool leaves;
      auto answer = g.intersect(linestring, leaves);
      for(auto a : answer)
        if(a > 24)
          throw std::runtime_error("Non-existant cell!");
    }
  }

  void test_random_circle() {
    grid<Point2> g(AABB2<Point2>{-1,-1,1,1}, 5);
    //TODO:
  }

}

int main() {
  test::suite suite("grid");

  suite.test(TEST_CASE(test_intersect_linestring));
  suite.test(TEST_CASE(test_intersect_circle));
  suite.test(TEST_CASE(test_random_linestring));
  suite.test(TEST_CASE(test_random_circle));

  return suite.tear_down();
}
