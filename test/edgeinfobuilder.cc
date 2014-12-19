#include "test.h"

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/edgeinfo.h>
#include "mjolnir/edgeinfobuilder.h"

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

void TrySetGet_nodea(const GraphId& nodea,
                     const GraphId& expected) {
  EdgeInfoBuilder eibuilder;
  eibuilder.set_nodea(nodea);
  if (!(expected == eibuilder.nodea()))
    throw runtime_error("SetGet_nodea test failed");
}

void TestSetGet_nodea() {
  TrySetGet_nodea(GraphId(10, 5, 2), GraphId(10, 5, 2));
}

void TrySetGet_nodeb(const GraphId& nodeb,
                     const GraphId& expected) {
  EdgeInfoBuilder eibuilder;
  eibuilder.set_nodeb(nodeb);
  if (!(expected == eibuilder.nodeb()))
    throw runtime_error("SetGet_nodeb test failed");
}

void TestSetGet_nodeb() {
  TrySetGet_nodeb(GraphId(10, 5, 2), GraphId(10, 5, 2));
}

void TryOpEqualTo(const EdgeInfoBuilder& eibuilder, const EdgeInfoBuilder& expected) {
  if (!(expected == eibuilder))
    throw runtime_error("OpEqualTo test failed");
  if (!(eibuilder == expected))
    throw runtime_error("OpEqualTo test failed");
}

void TestOpEqualTo() {
  EdgeInfoBuilder lhs;
  EdgeInfoBuilder rhs;
  lhs.set_nodea(GraphId(10, 1, 2));
  lhs.set_nodeb(GraphId(10, 1, 4));
  rhs.set_nodea(GraphId(10, 1, 2));
  rhs.set_nodeb(GraphId(10, 1, 4));
  TryOpEqualTo(lhs, rhs);
  lhs.set_nodea(GraphId(10, 1, 2));
  lhs.set_nodeb(GraphId(10, 1, 4));
  rhs.set_nodea(GraphId(10, 1, 4));
  rhs.set_nodeb(GraphId(10, 1, 2));
  TryOpEqualTo(lhs, rhs);
}

}

int main() {
  test::suite suite("edgeinfobuilder");

  // Set/Get nodea
  suite.test(TEST_CASE(TestSetGet_nodea));

  // Set/Get nodeb
  suite.test(TEST_CASE(TestSetGet_nodeb));

  // Op Equal To
  suite.test(TEST_CASE(TestOpEqualTo));

  return suite.tear_down();
}
