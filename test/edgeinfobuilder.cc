#include "test.h"

#include "include/config.h"
#include "baldr/graphid.h"
#include "baldr/edgeinfo.h"
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

//void TryOpEqualTo(const EdgeInfoBuilder& eibuilder, const EdgeInfoBuilder& expected) {
//  if (!(expected == eibuilder))
//    throw runtime_error("OpEqualTo test failed");
//  if (!(eibuilder == expected))
//    throw runtime_error("OpEqualTo test failed");
//}
//
//void TestOpEqualTo() {
//  EdgeInfoBuilder
//  TryOpEqualTo(EdgeInfoBuilder(1.0f, 3.0f), EdgeInfoBuilder(1.0f, 3.0f));
//  TryOpEqualTo(EdgeInfoBuilder(4.0f, -2.0f), EdgeInfoBuilder(4.0f, -2.0f));
//  TryOpEqualTo(EdgeInfoBuilder(-4.0f, 2.0f), EdgeInfoBuilder(-4.0f, 2.0f));
//}

}

int main() {
  test::suite suite("edgeinfobuilder");

  // Set/Get nodea
  suite.test(TEST_CASE(TestSetGet_nodea));

  // Set/Get nodeb
  suite.test(TEST_CASE(TestSetGet_nodeb));

  // Op Equal To
//  suite.test(TEST_CASE(TestOpEqualTo));

  return suite.tear_down();
}
