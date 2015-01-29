#include "test.h"

#include <vector>
#include <iostream>
#include <fstream>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/exitsign.h>
#include "mjolnir/edgeinfobuilder.h"

#include <boost/shared_array.hpp>
#include <memory>

using namespace std;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

namespace {

boost::shared_array<char> ToFileAndBack(const EdgeInfoBuilder& eibuilder) {
  // write EdgeInfoBuilder to binary file
  std::ofstream out_file("EdgeInfoBuilder_TestWriteRead.gph",
                         std::ios::out | std::ios::binary | std::ios::ate);
  if (out_file.is_open()) {
    out_file << eibuilder;
    out_file.close();
  } else {
    throw runtime_error("Failed to open file for writing");
  }

  // read EdgeInfo from binary file
  streampos size;
  boost::shared_array<char> memblock;

  std::ifstream in_file("EdgeInfoBuilder_TestWriteRead.gph",
                        std::ios::in | std::ios::binary | std::ios::ate);
  if (in_file.is_open()) {
    size = in_file.tellg();
    memblock.reset(new char[size]);
    in_file.seekg(0, ios::beg);
    in_file.read(memblock.get(), size);
    in_file.close();
  }

  return memblock;
}

void TestWriteRead() {
  // Make a builder to write the info to disk
  EdgeInfoBuilder eibuilder;

  // Name
  std::vector<uint32_t> street_name_offset_list;
  street_name_offset_list.push_back(963);
  street_name_offset_list.push_back(957);
  street_name_offset_list.push_back(862);
  eibuilder.set_street_name_offset_list(street_name_offset_list);

  // Shape
  std::vector<PointLL> shape;
  shape.push_back(PointLL(-76.3002, 40.0433));
  shape.push_back(PointLL(-76.3036, 40.043));
  eibuilder.set_shape(shape);

  // Exit signs
  std::vector<ExitSignBuilder> exit_signs;
  exit_signs.emplace_back(ExitSign::Type::kNumber, 100);
  exit_signs.emplace_back(ExitSign::Type::KBranch, 200);
  exit_signs.emplace_back(ExitSign::Type::kToward, 300);
  exit_signs.emplace_back(ExitSign::Type::kName, 400);
  std::vector<ExitSignBuilder> es(exit_signs);  // copy for compare below
  eibuilder.set_exit_signs(std::move(exit_signs));

  // Make an edge info object from the memory
  boost::shared_array<char> memblock = ToFileAndBack(eibuilder);
  std::unique_ptr<EdgeInfo> ei(new EdgeInfo(memblock.get()));

  //TODO: errors thrown should say what was found and what was expected

  // Validate the read in fields to the original EdgeInfoBuilder
  if (!(street_name_offset_list.size() == ei->name_count()))
    throw runtime_error("WriteRead:name_count test failed");
  if (!(shape.size() == ei->shape().size()))
    throw runtime_error("WriteRead:shape_count test failed");
  if (!(es.size() == ei->exit_sign_count()))
    throw runtime_error("WriteRead:exit_sign_count test failed");

  // Check the name indices
  for (uint8_t i = 0; i < ei->name_count(); ++i) {
    if (!(street_name_offset_list[i] == ei->GetStreetNameOffset(static_cast<uint8_t>(i))))
      throw runtime_error("WriteRead:GetStreetNameOffset test failed");
  }

  // Check the shape points
  for (size_t i = 0; i < ei->shape().size(); ++i) {
    if (!shape[i].ApproximatelyEqual(ei->shape()[i]))
      throw runtime_error("WriteRead:shape test failed");
  }

  // Check the exits
  for (size_t i = 0; i < ei->exit_sign_count(); ++i) {
    const ExitSign* esp = ei->GetExitSign(static_cast<uint8_t>(i));
    if (!(es[i].type() == esp->type()))
      throw runtime_error("WriteRead:GetExitSign.type test failed");
    if (!(es[i].text_offset() == esp->text_offset()))
      throw runtime_error("WriteRead:GetExitSign.text_offset test failed");
  }
}

}

int main() {
  test::suite suite("edgeinfobuilder");

  // Write to file and read into EdgeInfo
  suite.test(TEST_CASE(TestWriteRead));

  return suite.tear_down();
}
