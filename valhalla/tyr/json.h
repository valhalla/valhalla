#include <ostream>
#include <boost/variant.hpp>
#include <memory>
#include <string>
#include <cinttypes>
#include <cstddef>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <iomanip>

namespace valhalla {
namespace tyr {
namespace json {

//forward declare a little bit
class Jmap;
using MapPtr = std::shared_ptr<Jmap>;
class Jarray;
using ArrayPtr = std::shared_ptr<Jarray>;

//a variant of all the possible values to go with keys in json
using Value = boost::variant<std::string, uint64_t, int64_t, long double, bool, nullptr_t, MapPtr, ArrayPtr>;

//the map value type in json
class Jmap : public std::unordered_map<std::string, Value> {
 public:
  //just specialize unoredered_map
  using std::unordered_map<std::string, Value>::unordered_map;
  //and be able to spit out text
  friend std::ostream& operator<<(std::ostream& stream, const Jmap& json);
};

//the array value type in json
class Jarray : public std::vector<Value> {
 public:
  //just specialize vector
  using std::vector<Value>::vector;
 protected:
  //and be able to spit out text
  friend std::ostream& operator<<(std::ostream& stream, const Jarray& json);
};

//how we serialize the different primitives to string
class OstreamVisitor : public boost::static_visitor<std::ostream&>
{
 public:
  OstreamVisitor(std::ostream& o):ostream_(o){}

  std::ostream& operator()(const std::string& value) const {
    ostream_ << '"';
    //TODO: this may need to get more complicated
    for (const auto& c : value) {
      switch (c) {
      case '\\': ostream_ << "\\\\"; break;
      case '"': ostream_ << "\\\""; break;
      case '/': ostream_ << "\\/"; break;
      case '\b': ostream_ << "\\b"; break;
      case '\f': ostream_ << "\\f"; break;
      case '\n': ostream_ << "\\n"; break;
      case '\r': ostream_ << "\\r"; break;
      case '\t': ostream_ << "\\t"; break;
      default:
        if(c >= 0 && c < 32)
          ostream_ << "\\u" << std::hex << std::uppercase << std::setfill('0') << std::setw(4) << static_cast<int>(c);
        else
          ostream_ << c;
        break;
      }
    }
    return ostream_ << '"';
  }
  std::ostream& operator()(uint64_t value) const { return ostream_ << value; }
  std::ostream& operator()(int64_t value) const { return ostream_ << value; }
  std::ostream& operator()(long double value) const { return ostream_ << value; }
  std::ostream& operator()(bool value) const { return ostream_ << (value ? "true" : "false"); }
  std::ostream& operator()(nullptr_t value) const { return ostream_ << "null"; }
  std::ostream& operator()(const MapPtr& value) const { return ostream_ << *value; }
  std::ostream& operator()(const ArrayPtr& value) const { return ostream_ << *value; }
 private:
  std::ostream& ostream_;
};

std::ostream& operator<<(std::ostream& stream, const Jmap& json){
  stream << '{';
  bool seprator = false;
  for(const auto& key_value : json) {
    if(seprator)
      stream << ',';
    seprator = true;
    stream << '"' << key_value.first << "\":";
    boost::apply_visitor(OstreamVisitor(stream), key_value.second);
  }
  stream << '}';
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const Jarray& json){
  stream << '[';
  bool seprator = false;
  for(const auto& element : json) {
    if(seprator)
      stream << ',';
    seprator = true;
    boost::apply_visitor(OstreamVisitor(stream), element);
  }
  stream << ']';
  return stream;
}

MapPtr map(std::initializer_list<Jmap::value_type> list) {
  return MapPtr(new Jmap(list));
}

ArrayPtr array(std::initializer_list<Jarray::value_type> list) {
  return ArrayPtr(new Jarray(list));
}

}
}
}
