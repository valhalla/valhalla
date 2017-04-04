#ifndef VALHALLA_BALDR_JSON_H_
#define VALHALLA_BALDR_JSON_H_

#include <ostream>
#include <boost/variant.hpp>
#include <memory>
#include <string>
#include <cinttypes>
#include <cstddef>
#include <unordered_map>
#include <list>
#include <sstream>
#include <iomanip>

namespace valhalla {
namespace baldr {
namespace json {

//forward declare a little bit
class Jmap;
using MapPtr = std::shared_ptr<Jmap>;
class Jarray;
using ArrayPtr = std::shared_ptr<Jarray>;
struct fp_t {
  long double value;
  size_t precision;
  //and be able to spit out text
  friend std::ostream& operator<<(std::ostream& stream, const fp_t&);
};

//a variant of all the possible values to go with keys in json
using Value = boost::variant<std::string, uint64_t, int64_t, fp_t, bool, std::nullptr_t, MapPtr, ArrayPtr>;

//the map value type in json
class Jmap : public std::unordered_map<std::string, Value> {
 public:
  //just specialize unoredered_map
  using std::unordered_map<std::string, Value>::unordered_map;
  //and be able to spit out text
  friend std::ostream& operator<<(std::ostream&, const Jmap&);
};

//the array value type in json
class Jarray : public std::list<Value> {
 public:
  //just specialize vector
  using std::list<Value>::list;
 protected:
  //and be able to spit out text
  friend std::ostream& operator<<(std::ostream&, const Jarray&);
};

//how we serialize the different primitives to string
class OstreamVisitor : public boost::static_visitor<std::ostream&>
{
 public:
  OstreamVisitor(std::ostream& o):ostream_(o), fill(o.fill()){}

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
        if(c >= 0 && c < 32) {
          //format changes for json hex
          ostream_.setf(std::ios::hex, std::ios::basefield);
          ostream_.setf(std::ios::uppercase);
          ostream_.fill('0');
          //output hex
          ostream_ << "\\u" << std::setw(4) << static_cast<int>(c);
          //tear down format changes
          ostream_.unsetf(std::ios::basefield);
          ostream_.unsetf(std::ios::uppercase);
          ostream_.fill(fill);
        }
        else
          ostream_ << c;
        break;
      }
    }
    return ostream_ << '"';
  }
  std::ostream& operator()(uint64_t value) const { return ostream_ << value; }
  std::ostream& operator()(int64_t value) const { return ostream_ << value; }
  std::ostream& operator()(fp_t value) const { return ostream_ << value; }
  std::ostream& operator()(bool value) const { return ostream_ << (value ? "true" : "false"); }
  std::ostream& operator()(std::nullptr_t value) const { return ostream_ << "null"; }
  std::ostream& operator()(const MapPtr& value) const { return ostream_ << *value; }
  std::ostream& operator()(const ArrayPtr& value) const { return ostream_ << *value; }
 private:
  std::ostream& ostream_;
  char fill;
};

inline std::ostream& operator<<(std::ostream& stream, const fp_t& fp){
  stream << std::setprecision(fp.precision) << std::fixed << fp.value;
  return stream;
}

inline std::ostream& operator<<(std::ostream& stream, const Jmap& json){
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

inline std::ostream& operator<<(std::ostream& stream, const Jarray& json){
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

inline MapPtr map(std::initializer_list<Jmap::value_type> list) {
  return MapPtr(new Jmap(list));
}

inline ArrayPtr array(std::initializer_list<Jarray::value_type> list) {
  return ArrayPtr(new Jarray(list));
}

}
}
}

#endif //VALHALLA_BALDR_JSON_H_
