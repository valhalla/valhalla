#include <ostream>
#include <boost/variant.hpp>
#include <memory>
#include <string>
#include <cinttypes>
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace valhalla {
namespace tyr {
namespace json {

//base class so we can use pointers to refer to whichever
class JsonObject {
 public:
  virtual ~JsonObject(){};
 protected:
  //write yourself into a stream and return it
  virtual std::ostream& Serialize(std::ostream& ostream) const = 0;
  //write the json object
  friend std::ostream& operator<<(std::ostream& stream, const JsonObject& json);
};

//slightly less annoying to type
using JsonObjectPtr = std::shared_ptr<JsonObject>;

//use normal stream operator to write these things
std::ostream& operator<<(std::ostream& stream, const JsonObject& json){
  return json.Serialize(stream);
}

//how we serialize the different primitives to string
class OstreamVisitor : public boost::static_visitor<std::ostream&>
{
 public:
  OstreamVisitor(std::ostream& o):ostream_(o){}

  std::ostream& operator()(const std::string& value) const {
    return ostream_ << '"' << value << '"';
  }

  std::ostream& operator()(uint64_t value) const {
    return ostream_ << value;
  }

  std::ostream& operator()(int64_t value) const {
    return ostream_ << value;
  }

  std::ostream& operator()(long double value) const {
    return ostream_ << value;
  }

  std::ostream& operator()(bool value) const {
    return ostream_ << (value ? "true" : "false");
  }

  std::ostream& operator()(nullptr_t value) const {
    return ostream_ << "null";
  }

  std::ostream& operator()(const JsonObjectPtr& value) const {
    return ostream_ << *value;
  }
 private:
  std::ostream& ostream_;
};

//a variant of all the possible values to go with keys in json
using JsonType = boost::variant<std::string, uint64_t, int64_t, long double, bool, nullptr_t, JsonObjectPtr>;

//the map value type in json
class JsonMap:public JsonObject, public std::unordered_map<std::string, JsonType> {
 public:
  //just specialize unoredered_map
  using std::unordered_map<std::string, JsonType>::unordered_map;
  //and be able to spit out text
  virtual std::ostream& Serialize(std::ostream& ostream) const {
    ostream << '{';
    bool seprator = false;
    for(const auto& key_value : *this) {
      if(seprator)
        ostream << ',';
      seprator = true;
      ostream << '"' << key_value.first << "\":";
      boost::apply_visitor(OstreamVisitor(ostream), key_value.second);
    }
    ostream << '}';
    return ostream;
  }
 protected:
};

//the array value type in json
class JsonArray:public JsonObject, public std::vector<JsonType> {
 public:
  //just specialize vector
  using std::vector<JsonType>::vector;
 protected:
  //and be able to spit out text
  virtual std::ostream& Serialize(std::ostream& ostream) const {
    ostream << '[';
    bool seprator = false;
    for(const auto& element : *this) {
      if(seprator)
        ostream << ',';
      seprator = true;
      boost::apply_visitor(OstreamVisitor(ostream), element);
    }
    ostream << ']';
    return ostream;
  }
};

//slightly less annoying to type
using JsonMapPtr = std::shared_ptr<JsonMap>;
using JsonArrayPtr = std::shared_ptr<JsonArray>;

JsonMapPtr map(std::initializer_list<JsonMap::value_type> list) {
  return JsonMapPtr(new JsonMap(list));
}

JsonArrayPtr array(std::initializer_list<JsonArray::value_type> list) {
  return JsonArrayPtr(new JsonArray(list));
}

}
}
}
