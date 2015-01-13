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

//base class so we can use pointers to refer to whichever
class json_object {
 public:
  virtual ~json_object(){};
 protected:
  //write yourself into a stream and return it
  virtual std::ostream& serialize(std::ostream& ostream) const = 0;
  //write the json object
  friend std::ostream& operator<<(std::ostream& stream, const json_object& json);
};

//slightly less annoying to type
using json_object_ptr = std::shared_ptr<json_object>;

//use normal stream operator to write these things
std::ostream& operator<<(std::ostream& stream, const json_object& json){
  return json.serialize(stream);
}

//how we serialize the different primitives to string
class ostream_visitor : public boost::static_visitor<std::ostream&>
{
 public:
  ostream_visitor(std::ostream& o):ostream_(o){}

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

  std::ostream& operator()(const json_object_ptr& value) const {
    return ostream_ << *value;
  }
 private:
  std::ostream& ostream_;
};

//a variant of all the possible values to go with keys in json
using json_type = boost::variant<std::string, uint64_t, int64_t, long double, bool, nullptr_t, json_object_ptr>;

//the map value type in json
class json_map:public json_object, public std::unordered_map<std::string, json_type> {
 public:
  //just specialize unoredered_map
  using std::unordered_map<std::string, json_type>::unordered_map;
  //and be able to spit out text
  virtual std::ostream& serialize(std::ostream& ostream) const {
    ostream << '{';
    bool seprator = false;
    for(const auto& key_value : *this) {
      if(seprator)
        ostream << ',';
      seprator = true;
      ostream << '"' << key_value.first << "\":";
      boost::apply_visitor(ostream_visitor(ostream), key_value.second);
    }
    ostream << '}';
    return ostream;
  }
 protected:
};

//the array value type in json
class json_array:public json_object, public std::vector<json_type> {
 public:
  //just specialize vector
  using std::vector<json_type>::vector;
 protected:
  //and be able to spit out text
  virtual std::ostream& serialize(std::ostream& ostream) const {
    ostream << '[';
    bool seprator = false;
    for(const auto& element : *this) {
      if(seprator)
        ostream << ',';
      seprator = true;
      boost::apply_visitor(ostream_visitor(ostream), element);
    }
    ostream << ']';
    return ostream;
  }
};


}
}
