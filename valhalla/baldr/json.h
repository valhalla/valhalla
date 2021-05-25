#ifndef VALHALLA_BALDR_JSON_H_
#define VALHALLA_BALDR_JSON_H_

#include <boost/variant.hpp>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <list>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace valhalla {
namespace baldr {
namespace json {

// forward declare a little bit
class Jmap;
using MapPtr = std::shared_ptr<Jmap>;
class Jarray;
using ArrayPtr = std::shared_ptr<Jarray>;

// Fixed precision serializer - outputs a fixed number of decimal places
struct fixed_t {
  long double value;
  size_t precision;
  // and be able to spit out text
  friend std::ostream& operator<<(std::ostream& stream, const fixed_t&);
};

// Floating point serializer - variable number of decimal places (no trailing zeros)
struct float_t {
  long double value;
  friend std::ostream& operator<<(std::ostream& stream, const float_t&);
};

struct RawJSON {
  std::string data;
};

// a variant of all the possible values to go with keys in json
using Value = boost::variant<std::string,
                             uint64_t,
                             int64_t,
                             fixed_t,
                             float_t,
                             bool,
                             std::nullptr_t,
                             MapPtr,
                             ArrayPtr,
                             RawJSON>;

// the map value type in json
class Jmap : public std::unordered_map<std::string, Value> {
public:
  // just specialize unordered_map
  using std::unordered_map<std::string, Value>::unordered_map;
  // and be able to spit out text
  friend std::ostream& operator<<(std::ostream&, const Jmap&);
};

// the array value type in json
class Jarray : public std::vector<Value> {
public:
  // just specialize vector
  using std::vector<Value>::vector;

protected:
  // and be able to spit out text
  friend std::ostream& operator<<(std::ostream&, const Jarray&);
};

// how we serialize the different primitives to string
class OstreamVisitor : public boost::static_visitor<std::ostream&> {
public:
  OstreamVisitor(std::ostream& o) : ostream_(o), fill(o.fill()) {
  }

  std::ostream& operator()(const std::string& value) const {
    ostream_ << '"';
    // TODO: this may need to get more complicated
    for (const auto& c : value) {
      switch (c) {
        case '\\':
          ostream_ << "\\\\";
          break;
        case '"':
          ostream_ << "\\\"";
          break;
        case '/':
          ostream_ << "\\/";
          break;
        case '\b':
          ostream_ << "\\b";
          break;
        case '\f':
          ostream_ << "\\f";
          break;
        case '\n':
          ostream_ << "\\n";
          break;
        case '\r':
          ostream_ << "\\r";
          break;
        case '\t':
          ostream_ << "\\t";
          break;
        default:
          if (c >= 0 && c < 32) {
            // format changes for json hex
            ostream_.setf(std::ios::hex, std::ios::basefield);
            ostream_.setf(std::ios::uppercase);
            ostream_.fill('0');
            // output hex
            ostream_ << "\\u" << std::setw(4) << static_cast<int>(c);
            // tear down format changes
            ostream_.unsetf(std::ios::basefield);
            ostream_.unsetf(std::ios::uppercase);
            ostream_.fill(fill);
          } else {
            ostream_ << c;
          }
          break;
      }
    }
    return ostream_ << '"';
  }
  std::ostream& operator()(uint64_t value) const {
    return ostream_ << value;
  }
  std::ostream& operator()(int64_t value) const {
    return ostream_ << value;
  }
  std::ostream& operator()(fixed_t value) const {
    return ostream_ << value;
  }
  std::ostream& operator()(float_t value) const {
    return ostream_ << value;
  }
  std::ostream& operator()(bool value) const {
    return ostream_ << (value ? "true" : "false");
  }
  std::ostream& operator()(std::nullptr_t) const {
    return ostream_ << "null";
  }

  std::ostream& operator()(const RawJSON& value) const {
    return ostream_ << value.data;
  }

  template <class Nullable> std::ostream& operator()(const Nullable& value) const {
    if (value) {
      return ostream_ << *value;
    } else {
      return this->operator()(nullptr);
    }
  }

private:
  std::ostream& ostream_;
  char fill;
};

inline std::ostream& operator<<(std::ostream& stream, const fixed_t& fp) {
  if (std::isfinite(fp.value)) {
    stream << std::setprecision(fp.precision) << std::fixed << fp.value;
  } else {
    stream << std::setprecision(fp.precision) << std::fixed << '"' << fp.value << '"';
  }
  return stream;
}

inline std::ostream& operator<<(std::ostream& stream, const float_t& fp) {
  // precision defaults to 6 according to lib stdc++
  if (std::isfinite(fp.value)) {
    stream << std::setprecision(6) << std::defaultfloat << fp.value;
  } else {
    stream << std::setprecision(6) << std::defaultfloat << '"' << fp.value << '"';
  }
  return stream;
}

template <typename Visitable>
inline void applyOutputVisitor(std::ostream& stream, Visitable& visitable) {
  // Cannot use boost::apply_visitor with C++14 due to
  // ambiguity introduced in boost_1_58 and fixed in the latest
  // https://lists.boost.org/boost-bugs/2015/05/41067.php
  OstreamVisitor visitor(stream);
  visitable.apply_visitor(visitor);
}

inline std::ostream& operator<<(std::ostream& stream, const Jmap& json) {
  stream << '{';
  bool seprator = false;
  for (const auto& key_value : json) {
    if (seprator) {
      stream << ',';
    }
    seprator = true;
    stream << '"' << key_value.first << "\":";
    applyOutputVisitor(stream, key_value.second);
  }
  stream << '}';
  return stream;
}

inline std::ostream& operator<<(std::ostream& stream, const Jarray& json) {
  stream << '[';
  bool seprator = false;
  for (const auto& element : json) {
    if (seprator) {
      stream << ',';
    }
    seprator = true;
    applyOutputVisitor(stream, element);
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

} // namespace json
} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_JSON_H_
