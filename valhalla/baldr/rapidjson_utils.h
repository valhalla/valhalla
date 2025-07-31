#ifndef VALHALLA_BALDR_RAPIDJSON_UTILS_H_
#define VALHALLA_BALDR_RAPIDJSON_UTILS_H_

#include <valhalla/baldr/rapidjson_fwd.h>

#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <rapidjson/allocators.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/pointer.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/rapidjson.h>
#include <rapidjson/schema.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <fstream>
#include <istream>
#include <locale>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace {

// helper to only evaluate a constexpr once a type is known
template <class... T> constexpr bool always_false = false;

template <typename T>
constexpr bool is_string_like_v =
    std::is_constructible_v<std::string, std::remove_cv_t<std::remove_reference_t<T>>>;
} // namespace

namespace rapidjson {

inline std::string to_string(const rapidjson::Value& value, int decimal_places = -1) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  if (decimal_places >= 0)
    writer.SetMaxDecimalPlaces(decimal_places);
  value.Accept(writer);
  return std::string(buffer.GetString(), buffer.GetSize());
}

/* NOTE: all of these helper functions use the dom traversal style syntax for accessing nested keys
 * what this means is that every path you provide has to begin with '/' and allows for multiple to
 * get at children these functions don't currently check for '/' because rapidjson will throw when
 * its not found
 */

// if you dont want an arithmetic type dont try any lexical casting
template <typename T, typename V>
inline typename std::enable_if<!std::is_arithmetic<T>::value, boost::optional<T>>::type
get_optional(V&& v, const char* source) {
  // if we dont have this key bail
  auto* ptr = rapidjson::Pointer{source}.Get(std::forward<V>(v));
  if (!ptr) {
    return boost::none;
  }
  // if its the exact right type give it back
  if (ptr->template Is<T>()) {
    return ptr->template Get<T>();
  }
  // give up
  return boost::none;
}

// if you do want an arithmetic type dont try lexical casting as a last resort
template <typename T, typename V>
inline typename std::enable_if<std::is_arithmetic<T>::value, boost::optional<T>>::type
get_optional(V&& v, const char* source) {
  // if we dont have this key bail
  auto* ptr = rapidjson::Pointer{source}.Get(std::forward<V>(v));
  if (!ptr) {
    return boost::none;
  }
  // if its the exact right type give it back
  if (ptr->template Is<T>()) {
    return ptr->template Get<T>();
  }
  // try to convert from a string
  if (ptr->IsString()) {
    try {
      return boost::lexical_cast<T>(ptr->template Get<std::string>());
    } catch (...) {}
  }
  // numbers are strict in rapidjson but we don't want that strictness because it aborts the program
  // (wtf?)
  if (ptr->IsBool()) {
    return static_cast<T>(ptr->GetBool());
  }
  if (ptr->IsInt()) {
    return static_cast<T>(ptr->GetInt());
  }
  if (ptr->IsUint()) {
    return static_cast<T>(ptr->GetUint());
  }
  if (ptr->IsInt64()) {
    return static_cast<T>(ptr->GetInt64());
  }
  if (ptr->IsUint64()) {
    return static_cast<T>(ptr->GetUint64());
  }
  if (ptr->IsDouble()) {
    return static_cast<T>(ptr->GetDouble());
  }
  // give up
  return boost::none;
}

template <typename T, typename V> inline T get(V&& v, const char* source, const T& t) {
  auto value = get_optional<T>(v, source);
  if (!value) {
    return t;
  }
  return *value;
}

template <typename T, typename V> inline T get(V&& v, const char* source) {
  auto value = get_optional<T>(v, source);
  if (!value) {
    throw std::runtime_error(std::string("No member: ") + source);
  }
  return *value;
}

template <typename V> inline const rapidjson::Value& get_child(const V& v, const char* source) {
  const rapidjson::Value* ptr = rapidjson::Pointer{source}.Get(v);
  if (!ptr) {
    throw std::runtime_error(std::string("No child: ") + source);
  }
  return *ptr;
}

template <typename V>
inline const rapidjson::Value& get_child(const V& v, const char* source, const rapidjson::Value& t) {
  const rapidjson::Value* ptr = rapidjson::Pointer{source}.Get(v);
  if (!ptr) {
    return t;
  }
  return *ptr;
}

template <typename V> inline rapidjson::Value& get_child(V&& v, const char* source) {
  rapidjson::Value* ptr = rapidjson::Pointer{source}.Get(std::forward<V>(v));
  if (!ptr) {
    throw std::runtime_error(std::string("No child: ") + source);
  }
  return *ptr;
}

template <typename V>
inline boost::optional<const rapidjson::Value&> get_child_optional(const V& v, const char* source) {
  boost::optional<const rapidjson::Value&> c;
  const rapidjson::Value* ptr = rapidjson::Pointer{source}.Get(v);
  if (ptr) {
    c.reset(*ptr);
  }
  return c;
}

template <typename V>
inline boost::optional<rapidjson::Value&> get_child_optional(V&& v, const char* source) {
  boost::optional<rapidjson::Value&> c;
  rapidjson::Value* ptr = rapidjson::Pointer{source}.Get(std::forward<V>(v));
  if (ptr) {
    c.reset(*ptr);
  }
  return c;
}

template <class Ptree> void add_value(const Value& v, Ptree& pt) {
  switch (v.GetType()) {
    case kObjectType:
      add_object(v.GetObject(), pt);
      break;
    case kArrayType:
      add_array(v.GetArray(), pt);
      break;
    case kNullType:
      pt.put("", "null");
      break;
    case kFalseType:
      pt.put("", false);
      break;
    case kTrueType:
      pt.put("", true);
      break;
    case kStringType:
      pt.put("", v.GetString());
      break;
    case kNumberType:
      if (v.IsInt64())
        pt.put("", v.GetInt64());
      else if (v.IsUint64())
        pt.put("", v.GetUint64());
      else if (v.IsDouble())
        pt.put("", v.GetDouble());
      else
        throw std::runtime_error("unhandled number");
      break;
  }
}

template <class Ptree> void add_object(const GenericObject<true, Value::ValueType>& o, Ptree& pt) {
  for (const auto& e : o)
    add_value(e.value, pt.add_child(e.name.GetString(), Ptree{}));
}

template <class Ptree> void add_array(const GenericArray<true, Value::ValueType>& a, Ptree& pt) {
  for (const auto& e : a)
    add_value(e, pt.push_back(std::make_pair("", Ptree{}))->second);
}

template <class Ptree>
void read_json(std::basic_istream<typename Ptree::key_type::value_type>& stream, Ptree& pt) {
  Document d;
  IStreamWrapper wrapper(stream);
  d.ParseStream(wrapper);
  if (d.HasParseError())
    throw std::runtime_error("Could not parse json, error at offset: " +
                             std::to_string(d.GetErrorOffset()));
  if (d.IsObject())
    add_object(const_cast<const Document*>(&d)->GetObject(), pt);
  else if (d.IsArray())
    add_array(const_cast<const Document*>(&d)->GetArray(), pt);
  else
    throw std::runtime_error("Json is not an object or array");
}

template <class Ptree>
void read_json(const std::string& filename, Ptree& pt, const std::locale& loc = std::locale()) {
  std::basic_ifstream<typename Ptree::key_type::value_type> stream(filename);
  if (!stream)
    throw std::runtime_error("Cannot open file " + filename);
  stream.imbue(loc);
  rapidjson::read_json(stream, pt);
}

inline std::string serialize(const rapidjson::Document& doc) {
  rapidjson::StringBuffer buffer;
  rapidjson::PrettyWriter<rapidjson::StringBuffer, rapidjson::Document::EncodingType,
                          rapidjson::ASCII<>>
      writer(buffer);
  doc.Accept(writer);
  return buffer.GetString();
}

inline Document read_json(std::ifstream& stream) {
  Document d;
  IStreamWrapper wrapper(stream);
  d.ParseStream(wrapper);
  if (d.HasParseError())
    throw std::runtime_error("Could not parse json, error at offset: " +
                             std::to_string(d.GetErrorOffset()));
  if (!d.IsObject() && !d.IsArray())
    throw std::runtime_error("Json is not an object or array");
  return d;
}

inline Document read_json(const std::string& filename, const std::locale& loc = std::locale()) {
  std::ifstream stream(filename);
  if (!stream)
    throw std::runtime_error("Cannot open file " + filename);
  stream.imbue(loc);
  return rapidjson::read_json(stream);
}

/**
 * A convenience class to the writer stringbuffer rapidjson pattern. This takes whatever
 * data you want and serializes it directly to a json string rather than keeping it organized
 * as json in an in memory object. If you are familiar with XML parsing/writing this is analogous
 * to a SAX style writer/generator for json, as opposed to DOM (object model in memory) style
 *
 * Rapidjson's write is pretty complete but its quite verbose for common operations like
 * adding a key value pair. Because of that we have a small wrapper here to make it less verbose
 */
class writer_wrapper_t {
protected:
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<decltype(buffer)> writer;

public:
  writer_wrapper_t(size_t reservation = 0) : buffer(), writer(buffer) {
    if (reservation != 0)
      buffer.Reserve(reservation);
  }

  inline void start_object() {
    writer.StartObject();
  }

  inline void start_object(const char* name) {
    writer.String(name);
    writer.StartObject();
  }

  inline void start_array() {
    writer.StartArray();
  }

  inline void start_array(const char* name) {
    writer.String(name);
    writer.StartArray();
  }

  inline void end_object() {
    writer.EndObject();
  }

  inline void end_array() {
    writer.EndArray();
  }

  inline const char* get_buffer() const {
    return buffer.GetString();
  }

  inline void set_precision(int precision) {
    writer.SetMaxDecimalPlaces(precision);
  }

  template <typename K, typename V> inline void operator()(const K& key, const V& value) {
    if constexpr (is_string_like_v<K>) {
      writer.String(key);
    } else {
      static_assert(always_false<K>, "Unsupported key type");
    }

    if constexpr (std::is_same_v<V, int> || std::is_same_v<V, int32_t> || std::is_same_v<V, long>) {
      writer.Int64(static_cast<int64_t>(value));
    } else if constexpr (std::is_same_v<V, unsigned int> || std::is_same_v<V, uint32_t> ||
                         std::is_same_v<V, unsigned long>) {
      writer.Uint64(static_cast<uint64_t>(value));
    } else if constexpr (std::is_same_v<V, uint64_t> || std::is_same_v<V, unsigned long long>) {
      writer.Uint64(value);
    } else if constexpr (std::is_same_v<V, int64_t> || std::is_same_v<V, long long>) {
      writer.Int64(value);
    } else if constexpr (std::is_same_v<V, double> || std::is_same_v<V, float>) {
      writer.Double(value);
    } else if constexpr (std::is_same_v<V, bool>) {
      writer.Bool(value);
    } else if constexpr (std::is_same_v<V, std::nullptr_t>) {
      writer.Null();
    } else if constexpr (is_string_like_v<V>) {
      writer.String(value);
    } else {
      static_assert(always_false<V>, "Unsupported value type");
    }
  }

  template <typename V> inline void operator()(const V& value) {
    if constexpr (std::is_same_v<V, int> || std::is_same_v<V, int32_t>) {
      writer.Int64(static_cast<int64_t>(value));
    } else if constexpr (std::is_same_v<V, unsigned int> || std::is_same_v<V, uint32_t>) {
      writer.Uint64(static_cast<uint64_t>(value));
    } else if constexpr (std::is_same_v<V, uint64_t>) {
      writer.Uint64(value);
    } else if constexpr (std::is_same_v<V, int64_t>) {
      writer.Int64(value);
    } else if constexpr (std::is_same_v<V, double> || std::is_same_v<V, float>) {
      writer.Double(value);
    } else if constexpr (std::is_same_v<V, bool>) {
      writer.Bool(value);
    } else if constexpr (std::is_same_v<V, std::nullptr_t>) {
      writer.Null();
    } else if constexpr (is_string_like_v<V>) {
      writer.String(value);
    } else {
      static_assert(always_false<V>, "Unsupported value type");
    }
  }
};

} // namespace rapidjson

#endif /* VALHALLA_BALDR_RAPIDJSON_UTILS_H_ */
