#ifndef VALHALLA_BALDR_RAPIDJSON_UTILS_H_
#define VALHALLA_BALDR_RAPIDJSON_UTILS_H_

#include <stdexcept>
#include <type_traits>

#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

//rapidjson loves to assert and crash programs, its more useful to throw and catch
#define RAPIDJSON_ASSERT(x) if (!(x)) throw std::logic_error(RAPIDJSON_STRINGIFY(x))

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/pointer.h>
#include <rapidjson/error/en.h>
#include <rapidjson/prettywriter.h>

namespace rapidjson{

template<typename T>
inline std::string to_string(const T& document_or_value) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document_or_value.Accept(writer);
  return std::string(buffer.GetString(), buffer.GetSize());
}

}

namespace valhalla{

//if you dont want an arithmetic type dont try any lexical casting
template<typename T, typename V>
inline typename std::enable_if<!std::is_arithmetic<T>::value, boost::optional<T> >::type GetOptionalFromRapidJson(V&& v, const char* source){
  //if we dont have this key bail
  auto* ptr = rapidjson::Pointer{source}.Get(std::forward<V>(v));
  if(!ptr)
    return boost::none;
  //if its the exact right type give it back
  if(ptr->template Is<T>())
    return ptr->template Get<T>();
  //give up
  return boost::none;
}

//if you do want an arithmetic type dont try lexical casting as a last resort
template<typename T, typename V>
inline typename std::enable_if<std::is_arithmetic<T>::value, boost::optional<T> >::type GetOptionalFromRapidJson(V&& v, const char* source){
  //if we dont have this key bail
  auto* ptr = rapidjson::Pointer{source}.Get(std::forward<V>(v));
  if(!ptr)
    return boost::none;
  //if its the exact right type give it back
  if(ptr->template Is<T>())
    return ptr->template Get<T>();
  //try to convert from a string
  if(ptr->IsString()) {
    try { return boost::lexical_cast<T>(ptr->template Get<std::string>()); }
    catch (...) { }
  }
  //numbers are strict in rapidjson but we don't want that strictness because it aborts the program (wtf?)
  if(ptr->IsBool())
    return static_cast<T>(ptr->GetBool());
  if(ptr->IsInt())
    return static_cast<T>(ptr->GetInt());
  if(ptr->IsUint())
    return static_cast<T>(ptr->GetUint());
  if(ptr->IsInt64())
    return static_cast<T>(ptr->GetInt64());
  if(ptr->IsUint64())
    return static_cast<T>(ptr->GetUint64());
  if(ptr->IsDouble())
    return static_cast<T>(ptr->GetDouble());
  //give up
  return boost::none;
}

template<typename T, typename V>
inline T GetFromRapidJson(V&& v, const char* source, const T& t){
  auto value = GetOptionalFromRapidJson<T>(v, source);
  if(value)
    return *value;
  return t;
}

}

#endif /* VALHALLA_BALDR_RAPIDJSON_UTILS_H_ */
