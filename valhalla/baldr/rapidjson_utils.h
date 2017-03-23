#ifndef VALHALLA_BALDR_RAPIDJSON_UTILS_H_
#define VALHALLA_BALDR_RAPIDJSON_UTILS_H_

#include <boost/lexical_cast.hpp>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/pointer.h>
#include <type_traits>

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
  //or you want a number but is not quite the same type then convert
  if(ptr->template Is<T>() || (std::is_arithmetic<T>::value && ptr->IsNumber()))
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
  //or you want a number but is not quite the same type then convert
  if(ptr->template Is<T>() || (std::is_arithmetic<T>::value && ptr->IsNumber()))
    return ptr->template Get<T>();
  //finally, try to convert from a string
  if(ptr->IsString()) {
    try { return boost::lexical_cast<T>(ptr->template Get<std::string>()); }
    catch (...) { }
  }
  //give up
  return boost::none;
}

}

#endif /* VALHALLA_BALDR_RAPIDJSON_UTILS_H_ */

