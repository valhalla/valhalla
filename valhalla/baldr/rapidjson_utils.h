#ifndef VALHALLA_BALDR_RAPIDJSON_UTILS_H_
#define VALHALLA_BALDR_RAPIDJSON_UTILS_H_

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/pointer.h>
#include <type_traits>

namespace rapidjson{

template<typename T, typename P>
bool Is(const P* ptr){
  if (std::is_convertible<T, double>::value){
    return ptr->IsNumber();
  }else{
    return ptr->Is<T>();
  }
}

template<typename T>
inline std::string to_string(const T& document_or_value) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document_or_value.Accept(writer);
  return std::string(buffer.GetString(), buffer.GetSize());
}

}

namespace valhalla{

template<typename T, typename V>
inline boost::optional<T> GetOptionalFromRapidJson(V&& v, const char* source){
  auto* ptr= rapidjson::Pointer{source}.Get(std::forward<V>(v));
  if(! ptr || ! rapidjson::template Is<T>(ptr)) {
    return boost::nonde;
  }
  return ptr->template Get<T>();
}

}

#endif /* VALHALLA_BALDR_RAPIDJSON_UTILS_H_ */

