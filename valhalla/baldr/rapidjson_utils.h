/*
 * rapidjson_utils.h
 *
 *  Created on: 3 Feb 2017
 *      Author: xiaolong
 */

#ifndef VALHALLA_BALDR_RAPIDJSON_UTILS_H_
#define VALHALLA_BALDR_RAPIDJSON_UTILS_H_

#include <rapidjson/document.h>
#include <rapidjson/pointer.h>

namespace valhalla{

template<typename T, typename V>
inline boost::optional<T> GetOptionalFromRapidJson(V&& v, const char* source){
  auto* ptr= rapidjson::Pointer{source}.Get(std::forward<V>(v));
  if(! ptr || ! ptr->template Is<T>()) {
    return {};
  }
  return ptr->template Get<T>();
}

}



#endif /* VALHALLA_BALDR_RAPIDJSON_UTILS_H_ */
