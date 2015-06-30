#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace valhalla {
  namespace skadi {

    class sample{
     public:
      sample(const std::string& data_source);
      template <class T>
      int32_t get(const std::pair<T, T> coord/*, filter*/);
      //T<int32_t> get(const T& list*/, filter*/);

     protected:
      std::shared_ptr<void> source;
    };

  }
}


#endif //__VALHALLA_SAMPLE_H__
