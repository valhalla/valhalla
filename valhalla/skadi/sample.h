#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <cstdint>
#include <memory>
#include <string>

namespace valhalla {
  namespace skadi {

    class sample{
     public:
      sample(const std::string& data_source);
      int32_t get(int32_t x, int32_t y/*, filter*/);
      //T<int32_t> get(const T& list*/, filter*/);

     protected:
      std::shared_ptr<void> source;
    };

  }
}


#endif //__VALHALLA_SAMPLE_H__
