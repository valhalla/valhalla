#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <cstdint>
#include <memory>
#include <gdal.h>

namespace valhalla {
  namespace skadi {

    //TODO: one of two ways, need the class around to carry some state or memory of the data we are accessing?
    //or is it good enough to just make namespace functions to access this stuff on the fly. need to get to
    //testing this stuff

    class sample{
     public:
      sample(const char* data_source);
      int32_t get(int32_t x, int32_t y/*, filter*/);
      //T<int32_t> get(const T& list*/, filter*/);

     protected:
      std::unique_ptr<GDALDatasetH, decltype(GDALClose)> source;
    };

  }
}


#endif //__VALHALLA_SAMPLE_H__
