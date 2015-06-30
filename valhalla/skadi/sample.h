#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <cstdint>
#include <memory>
#include <string>

namespace valhalla {
  namespace skadi {

    //NOT THREADSAFE, ONLY ONE PER PROCESS
    struct sample_driver_t{
      sample_driver_t();
      ~sample_driver_t();
    };

    //TODO: one of two ways, need the class around to carry some state or memory of the data we are accessing?
    //or is it good enough to just make namespace functions to access this stuff on the fly. need to get to
    //testing this stuff

    class sample{
     public:
      sample(const sample_driver_t& driver, const std::string& data_source);
      int32_t get(int32_t x, int32_t y/*, filter*/);
      //T<int32_t> get(const T& list*/, filter*/);

     protected:
      std::shared_ptr<void> source;
    };

  }
}


#endif //__VALHALLA_SAMPLE_H__
