#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <memory>
#include <string>
#include <utility>

namespace valhalla {
  namespace skadi {

    class sample{
     public:
      sample(const std::string& data_source);
      template <class T>
      double get(const std::pair<T, T>& coord);
      //T<double> get(const T& list);

     protected:
      std::shared_ptr<void> source;
      void* band;
      double no_data_value;
      double inverse_transform[6];
    };

  }
}


#endif //__VALHALLA_SAMPLE_H__
