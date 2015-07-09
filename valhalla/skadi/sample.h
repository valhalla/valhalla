#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace valhalla {
  namespace skadi {

    class sample{
     public:
      /**
       * Constructor
       * @param data_source  file name of the datasource from which to sample
       */
      sample(const std::string& data_source);


      /**
       * Get a single sample from the datasource
       * @param coord  the single posting at which to sample the datasource
       */
      template <class coord_t>
      double get(const coord_t& coord);


      /**
       * Get multiple samples from the datasource
       * @param coords  the list of postings at which to sample the datasource
       */
      template <class coords_t>
      std::vector<double> get_all(const coords_t& coords);

     protected:
      std::shared_ptr<void> source;
      void* band;
      double no_data_value;
      double inverse_transform[6];
    };

  }
}


#endif //__VALHALLA_SAMPLE_H__
