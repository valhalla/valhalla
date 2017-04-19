#ifndef __VALHALLA_SAMPLE_H__
#define __VALHALLA_SAMPLE_H__

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include <valhalla/midgard/sequence.h>

namespace valhalla {
  namespace skadi {

    class sample{
     public:
      //non-default-constructable and non-copyable
      sample() = delete;
      sample(sample&&) = default;
      sample& operator=(sample&&) = default;
      sample(const sample&) = delete;
      sample& operator=(const sample&) = delete;

      /**
       * Constructor
       * @param data_source  directory name of the datasource from which to sample
       */
      sample(const std::string& data_source);

      /**
       * Get a single sample from the datasource
       * @param coord  the single posting at which to sample the datasource
       */
      template <class coord_t>
      double get(const coord_t& coord) const;

      /**
       * Get multiple samples from the datasource
       * @param coords  the list of postings at which to sample the datasource
       */
      template <class coords_t>
      std::vector<double> get_all(const coords_t& coords) const;

      /**
       * @return the no data value for this data source
       */
      double get_no_data_value() const;

     protected:

      std::vector<midgard::mem_map<int16_t> > cache;
    };

  }
}


#endif //__VALHALLA_SAMPLE_H__
