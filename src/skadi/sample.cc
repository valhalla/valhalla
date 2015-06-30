#include "skadi/sample.h"

#include <cstddef>
#include <stdexcept>
#include <gdal.h>

namespace valhalla {
namespace skadi {

  sample_driver_t::sample_driver_t() {
    GDALAllRegister();
  }

  sample_driver_t::~sample_driver_t() {
    GDALDestroyDriverManager();
  }

  sample::sample(const sample_driver_t& driver, const std::string& data_source) {
    GDALDatasetH src_ptr = GDALOpenEx(data_source.c_str(), GDAL_OF_RASTER, nullptr, nullptr, nullptr);
    if(src_ptr == nullptr)
      throw std::runtime_error("Couldn't open " + data_source);
    source.reset(src_ptr, GDALClose);
  }

  int32_t sample::get(int32_t x, int32_t y) {

    return 0;
  }

}
}
