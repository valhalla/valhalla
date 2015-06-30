#include "skadi/sample.h"

#include <cstddef>
#include <stdexcept>
#include <gdal.h>

namespace {

  struct sample_driver_t{
    sample_driver_t() {
      GDALAllRegister();
    }
    ~sample_driver_t() {
      GDALDestroyDriverManager();
    }
  };

  //singleton so we can hide this from the interface
  void initialize() {
    static sample_driver_t driver;
  }

}

namespace valhalla {
namespace skadi {

  sample::sample(const std::string& data_source) {
    initialize();
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
