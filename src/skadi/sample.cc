#include "skadi/sample.h"

#include <cstddef>
#include <stdexcept>
#include <gdal.h>

namespace {

  struct driver_t {
    driver_t() {
      GDALAllRegister();
    }
    ~driver_t() {
      GDALDestroyDriverManager();
    }
  };

  //singleton so we can hide this from the interface
  void initialize() {
    static driver_t driver;
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

  template <class T>
  int32_t sample::get(const std::pair<T, T> coord) {

    //project the coordinates back to image space (pixels)

    //check if its in bounds and if not return no data value

    //round them

    //pull out quad of pixels
    //GDALRasterBandH band = GDALGetRasterBand(source.get(), 1);
    //double quad[4];
    //GDALRasterIO(band, GF_Read, column, row, 2, 2, quad, 2, 2, GDT_CFloat64, 0, 0) == CE_None

    //bilinear interpolation

    return 0;
  }

  //explicit instantiations for templated get
  template int32_t sample::get<double>(const std::pair<double, double>);
  template int32_t sample::get<float>(const std::pair<float, float>);

}
}
