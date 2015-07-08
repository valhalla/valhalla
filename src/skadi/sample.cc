#include "skadi/sample.h"

#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <limits>

#include <gdal.h>

#include <valhalla/midgard/logging.h>

namespace {

  //this is a once per process thing
  struct driver_t {
    driver_t() {
      GDALAllRegister();
    }
    ~driver_t() {
      GDALDestroyDriverManager();
    }
  };

  //so we put it in a hidden singleton
  void initialize() {
    static driver_t driver;
  }

}

namespace valhalla {
namespace skadi {

  sample::sample(const std::string& data_source)
    :no_data_value(std::numeric_limits<double>::min()) {

    //init the format drivers
    initialize();

    //open the dataset
    GDALDatasetH src_ptr = GDALOpenEx(data_source.c_str(), GDAL_OF_RASTER, nullptr, nullptr, nullptr);
    if(src_ptr == nullptr)
      throw std::runtime_error("Couldn't open " + data_source);
    source.reset(src_ptr, GDALClose);

    //get the first band of data
    band = GDALGetRasterBand(source.get(), 1);
    if(band == nullptr)
      throw std::runtime_error("Couldn't get raster band 1");

    //what's the value when there is no value
    int success;
    no_data_value = GDALGetRasterNoDataValue(band, &success);
    if(!success)
      LOG_WARN("No data value is not set for data source, using double::min");
    LOG_INFO("No data value is " + std::to_string(no_data_value));

    //get the inverse transform to go from lat,lon to pixels
    double transform[6];
    if(GDALGetGeoTransform(source.get(), transform) != CE_None)
      throw std::runtime_error("Could not get coordinate transform for data source");
    if(!GDALInvGeoTransform(transform, inverse_transform))
      throw std::runtime_error("Could not invert the coordinate transform");
  }

  template <class T>
  double sample::get(const std::pair<T, T> coord) {
    //project the coordinates into to image space
    double x = inverse_transform[0] +
               inverse_transform[1] * coord.first +
               inverse_transform[2] * coord.second;
    double y = inverse_transform[3] +
               inverse_transform[4] * coord.first +
               inverse_transform[5] * coord.second;

    //round them
    int mid_x = static_cast<int>(x + .5);
    int mid_y = static_cast<int>(y + .5);

    //pull out quad of pixels
    double quad[2];
    if(GDALRasterIO(band, GF_Read, mid_x, mid_y, 1, 1, quad,
                    1, 1, GDT_CFloat64, 0, 0) == CE_None)
      return quad[0];
    else
      return no_data_value;

    //bilinear interpolation
  }

  //explicit instantiations for templated get
  template double sample::get<double>(const std::pair<double, double>);
  template double sample::get<float>(const std::pair<float, float>);

}
}
