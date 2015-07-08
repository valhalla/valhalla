#include "skadi/sample.h"

#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <limits>

//TODO: switch to using gdal_priv.h, which looks like proper c++ bindings
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

    //get the inverse transform to go from lat,lon to pixels
    double transform[6];
    if(GDALGetGeoTransform(source.get(), transform) != CE_None)
      throw std::runtime_error("Could not get coordinate transform for data source");
    if(!GDALInvGeoTransform(transform, inverse_transform))
      throw std::runtime_error("Could not invert the coordinate transform");
  }

  template <class T>
  double sample::get(const std::pair<T, T>& coord) const {
    //project the coordinates into to image space
    double x = inverse_transform[0] +
               inverse_transform[1] * coord.first +
               inverse_transform[2] * coord.second;
    double y = inverse_transform[3] +
               inverse_transform[4] * coord.first +
               inverse_transform[5] * coord.second;
    int fx = floor(x);
    int fy = floor(y);

    //pull out quad of pixels, image origin is top left
    //quad is laid out exactly the same
    double quad[2];
    GDALRasterIOExtraArg args{RASTERIO_EXTRA_ARG_CURRENT_VERSION, GRIORA_Bilinear,
                              nullptr, nullptr, true, x - fx, y - fy, 1, 1};
    auto failure = GDALRasterIOEx(band, GF_Read, fx, fy, 2, 2, quad,
        1, 1, GDT_CFloat64, 0, 0, &args);

    //got back some data
    if(failure == CE_None)
      return quad[0];
    //need to check corner cases (quad partially outside of image)
    else
      return no_data_value;
  }

  //explicit instantiations for templated get
  template double sample::get<double>(const std::pair<double, double>&) const;
  template double sample::get<float>(const std::pair<float, float>&) const;

}
}
