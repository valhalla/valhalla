#include "skadi/sample.h"

#include <cstddef>
#include <stdexcept>
#include <gdal.h>

namespace valhalla {
namespace skadi {

  sample::sample(const std::string& data_source) {
    GDALDatasetH src_ptr = GDALOpenEx(data_source.c_str(), GDAL_OF_RASTER, NULL, (const char* const* )NULL, NULL);
    if(src_ptr == nullptr)
      throw std::runtime_error("Couldn't open " + data_source);
    source.reset(src_ptr, GDALClose);
  }

  int32_t sample::get(int32_t x, int32_t y) {

    return 0;
  }

}
}
