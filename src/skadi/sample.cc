#include "skadi/sample.h"

#include <cstddef>
#include <stdexcept>

namespace valhalla {
namespace skadi {

  sample::sample(const char* data_source) {
    GDALDatasetH src_ptr = GDALOpenEx(data_source, GDAL_OF_RASTER, NULL, (const char* const* )NULL, NULL);
    if(src_ptr == nullptr)
      throw std::runtime_error("Couldn't open " + data_source);
    source = std::unique_ptr(src_ptr, GDALClose);
  }

  int32_t sample::sample(int32_t x, int32_t y) {

    return 0;
  }

}
}
