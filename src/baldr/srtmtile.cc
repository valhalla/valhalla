#include "baldr/srtmtile.h"

#include <iostream>
#include <fstream>

#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;

namespace valhalla {
namespace baldr {

namespace {
  // SRTM is 30m postings.
  constexpr uint32_t kRowColCount = 3601;
  constexpr uint32_t kSRTMPosts = 3601 * 3601;

  // Return an invalid height if no SRTM tile is found.
  // THis is void filled SRTM so perhaps it won't exist?
  // http://dds.cr.usgs.gov/srtm/version2_1/Documentation/Quickstart.pdf
  constexpr float kEmptyHeight = -32768.0f;
}

// Default constructor
SRTMTile::SRTMTile(const std::string& dir, const int32_t baselat,
                   const int32_t baselng)
    : loaded_(false),
      baselat_(baselat),
      baselng_(baselng) {
  // SRTM is only valid between -60 and 60 latitude
  int32_t l = std::abs(baselat);
  if (baselat <= -60 || baselat >= 60) {
    LOG_WARN("No SRTM at this latitude: " + std::to_string(baselat));
    return;
  }

  // Allocate height array
  heights_ = new int16_t[kSRTMPosts];

  // Load the SRTM data from file
  std::string fname = filename(dir, baselat_, baselng_);
  std::ifstream file(fname, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    LOG_WARN("No SRTM file: " + fname);
    return;
  }

  // Read the SRTM height data
  file.read(reinterpret_cast<char*>(heights_), kSRTMPosts * sizeof(int16_t));
  if (file.bad()) {
    LOG_ERROR("Error reading SRTM from " + fname);
    return;
  }

  // Close file and set loaded = true to mark success
  file.close();
  loaded_ = true;
}

// Destructor.
SRTMTile::~SRTMTile() {
  delete [] heights_;
}

// Check the status - is the SRTM data loaded for this tile.
bool SRTMTile::loaded() const {
  return loaded_;
}

// Get the base latitude of the tile
float SRTMTile::baselat() const {
  return baselat_;
}

/// Get the base longitude of the tile
float SRTMTile::baselng() const {
  return baselng_;
}

// Gets the height at the specified lat,lng. If optional filtering is
// specified, a box filter (weighted) using the 4 nearest height postings
// is used.
float SRTMTile::height(const PointLL& ll, const bool filter) const {
  // Get the base row and column for the lat,lng
  float row = ((ll.lat() - baselat_) * static_cast<float>(kRowColCount - 1));
  float col = ((ll.lng() - baselng_) * static_cast<float>(kRowColCount - 1));
  int32_t r0 = static_cast<int32_t>(row);
  int32_t c0 = static_cast<int32_t>(col);

  // Use a bilinear (box) filter
  if (filter) {
    // Get the integer row and column positions (cells are padded +1)
    int32_t r1 = r0 + 1;
    int32_t c1 = c0 + 1;

    // Check each height and get its weight (box filter), track number of
    // posts with no data. Hopefully none since this should be void
    // filled data!
    float dr = row - r0;
    float dc = col - c0;
    int32_t n_invalid = 0;

    float w00;
    float h00 = static_cast<float>(heights_[(r0 * kRowColCount) + c0]);
    if (h00 == kEmptyHeight) {
      w00 = 0.0f;
      n_invalid++;
    } else {
      w00 = (1.0f - dr) * (1.0f - dc);
    }

    float w10;
    float h10 = heights_[(r1 * kRowColCount) + c0];
    if (h10 == kEmptyHeight) {
      w10 = 0.0f;
      n_invalid++;
    } else {
      w10 = dr * (1.0f - dc);
    }

    float w01;
    float h01 = heights_[(r0 * kRowColCount) + c1];
    if (h01 == kEmptyHeight) {
      w01 = 0.0f;
      n_invalid++;
    } else {
      w01 = (1.0f - dr) * dc;
    }

    float w11;
    float h11 = heights_[(r1 * kRowColCount) + c1];
    if (h11 == kEmptyHeight) {
      w11 = 0.0f;
      n_invalid++;
    } else {
      w11 = dc * dr;
    }

    // No data on any of the 4 cells
    if (n_invalid == 4)
      return kEmptyHeight;

    // Redistribute the weights so they sum to 1 (in case any of them
    // had no data)
    float scale = 1.0f / (w00 + w10 + w01 + w11);

    // Return the weighted average of the 4 postings
    return scale * w00 * h00 + scale * w10 * h10 +
           scale * w01 * h01 + scale * w11 * h11;
  }
  else {
    // Return height of a single post
    return static_cast<float>(heights_[(r0 * kRowColCount) + c0]);
  }
}

// Constructs the SRTM file name given the latitude, longitude of the
// south west corner.
std::string SRTMTile::filename(const std::string& basedir, const int32_t lat,
                               const int32_t lng) const {
  std::string subdir;
  char latstr[8];
  if (lat < 0) {
    sprintf(latstr, "%d", -lat);
    subdir = "S" + std::to_string((-lat / 10));
  } else {
    sprintf(latstr, "%d", lat);
    subdir = "N" + std::to_string((lat / 10));
  }

  std::string fname;
  if (lat <= -10)
    fname = "S" + std::string(latstr);
  else if (lat < 0)
    fname = "S0" + std::string(latstr);
  else if (lat >= 10)
    fname = "N" + std::string(latstr);
  else
    fname = "N0" + std::string(latstr);

  char lngstr[8];
  if (lng < 0) {
    sprintf(lngstr, "%d", -lng);
  } else {
    sprintf(lngstr, "%d", lng);
  }
  if (lng <= -100)
    fname += "W" + std::string(lngstr);
  else if (lng <= -10)
    fname += "W0" + std::string(lngstr);
  else if (lng < 0)
    fname += "W00" + std::string(lngstr);
  else if (lng >= 100)
    fname += "E" + std::string(lngstr);
  else if (lng >= 10)
    fname += "E0" + std::string(lngstr);
  else
    fname += "E00" + std::string(lngstr);

  fname += ".hgt";
  return basedir + "/" + subdir + "/" + fname;
}

}
}
