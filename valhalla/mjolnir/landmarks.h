#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include <valhalla/baldr/landmark.h>

namespace valhalla {
namespace mjolnir {

struct LandmarkDatabase {
public:
  /**
   * LandmarkDatabase constructor.
   * Create a new LandmarkDatabase object that connects to the SQLite landmark database
   * with the given `db_name`. The `read_only` parameter determines whether the
   * database connection is read-only or read-write.
   *
   * @param db_name The file path of the SQLite database to connect to.
   * @param read_only Set to true to open the database in read-only mode, false for read-write.
   */
  LandmarkDatabase(const std::string& db_name, bool read_only);

  /**
   * Insert a new landmark into the database.
   * This function inserts a new landmark into the database with the given `name`, `type`,
   * `lng`, and `lat`. The landmark's primary key will be automatically assigned by the database.
   *
   * @param name The name of the landmark.
   * @param type The type of the landmark (LandmarkType enum).
   * @param lng The longitude of the landmark.
   * @param lat The latitude of the landmark.
   */
  void insert_landmark(const std::string& name,
                       const baldr::LandmarkType& type,
                       const double lng,
                       const double lat);

  /**
   * Retrieve a vector of landmarks from the database within the specified bounding box.
   * Bounding box is a rectangle area defined by min latitude, max latitude, min longitude, and max
   * longitude.
   *
   * @param minlng The minimum longitude of the bounding box (min x).
   * @param minlat The minimum latitude of the bounding box (min y).
   * @param maxlng The maximum longitude of the bounding box (max x).
   * @param maxlat The maximum latitude of the bounding box (max y).
   * @return A vector of Landmark objects within the specified bounding box.
   */
  std::vector<baldr::Landmark> get_landmarks_by_bbox(const double minlng,
                                                     const double minlat,
                                                     const double maxlng,
                                                     const double maxlat);

  /**
   * Retrieve a vector of landmarks that match the provided primary keys.
   * NOTE: The return size may be less than the size of the input if some of the provided
   * primary keys do not exist in the database. In such cases, the function will skip the missing IDs
   * without throwing an exception.
   *
   * @param pkeys A vector of int64_t representing the primary keys of the landmarks to retrieve.
   * @return A vector of Landmark objects matching the given primary keys.
   */
  std::vector<baldr::Landmark> get_landmarks_by_ids(const std::vector<int64_t>& pkeys);

protected:
  struct db_pimpl;
  std::shared_ptr<db_pimpl> pimpl;
};

/**
 * Build a SQLite landmark database and insert landmarks data from the PBF files to the database.
 * This function reads landmark data from PBF files specified in the `input_files` vector.
 * It then parses the data to extract landmark nodes and stores the landmarks in the database.
 * The database is automatically built based on the configuration settings provided in `pt`.
 *
 * @param pt The configuration settings for the landmark building process.
 * @param input_files A vector of file paths to the PBF input files.
 * @return True if landmarks were successfully built and inserted into the database, false otherwise.
 */
bool BuildLandmarkFromPBF(const boost::property_tree::ptree& pt,
                          const std::vector<std::string>& input_files);

/**
 * Add landmarks from the landmark database to graph tiles.
 * This function finds landmarks in each graph tile, retrieves them from the landmark database,
 * associate them with nearby edges, and update the graph tiles with these associations of landmark -
 * edge pairs.
 *
 * @param pt The configuration settings
 * @return
 */
bool AddLandmarks(const boost::property_tree::ptree& pt);

} // namespace mjolnir

} // namespace valhalla
