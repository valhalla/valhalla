#include <boost/format.hpp>
#include <fstream>
#include <iostream>
#include <sqlite3.h>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "baldr/json.h"

using dataPair =
    std::pair<std::vector<std::string>*, std::unordered_map<std::string, std::vector<float>>*>;

/*
 * Handle errors returned from the db
 * Params
 *  rc - response code from previous sqlite3_exec call
 *  errMsg - char* passed into the previous sqlite3_exec call
 * Post
 *  returns normally if there is not error
 *  prints the db error and exits if an error occurred
 */
void checkDBResponse(int rc, char* errMsg) {
  if (rc != SQLITE_OK) {
    std::cout << "SQL error: " << errMsg << std::endl;
    sqlite3_free(errMsg);
    exit(0);
  }
}

/*
 * Callback function that gets the column names from the database
 */
static int classCallback(void* data, int argc, char** argv, char** colName) {
  std::vector<std::string>* cat = (std::vector<std::string>*)data;
  for (int i = 1; i < argc; ++i) {
    cat->push_back(colName[i]);
  }
  cat->push_back("total");
  return 0;
}

/*
 * Queries the database to get road class types
 * Params
 *  *db - sqlite3 database connection object
 *  classes - a vector of strings representing the road types
 *
 * Post
 *  The array containing the road classes is filled
 */
void fillClasses(sqlite3* db, std::vector<std::string>& classes) {

  std::string sql = "SELECT * FROM countrydata LIMIT 1";
  char* errMsg = 0;
  int rc = sqlite3_exec(db, sql.c_str(), classCallback, (void*)&classes, &errMsg);
  checkDBResponse(rc, errMsg);
}

/*
 * Callback function to reveive query results from sqlite3_exec
 * Params
 *  *dataPair - the pointer used to pass data structures into this function
 *  argc - # arguments
 *  **argv - char array of arguments
 *  **colName - char array returned column names from the database
 */
static int countryDataCallback(void* data_pair, int argc, char** argv, char** colName) {
  dataPair* p = (dataPair*)data_pair;
  auto* countries = std::get<0>(*p);
  auto* data = std::get<1>(*p);

  countries->push_back(argv[0]);
  std::string line = "";
  for (int i = 1; i < argc; ++i) {
    line += argv[i];
    line += " ";
  }

  std::istringstream iss(line);
  float tok = 0;
  float sum = 0;
  std::vector<float>* classData = new std::vector<float>();

  while (iss >> tok) {
    classData->push_back(tok);
    sum += tok;
  }
  classData->push_back(sum);
  data->emplace(argv[0], *classData);

  return 0;
}

/*
 * Queries the database to get length of roads in each class per country
 * Params
 *  *db - sqlite3 database connection object
 *  countries - vector of country iso codes
 *  data - 2D vector of road lengths
 *    each column belongs to a certain country
 *    each row is a type of road
 * Post
 *  countries contains the iso codes of all countries in the database
 *  data contains the lengths of each type of road for each country
 */
void fillCountryData(sqlite3* db,
                     std::vector<std::string>& countries,
                     std::unordered_map<std::string, std::vector<float>>& data) {

  std::string sql = "SELECT * FROM countrydata WHERE isocode IS NOT \"\"";

  char* errMsg = 0;
  dataPair data_pair = {&countries, &data};
  int rc = sqlite3_exec(db, sql.c_str(), countryDataCallback, (void*)&data_pair, &errMsg);
  checkDBResponse(rc, errMsg);
}

/*
 * Callback function to receive query results from sqlite3_exec
 * Params
 *  *dataPair - the pointer used to pass data structures into this function
 *  argc - # arguments
 *  **argv - char array of arguments
 *  **colName - char array returned column names from the database
 */
static int maxSpeedCallback(void* data, int argc, char** argv, char** colName) {
  auto* maxSpeedInfo = (std::unordered_map<std::string, std::vector<float>>*)data;

  std::istringstream ss(argv[2]);
  float val;
  ss >> val;

  maxSpeedInfo->at(argv[0]).push_back(val);

  return 0;
}

/*
 * Retrieves the maxspeed data from the database and fills the data structure
 */
void fillMaxSpeedData(sqlite3* db,
                      std::unordered_map<std::string, std::vector<float>>& maxSpeedInfo) {

  std::string sql = "SELECT isocode,type,maxspeed";
  sql += " FROM rclassctrydata";
  sql += " WHERE (type='Motorway' OR type='Primary' OR type='Secondary' OR type='Trunk')";
  sql += " AND isocode IS NOT \"\"";

  char* errMsg = 0;
  int rc = sqlite3_exec(db, sql.c_str(), maxSpeedCallback, (void*)&maxSpeedInfo, &errMsg);
  checkDBResponse(rc, errMsg);
}

/*
 * Callback function to receive query results from sqlite3_exec
 * Params
 *  *data - the pointer used to pass data structures into this function
 *  argc - # arguments
 *  **argv - char array of arguments
 *  **colName - char array returned column names from the database
 */
static int namedCallback(void* data, int argc, char** argv, char** colName) {
  auto* namedInfo = (std::unordered_map<std::string, std::vector<float>>*)data;

  std::istringstream ss(argv[2]);
  float val;
  ss >> val;

  namedInfo->at(argv[0]).push_back(val);

  return 0;
}

/*
 * Retrieves the named road data from the database and fills the data structure
 */
void fillNamedData(sqlite3* db, std::unordered_map<std::string, std::vector<float>>& namedInfo) {

  std::string sql = "SELECT isocode,type,named";
  sql += " FROM rclassctrydata";
  sql += " WHERE (type='Residential' OR type='Unclassified')";
  sql += " AND isocode IS NOT \"\"";

  char* errMsg = 0;
  int rc = sqlite3_exec(db, sql.c_str(), namedCallback, (void*)&namedInfo, &errMsg);
  checkDBResponse(rc, errMsg);
}
/*
 * Generates a javascript file that has a function to return
 *  all the data queried from the database
 * Params
 *  countries - names of all the countries from the database
 *  data - float values for all the lengths of road per country
 *  classes - the types of road classes
 *  maxClasses - types of road data which have corresponding maxspeed data
 *  maxSpeedInfo - float values for each maxClass for each country
 */
void generateJson(std::vector<std::string>& countries,
                  std::unordered_map<std::string, std::vector<float>>& data,
                  std::vector<std::string>& classes,
                  std::unordered_map<std::string, std::vector<float>>& maxSpeedInfo,
                  std::vector<std::string>& maxClasses,
                  std::unordered_map<std::string, std::vector<float>>& namedInfo,
                  std::vector<std::string>& namedClasses) {

  using namespace valhalla::baldr;

  std::ofstream out("road_data.json");
  json::MapPtr map = json::map({});
  for (size_t i = 0; i < countries.size(); ++i) {
    map->emplace(countries[i],
                 json::map(
                     {{"iso2", countries[i]},
                      {"classinfo", json::map({{classes[0], json::fp_t{data[countries[i]][0]}},
                                               {classes[1], json::fp_t{data[countries[i]][1]}},
                                               {classes[2], json::fp_t{data[countries[i]][2]}},
                                               {classes[3], json::fp_t{data[countries[i]][3]}},
                                               {classes[4], json::fp_t{data[countries[i]][4]}},
                                               {classes[5], json::fp_t{data[countries[i]][5]}},
                                               {classes[6], json::fp_t{data[countries[i]][6]}},
                                               {classes[7], json::fp_t{data[countries[i]][7]}},
                                               {classes[8], json::fp_t{data[countries[i]][8]}}})},
                      {"maxspeed",
                       json::map({{maxClasses[0], json::fp_t{maxSpeedInfo[countries[i]][0]}},
                                  {maxClasses[1], json::fp_t{maxSpeedInfo[countries[i]][1]}},
                                  {maxClasses[2], json::fp_t{maxSpeedInfo[countries[i]][2]}},
                                  {maxClasses[3], json::fp_t{maxSpeedInfo[countries[i]][3]}}})},
                      {"named", json::map({
                                    {namedClasses[0], json::fp_t{namedInfo[countries[i]][0]}},
                                    {namedClasses[1], json::fp_t{namedInfo[countries[i]][1]}},
                                })}}));
  }
  out << *map << std::endl;

  out.close();
}

int main(int argc, char** argv) {
  // If there is no input file print and error and exit
  if (argc < 2) {
    std::cout << "ERROR: No input file specified." << std::endl;
    std::cout << "Usage: " << argv[0] << " statistics.sqlite" << std::endl;
    exit(0);
  }

  // data structures
  std::vector<std::string> roadClasses;
  std::vector<std::string> countries;
  std::unordered_map<std::string, std::vector<float>> roadClassInfo;
  // speed info
  std::vector<std::string> maxClasses = {"Motorway", "Primary", "Secondary", "Trunk"};
  std::unordered_map<std::string, std::vector<float>> maxSpeedInfo;
  // name info
  std::vector<std::string> namedClasses = {"Residential", "Unclassified"};
  std::unordered_map<std::string, std::vector<float>> namedInfo;

  // open DB file
  sqlite3* db;
  int rc = sqlite3_open(argv[1], &db);
  if (rc) {
    std::cout << "Opening DB failed: " << sqlite3_errmsg(db);
    exit(0);
  }

  // fill data structures
  fillClasses(db, roadClasses);
  fillCountryData(db, countries, roadClassInfo);
  // create the entries in the map
  for (auto& s : countries) {
    maxSpeedInfo[s];
    namedInfo[s];
  }
  fillMaxSpeedData(db, maxSpeedInfo);
  fillNamedData(db, namedInfo);

  generateJson(countries, roadClassInfo, roadClasses, maxSpeedInfo, maxClasses, namedInfo,
               namedClasses);

  sqlite3_close(db);
  return 0;
}
