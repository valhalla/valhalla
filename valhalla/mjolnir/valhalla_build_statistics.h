#include <string>
#include <vector>
#include <boost/format.hpp>
#include <sqlite3.h>

using dataPair = std::pair<std::vector<std::string>*, std::vector<std::vector<float>>*>;

void fillData (std::ifstream& in, std::vector<std::vector<float>>& data, std::vector<std::string>& countries);

void generateJson (std::vector<std::string>& countries, std::vector<std::vector<float>>& data, std::vector<std::string>& classes);

void checkDBResponse(int rc, char* errMsg);

void fillClasses(sqlite3 *db, std::vector<std::string>& classes);

void fillMaxSpeedData(sqlite3 *db);

void fillCountryData(sqlite3 *db, std::vector<std::string>& countries, std::vector<std::vector<float>>& data);

static int classCallback (void *data, int argc, char **argv, char **colName);

static int countryDataCallback (void *data, int argc, char **argv, char **colName);

static int maxSpeedCallback (void *data, int argc, char **argv, char **colName);
