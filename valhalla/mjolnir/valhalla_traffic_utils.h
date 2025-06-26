#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <vector>
#include <string>
#include <cstdint>  // for uint64_t
#include <iostream>

void update_traffic_tile(uint64_t tile_offset, const std::vector<uint64_t>& traffic_params, uint64_t last_updated, std::string traffic_path);
int handle_ways_to_edges(std::string config_file_path);
int handle_tile_offset_index(std::string config_file_path);


#endif // TRAFFIC_H
