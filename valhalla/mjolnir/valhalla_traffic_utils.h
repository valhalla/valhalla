#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <cstdint> // for uint64_t
#include <iostream>
#include <string>
#include <vector>

void update_traffic_tile(uint64_t tile_offset,
                         const std::vector<uint64_t>& traffic_params,
                         uint64_t last_updated,
                         std::string traffic_path);
int handle_tile_offset_index(std::string config_file_path);

int handle_build_verification(std::string config_file_path);
int handle_verify(std::string traffic_file_path, std::string verify_path);
int handle_copy_traffic(std::string traffic_src_path, std::string traffic_dest_path);

#endif // TRAFFIC_H
