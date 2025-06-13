#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <vector>
#include <string>
#include <cstdint>  // for uint64_t
#include <iostream>

void update_traffic_tile(uint64_t tile_offset, const std::vector<uint64_t>& traffic_params, uint64_t last_updated);

#endif // TRAFFIC_H
