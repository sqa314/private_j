#pragma once

#include "models.h"

std::vector<point_t> find_path(const grid_t& grid, const point_t& start, const point_t& goal);
void print_path(const grid_t& grid, const std::vector<point_t>& path);