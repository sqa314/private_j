#include "jps_plus.h"

#include <iostream>

namespace
{
const std::vector<point_t> directions = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

const std::map<point_t, std::vector<std::pair<int, int>>> target_validators = {{directions[0], {{4, 2}, {6, 5}}},
                                                                               {directions[1], {{3, 0}, {4, 2}}},
                                                                               {directions[2], {{3, 0}, {6, 7}}},
                                                                               {directions[3], {{1, 0}, {6, 5}}},
                                                                               {directions[4], {{1, 2}, {6, 7}}},
                                                                               {directions[5], {{1, 0}, {4, 7}}},
                                                                               {directions[6], {{3, 5}, {4, 7}}},
                                                                               {directions[7], {{3, 5}, {1, 2}}}};

bool is_valid_point(const grid_t& grid, const point_t point)
{
    const auto [row, column] = point;

    return row >= 0 && row < grid.size() && column >= 0 && column < grid[row].size();
}

bool is_force_neighbor(const grid_t& grid, const point_t point, const point_t direction)
{
    if (!is_valid_point(grid, point) || !grid[point.row][point.column])
    {
        return false;
    }

    for (const auto [close, open] : target_validators.at(direction))
    {
        const auto corner = point + directions[close];
        const auto path = point + directions[open];

        if (is_valid_point(grid, corner) && is_valid_point(grid, path) &&
            grid[corner.row][corner.column] < grid[point.row][point.column] && grid[path.row][path.column])
        {
            return true;
        }
    }

    return false;
}

node_t jump(const grid_t& grid, const node_t current, const point_t direction, const point_t goal)
{
    const auto next_point = current.point + direction;

    if (!is_valid_point(grid, next_point) || !grid[next_point.row][next_point.column])
    {
        return current;
    }

    const auto next = node_t(next_point, current.duration + grid[next_point.row][next_point.column]);

    if (next_point == goal)
    {
        return next;
    }

    if (direction.row != 0 && direction.column != 0)
    {
        if (is_force_neighbor(grid, next_point, direction))
        {
            return next;
        }

        const auto& h_jump = jump(grid, next, {0, direction.column}, goal);
        const auto& v_jump = jump(grid, next, {direction.row, 0}, goal);

        if (h_jump.point == next_point || v_jump.point == next_point)
        {
            return next;
        }

        return jump(grid, next, direction, goal);
    }
    else
    {
        if (is_force_neighbor(grid, next_point, direction))
        {
            return next;
        }

        return jump(grid, next, direction, goal);
    }
}

float get_heuristic(const point_t current, const point_t goal)
{
    const auto h = std::abs(current.column - goal.column);
    const auto v = std::abs(current.row - goal.row);
 
    return std::pow(std::pow(h, 2) + std::pow(v, 2), 0.5);
}

void grid_scan(jps_t& jps, const grid_t grid, const node_t& current, const point_t& goal)
{
    for (const auto& dir : directions)
    {
        const auto& [point, duration, _] = jump(grid, current, dir, goal);

        if (point == current.point)
        {
            continue;
        }

        if (jps.optimal_duration.count(point) == 0 || duration < jps.optimal_duration[point])
        {
            jps.came_from[point] = current.point;
            jps.optimal_duration[point] = duration;
            jps.open_set.emplace(point, duration, duration + get_heuristic(point, goal));
        }
    }
}
} // namespace

std::vector<point_t> find_path(const grid_t& grid, const point_t& start, const point_t& goal)
{
    auto jps = jps_t();
    auto& [_, open_set, optimal_duration, came_from] = jps;

    open_set.emplace(start, 0, get_heuristic(start, goal));
    optimal_duration[start] = 0;

    while (!open_set.empty())
    {
        auto current = open_set.top();
        open_set.pop();

        if (current.point == goal)
        {
            std::vector<point_t> path;

            for (auto node = goal; node != start; node = came_from[node])
            {
                path.push_back(node);
            }

            path.push_back(start);
            std::reverse(path.begin(), path.end());

            return path;
        }

        grid_scan(jps, grid, current, goal);
    }

    return {}; // No path found
}

void print_path(const grid_t& grid, const std::vector<point_t>& path)
{
    const auto row_count = grid.size();
    const auto column_count = grid[0].size();

    std::vector<std::vector<char>> display(row_count, std::vector<char>(column_count));

    // Mark obstacles
    for (int i = 0; i < row_count; ++i)
    {
        for (int j = 0; j < column_count; ++j)
        {
            if (!grid[i][j])
            {
                display[i][j] = '0';
            }
            else
            {
                display[i][j] = ' ';
            }
        }
    }

    // Mark path
    for (const auto& p : path)
    {
        display[p.row][p.column] = '*';
    }

    // Print grid
    for (const auto& row : display)
    {
        for (char cell : row)
        {
            std::cout << cell;
        }

        std::cout << '\n';
    }
}