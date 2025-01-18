#include "jps_plus.h"

#include <iostream>
#include <fstream>

namespace
{
    grid_t grid;
    jps_t jps;
    const std::vector<std::vector<int>> scan_candidates = { {1,3,0}, {1,0,2}, {1,5,2}, {3,0,6 },{1,3,5,7,0,2,6,8 }, {5,2,8}, {3,7,6}, {7,6,8}, {5,7,8} };
    const std::vector<point_t> directions = { {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},{0,0}, {0, 1}, {1, -1}, {1, 0}, {1, 1} };

    const std::map<point_t, std::vector<std::pair<int, int>>> target_validators = { {directions[0], {{5, 2}, {7, 6}}},
                                                                                   {directions[1], {{3, 0}, {5, 2}}},
                                                                                   {directions[2], {{3, 0}, {7, 8}}},
                                                                                   {directions[3], {{1, 0}, {7, 6}}},
                                                                                   {directions[5], {{1, 2}, {7, 8}}},
                                                                                   {directions[6], {{1, 0}, {5, 8}}},
                                                                                   {directions[7], {{3, 6}, {5, 8}}},
                                                                                   {directions[8], {{3, 6}, {1, 2}}} };

    bool is_valid_point(const grid_t& grid, const point_t& point)
    {
        const auto [row, column] = point;

        return row >= 0 && row < grid.size() && column >= 0 && column < grid[row].size();
    }

    bool is_force_neighbor(const grid_t& grid, const node_t& node)
    {
        const auto& [point, direction, _, __] = node;

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

    float get_heuristic(const point_t& current, const point_t& goal)
    {
        const auto h = std::abs(current.column - goal.column);
        const auto v = std::abs(current.row - goal.row);

        return std::pow(std::pow(h, 2) + std::pow(v, 2), 0.5);
    }

    node_t jump(const grid_t& grid, const node_t& current, const point_t& goal)
    {
        const auto next_point = current.point + current.direction;

        if (!is_valid_point(grid, next_point) || !grid[next_point.row][next_point.column])
        {
            return current;
        }

        const auto next = node_t(next_point, current.direction, current.duration + get_heuristic({ 0,0 }, current.direction) * grid[next_point.row][next_point.column]);

        if (next_point == goal)
        {
            return next;
        }

        if (current.direction.row != 0 && current.direction.column != 0)
        {
            if (is_force_neighbor(grid, next))
            {
                return next;
            }

            auto wave = next;
            wave.direction = { 0, current.direction.column };
            const auto& h_jump = jump(grid, wave, goal);

            wave.direction = { current.direction.row, 0 };
            const auto& v_jump = jump(grid, wave, goal);

            if (h_jump.point != next_point || v_jump.point != next_point)
            {
                return next;
            }

            const auto& d_jump = jump(grid, next, goal);

            return d_jump.point == next_point ? current : d_jump;
        }
        else
        {
            if (is_force_neighbor(grid, next))
            {
                return next;
            }

            const auto& o_jump = jump(grid, next
                , goal);

            return o_jump.point == next_point ? current : o_jump;
        }
    }

    void grid_scan(jps_t& jps, const grid_t& grid, const node_t& current, const point_t& goal)
    {
        for (const auto& dir : scan_candidates[std::ranges::find(directions, current.direction) - directions.begin()])
        {
            auto node = current;
            node.direction = directions[dir];
            const auto& [point, direction, duration, __] = jump(grid, node, goal);

            if (point == current.point)
            {
                continue;
            }

            if (jps.optimal_duration.count(point) == 0 || duration < jps.optimal_duration[point])
            {
                jps.came_from[point] = current.point;
                jps.optimal_duration[point] = duration;
                jps.open_set.emplace(point, direction, duration, duration + get_heuristic(point, goal));
            }
        }
    }
} // namespace

std::vector<point_t> find_path(const grid_t& gridd, const point_t& start, const point_t& goal)
{
    grid = gridd;
    auto jps = jps_t();
    auto& [_, open_set, optimal_duration, came_from] = jps;

    open_set.emplace(start, point_t(0, 0), 0, get_heuristic(start, goal));
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
                display[i][j] = '1';
            }
        }
    }

    // Mark path
    for (int i = 0; i < path.size() - 1; ++i)
    {
        auto now = path[i];
        auto next = path[i + 1];

        for (auto p = now; p != next; display[p.row][p.column] = '*')
        {
            point_t direction;

            if (p.row > next.row)
            {
                direction.row = -1;
            }
            else if (p.row < next.row)
            {
                direction.row = 1;
            }

            if (p.column > next.column)
            {
                direction.column = -1;
            }
            else if (p.column < next.column)
            {
                direction.column = 1;
            }
            p = p + direction;
        }
    }

    std::ofstream out("output.out", std::ios::trunc);

    if (!out.is_open())
    {
        return;
    }

    // Print grid
    for (const auto& row : display)
    {
        for (char cell : row)
        {
            out << cell << ' ';
        }

        out << '\n';
    }
}