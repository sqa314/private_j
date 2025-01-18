#include "jps_plus.h"

#include <iostream>

int main()
{
    int row_count = 0;
    int column_count = 0;

    std::cin >> row_count >> column_count;

    point_t start;
    point_t goal;

    std::cin >> start.row >> start.column >> goal.row >> goal.column;

    if (row_count == 0 || column_count == 0)
    {
        return -1;
    }

    grid_t grid;

    grid.resize(row_count);

    for (int i = 0; i < row_count; ++i)
    {
        for (int j = 0; j < column_count; ++j)
        {
            int value;
            std::cin >> value;

            grid[i].push_back(value);
        }
    }

    const auto& path = find_path(grid, start, goal);

    if (!path.empty())
    {
        print_path(grid, path);
    }
    else
    {
        std::cout << "no path";
    }

    return 0;
}