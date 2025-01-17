#include <functional>
#include <map>
#include <queue>
#include <vector>

using grid_t = std::vector<std::vector<int>>;

struct point_t
{
    int row = 0;
    int column = 0;

    bool operator==(const point_t& r) const
    {
        return row == r.row && column == r.column;
    }
    bool operator<(const point_t& r) const
    {
        return row != r.row ? row < r.row : column < r.column;
    }

    point_t operator+(const point_t& r) const
    {
        return {row + r.row, column + r.column};
    }
};

struct node_t
{
    point_t point;
    float duration = 0;
    float heuristic = 0;
};

struct jps_t
{
    const std::function<bool(const node_t&, const node_t&)> comparator = [](const node_t& l, const node_t& r) {
        return l.heuristic > r.heuristic;
    };

    std::priority_queue<node_t, std::vector<node_t>, decltype(comparator)> open_set{comparator};
    std::map<point_t, float> optimal_duration;
    std::map<point_t, point_t> came_from;
};