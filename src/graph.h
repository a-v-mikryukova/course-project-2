#pragma once
#include <vector>
#include <utility>
#include "point.h"

class Graph {
public:
    virtual int find_nearest_node(const Point& p) const = 0;
    virtual std::vector<Point> get_nodes() const = 0;
    virtual std::vector<std::vector<std::pair<int, double>>> get_edges() const = 0;
    virtual ~Graph() = default;
};