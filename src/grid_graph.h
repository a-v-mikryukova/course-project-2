#pragma once
#include "graph.h"
#include <string>
#include <vector>
#include "constants.h"

class GridGraph : public Graph {
    struct GridNode {
        int x, y;
        Point center;
    };

    std::vector<std::vector<bool>> grid;
    std::vector<GridNode> nodes;
    std::vector<std::vector<std::pair<int, double>>> edges;
    int width, height;

    bool isPointSafe(const Point& p) const;
    bool isPathSafe(const Point& a, const Point& b) const;

public:
    GridGraph(const std::string& mapFile);
    int find_nearest_node(const Point& p) const override;
    std::vector<Point> get_nodes() const override;
    std::vector<std::vector<std::pair<int, double>>> get_edges() const override;
};