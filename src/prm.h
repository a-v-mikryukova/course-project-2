#pragma once
#include "graph.h"
#include <string>
#include <vector>
#include "constants.h"


class PRM : public Graph {
    std::vector<Point> nodes;
    std::vector<std::vector<std::pair<int, double>>> edges;
    std::vector<std::vector<bool>> grid;
    int width, height;

    bool isPointSafe(const Point& p) const;
    bool isPathSafe(const Point& a, const Point& b) const;

public:
    PRM(const std::string& mapFile,
        const std::vector<Point>& starts,
        const std::vector<Point>& goals,
        int numNodes,
        double connectionRadius);

    int find_nearest_node(const Point& p) const override;
    std::vector<Point> get_nodes() const override;
    std::vector<std::vector<std::pair<int, double>>> get_edges() const override;
};