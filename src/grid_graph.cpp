#include "grid_graph.h"
#include <fstream>
#include <cmath>
#include <vector>
#include <limits>

using namespace std;

GridGraph::GridGraph(const string& mapFile) {
    ifstream file(mapFile);
    if (!file.is_open()) {
        throw runtime_error("Failed to open map file");
    }

    string line;
    while (getline(file, line)) {
        grid.push_back(vector<bool>(line.size()));
        for (size_t i = 0; i < line.size(); ++i) {
            grid.back()[i] = (line[i] != '.');
        }
    }

    height = grid.size();
    if (height == 0) throw runtime_error("Empty map");
    width = grid[0].size();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (!grid[y][x]) {
                nodes.push_back({x, y, Point(x + 0.5, y + 0.5)});
            }
        }
    }

    edges.resize(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i+1; j < nodes.size(); ++j) {
            const auto& node_i = nodes[i];
            const auto& node_j = nodes[j];

            int dx = abs(node_i.x - node_j.x);
            int dy = abs(node_i.y - node_j.y);
            if (dx <= 1 && dy <= 1) {
                double dist =  node_i.center.distance(node_j.center);
                if (isPathSafe(node_i.center, node_j.center)) {
                    edges[i].emplace_back(j, dist);
                    edges[j].emplace_back(i, dist);
                }
            }
        }
    }
}

bool GridGraph::isPointSafe(const Point& p) const {
    int x_min = static_cast<int>(floor(p.x - OBSTACLE_MARGIN));
    int x_max = static_cast<int>(ceil(p.x + OBSTACLE_MARGIN));
    int y_min = static_cast<int>(floor(p.y - OBSTACLE_MARGIN));
    int y_max = static_cast<int>(ceil(p.y + OBSTACLE_MARGIN));

    int obs = 0;
    bool is_safe = true;

    for (int y = y_min; y < y_max; ++y) {
        for (int x = x_min; x < x_max; ++x) {
            if (x < 0 || y < 0 || x > width || y > height){
                return false;
            }
            if (grid[y][x]) {
                is_safe = false;
                obs += 1;
            }

        }
    }
    if ((y_max-y_min) == 2 && (x_max-x_min) == 2 && obs == 1){
        Point centre(x_min + 1, y_min + 1);
        if (p.distance(centre) >= OBSTACLE_MARGIN) {
            is_safe = true;
        }

    }
    return is_safe;
}

bool GridGraph::isPathSafe(const Point& a, const Point& b) const {
    const int steps = 100;
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i)/steps;
        Point p(a.x + t*(b.x - a.x),
                a.y + t*(b.y - a.y));
        if (!isPointSafe(p)) return false;
    }
    return true;
}

int GridGraph::find_nearest_node(const Point& p) const {
    int best = 0;
    double min_dist = numeric_limits<double>::max();
    for (size_t i = 0; i < nodes.size(); ++i) {
        double d = p.distance(nodes[i].center);
        if (d < min_dist) {
            min_dist = d;
            best = i;
        }
    }
    return best;
}

vector<Point> GridGraph::get_nodes() const {
    vector<Point> result;
    for (const auto& node : nodes)
        result.push_back(node.center);
    return result;
}

vector<vector<pair<int, double>>> GridGraph::get_edges() const {
    return edges;
}
