#include "prm.h"
#include <fstream>
#include <random>
#include <cmath>
#include <stdexcept>
#include <algorithm>

using namespace std;

PRM::PRM(const string& mapFile,
         const vector<Point>& starts,
         const vector<Point>& goals,
         int numNodes,
         double connectionRadius) :
        width(0), height(0)
{

    ifstream file(mapFile);
    if (!file.is_open()) {
        throw runtime_error("Failed to open map file: " + mapFile);
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

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> distX(0 + OBSTACLE_MARGIN, width - OBSTACLE_MARGIN);
    uniform_real_distribution<double> distY(0 + OBSTACLE_MARGIN, height - OBSTACLE_MARGIN);

    for (const auto& p : starts) {
        if (!isPointSafe(p))
            throw runtime_error("Start position is in collision!");
        nodes.push_back(p);
    }
    for (const auto& p : goals) {
        if (!isPointSafe(p))
            throw runtime_error("Goal position is in collision!");
        nodes.push_back(p);
    }


    while (nodes.size() < numNodes) {
        Point p(distX(gen), distY(gen));
        int x = static_cast<int>(p.x), y = static_cast<int>(p.y);
        if (isPointSafe(p)) nodes.push_back(p);
    }
    edges.resize(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i+1; j < nodes.size(); ++j) {
            if (nodes[i].distance(nodes[j]) <= connectionRadius &&
                isPathSafe(nodes[i], nodes[j])) {
                double dist = nodes[i].distance(nodes[j]);
                edges[i].emplace_back(j, dist);
                edges[j].emplace_back(i, dist);
            }
        }
    }
}

bool PRM::isPointSafe(const Point& p) const {
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

bool PRM::isPathSafe(const Point& a, const Point& b) const {
    const int steps = 100;
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i)/steps;
        Point p(a.x + t*(b.x - a.x),
                a.y + t*(b.y - a.y));
        if (!isPointSafe(p)) return false;
    }
    return true;
}

int PRM::find_nearest_node(const Point& p) const {
    int best = 0;
    double min_dist = numeric_limits<double>::max();
    for (size_t i = 0; i < nodes.size(); ++i) {
        double d = p.distance(nodes[i]);
        if (d < min_dist) {
            min_dist = d;
            best = i;
        }
    }
    return best;
}

vector<Point> PRM::get_nodes() const {
    return nodes;
}

vector<vector<pair<int, double>>> PRM::get_edges() const {
    return edges;
}
