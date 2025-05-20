#pragma once

#include "graph.h"
#include "point.h"
#include <vector>
#include <queue>
#include <optional>
#include <stdexcept>
#include <limits>
#include <functional>
#include "constants.h"

struct TimedPath {
    std::vector<Point> points;
    std::vector<double> timestamps;
    double start_delay = 0.0;
};

struct TrajectoryInfo {
    std::vector<Point> path;
    std::vector<double> timestamps;
    double start_delay;
};

struct SegmentIntersection {
    Point point;
    double t;
    double u;
};

struct PathConflict {
    double required_delay = 0.0;
};


std::vector<double> dijkstra(const Graph& graph, int start);
std::vector<int> dijkstra_path(const Graph& graph, int start, int goal);


double distanceToSegment(const Point& p, const Point& a, const Point& b);
double minDistance(const std::vector<Point>& path, const Point& target);


std::vector<int> prioritize_robots(
        const std::vector<std::vector<double>>& cost_matrix,
        const std::vector<int>& assignment,
        const std::vector<Point>& starts,
        const std::vector<Point>& goals,
        const Graph& graph
);


std::optional<SegmentIntersection> findSegmentIntersection(
        const Point& a1, const Point& a2,
        const Point& b1, const Point& b2
);

PathConflict findPathConflicts(
        const std::vector<Point>& path_a,
        const std::vector<double>& time_a,
        const std::vector<Point>& path_b,
        const std::vector<double>& time_b
);


std::vector<TimedPath> schedulePaths(
        const std::vector<Point>& starts,
        const std::vector<Point>& goals,
        const std::vector<int>& assignment,
        const std::vector<int>& priorityOrder,
        const Graph& graph,
        double speed = 1.0
);