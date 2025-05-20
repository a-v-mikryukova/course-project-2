#include "path_planning.h"
#include "graph.h"
#include <queue>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <cmath>

using namespace std;


vector<double> dijkstra(const Graph& graph, int start) {
    const auto& nodes = graph.get_nodes();
    const auto& edges = graph.get_edges();
    vector<double> dist(nodes.size(), numeric_limits<double>::infinity());
    std::vector<bool> used(nodes.size());
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
    dist[start] = 0;
    pq.emplace(0, start);
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (used[u]) {
            continue;
        }
        used[u] = true;
        for (const auto& [v, w] : edges[u]) {
            if (dist[v] > dist[u] + w) {
                dist[v] = dist[u] + w;
                pq.emplace(dist[v], v);
            }
        }
    }
    return dist;
}


vector<int> dijkstra_path(const Graph& graph, int start, int goal) {
    const auto& nodes = graph.get_nodes();
    const auto& edges = graph.get_edges();
    vector<double> dist(nodes.size(), numeric_limits<double>::infinity());
    vector<bool> used(nodes.size());
    vector<int> prev(nodes.size());
    vector<int> path;
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
    dist[start] = 0;
    pq.emplace(0, start);
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (used[u]) {
            continue;
        }
        used[u] = true;
        for (const auto& [v, w] : edges[u]) {
            if (dist[v] > dist[u] + w) {
                dist[v] = dist[u] + w;
                pq.emplace(dist[v], v);
                prev[v] = u;
            }
        }
    }
    if (dist[goal] < 1e9) {
        int now = goal;
        path.push_back(now);
        while (now != start) {
            now = prev[now];
            path.push_back(now);
        }
        reverse(path.begin(), path.end());
    } else {
        throw runtime_error("Unable to recover dijkstra path");
    }
    return path;
}

double distanceToSegment(const Point& p, const Point& a, const Point& b) {
    const double l2 = a.distance(b) * a.distance(b);
    if (l2 == 0.0) return p.distance(a);
    const double t = max(0.0, min(1.0, ((p.x - a.x)*(b.x - a.x) + (p.y - a.y)*(b.y - a.y)) / l2));
    const Point projection(a.x + t*(b.x - a.x), a.y + t*(b.y - a.y));
    return p.distance(projection);
}

double minDistance(const vector<Point>& path, const Point& target) {
    if (path.empty()) return numeric_limits<double>::infinity();

    double min_dist = numeric_limits<double>::max();

    for (size_t i = 1; i < path.size(); ++i) {
        const double dist = distanceToSegment(target, path[i-1], path[i]);
        if (dist < min_dist) {
            min_dist = dist;
        }
    }

    return min_dist;
}

vector<int> prioritize_robots(
        const vector<vector<double>>& cost_matrix,
        const vector<int>& assignment,
        const vector<Point>& starts,
        const vector<Point>& goals,
        const Graph& graph
) {
    const double COLLISION_DISTANCE = 2.0 * ROBOT_RADIUS;
    int N = starts.size();
    vector<vector<int>> adj(N);



    for (int i = 0; i < N; ++i) {
        int start_node = graph.find_nearest_node(starts[i]);
        int goal_node = graph.find_nearest_node(goals[assignment[i]]);
        if (cost_matrix[i][assignment[i]] > 10000000) {
            throw runtime_error("Unable to build a path");
        }
        vector<Point> path;
        vector<int> path_dijkstra = dijkstra_path(graph, start_node, goal_node);
        const auto& nodes = graph.get_nodes();
        for (auto & j : path_dijkstra){
            path.push_back(nodes[j]);
        }

        for (int j = 0; j < N; ++j) {
            if (i == j) continue;

            double dist_start = minDistance(path, starts[j]);
            if (dist_start < COLLISION_DISTANCE) {
                adj[j].push_back(i);
            }

            double dist_goal = minDistance(path, goals[assignment[j]]);
            if (dist_goal < COLLISION_DISTANCE) {
                adj[i].push_back(j);
            }
        }
    }

    vector<int> order;
    vector<bool> visited(N, false);
    vector<int> v(N);
    vector<int> empty_v;
    vector<int> ans;
    for (int x = 0; x < N; ++x) {
        if (v[x] == 0) {
            empty_v.push_back(x);
        }
    }
    while (!empty_v.empty()) {
        int last = empty_v.size() - 1;
        int k = empty_v[last];
        ans.push_back(k);
        empty_v.pop_back();
        for (auto r: adj[k]) {
            v[r] -= 1;
            if (v[r] == 0) {
                empty_v.push_back(r);
            }
        }
    }

    if (ans.size() != N) {
        throw runtime_error("Unable to construct topological sorting");
    }
    return ans;
}



optional<SegmentIntersection> findSegmentIntersection(const Point& a1, const Point& a2, const Point& b1, const Point& b2) {
    const double eps = 1e-9;
    const double dx_a = a2.x - a1.x;
    const double dy_a = a2.y - a1.y;
    const double dx_b = b2.x - b1.x;
    const double dy_b = b2.y - b1.y;

    const double det = (-dx_b * dy_a + dx_a * dy_b);
    if (fabs(det) < eps) return nullopt;

    const double s = (-dy_a * (a1.x - b1.x) + dx_a * (a1.y - b1.y)) / det;
    const double t = (dx_b * (a1.y - b1.y) - dy_b * (a1.x - b1.x)) / det;

    if (t >= 0.0 - eps && t <= 1.0 + eps &&
        s >= 0.0 - eps && s <= 1.0 + eps) {
        return SegmentIntersection{
                {a1.x + t * dx_a, a1.y + t * dy_a},
                clamp(t, 0.0, 1.0),
                clamp(s, 0.0, 1.0)
        };
    }
    return nullopt;
}

PathConflict findPathConflicts(
        const vector<Point>& path_a,
        const vector<double>& time_a,
        const vector<Point>& path_b,
        const vector<double>& time_b
) {
    const Point goal_a = path_a[path_a.size() - 1];
    PathConflict conflict;

    double min_goal_dist = numeric_limits<double>::max();
    double b_goal_time = 0.0;

    for (size_t i = 1; i < path_b.size(); ++i) {
        const Point& b1 = path_b[i-1];
        const Point& b2 = path_b[i];

        double dist = distanceToSegment(goal_a, b1, b2);
        double t = clamp(
                ((goal_a.x - b1.x)*(b2.x - b1.x) + (goal_a.y - b1.y)*(b2.y - b1.y)) /
                (pow(b2.x - b1.x, 2) + pow(b2.y - b1.y, 2)),
                0.0, 1.0
        );

        double segment_time = time_b[i-1] + t * (time_b[i] - time_b[i-1]);

        if (dist < min_goal_dist) {
            min_goal_dist = dist;
            b_goal_time = segment_time;
        }
    }

    if (min_goal_dist < ROBOT_RADIUS * 2) {
        double a_goal_time = time_a.empty() ? 0 : time_a.back();
        if (a_goal_time < b_goal_time) {
            conflict.required_delay = max(
                    conflict.required_delay,
                    b_goal_time - a_goal_time + EPSILON
            );
        }
    }

    for (size_t i = 1; i < path_a.size(); ++i) {
        const Point& a1 = path_a[i-1];
        const Point& a2 = path_a[i];
        double a_t1 = time_a[i-1] + conflict.required_delay;
        double a_t2 = time_a[i] + conflict.required_delay;

        for (size_t j = 1; j < path_b.size(); ++j) {
            const Point& b1 = path_b[j-1];
            const Point& b2 = path_b[j];
            const double b_t1 = time_b[j-1];
            const double b_t2 = time_b[j];

            auto intersection = findSegmentIntersection(a1, a2, b1, b2);
            if (!intersection) continue;

            double a_time = a_t1 + intersection->t * (a_t2 - a_t1);
            double b_time = b_t1 + intersection->u * (b_t2 - b_t1);

            if (abs(a_time - b_time) < EPSILON) {
                conflict.required_delay += EPSILON;
                a_t1 += EPSILON;
                a_t2 += EPSILON;

            }
        }
    }

    return conflict;
}


vector<TimedPath> schedulePaths(const vector<Point>& starts, const vector<Point>& goals,
                                const vector<int>& assignment, const vector<int>& priorityOrder,
                                const Graph& graph, double speed) {
    vector<TimedPath> paths(priorityOrder.size());
    vector<TrajectoryInfo> scheduled;

    for (int robot : priorityOrder) {
        TimedPath& path = paths[robot];
        int start_node = graph.find_nearest_node(starts[robot]);
        int goal_node = graph.find_nearest_node(goals[assignment[robot]]);

        vector<Point> full_path;
        vector<int> path_dijkstra = dijkstra_path(graph, start_node, goal_node);
        const auto& nodes = graph.get_nodes();
        for (auto & j : path_dijkstra){
            full_path.push_back(nodes[j]);
        }
        path.points = full_path;
        path.timestamps.resize(full_path.size(), 0);

        for (size_t i = 1; i < full_path.size(); ++i) {
            double dist = full_path[i-1].distance(full_path[i]);
            path.timestamps[i] = path.timestamps[i-1] + dist/speed;
        }

        double delay = 0.0;
        for (const auto& other : scheduled) {
            auto conflicts = findPathConflicts(path.points, path.timestamps, other.path, other.timestamps);
            delay = max(delay, conflicts.required_delay);
        }

        path.start_delay = delay;
        for (auto& t : path.timestamps) t += delay;
        scheduled.push_back({path.points, path.timestamps, delay});
    }

    return paths;
}
