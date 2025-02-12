#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std;

struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}

    double distance(const Point& other) const {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }
};

struct Obstacle {
    vector<Point> vertices;
};

struct Node {
    Point point;
    vector<size_t> neighbors;
};

class PRM {
private:
    vector<Node> nodes;
    vector<Obstacle> obstacles;
    double connection_radius;
    size_t num_samples;
    double robot_radius;
    mt19937 gen;

    bool is_collision_free(const Point& a, const Point& b) const {
        for (const auto& obs : obstacles) {
            for (size_t i = 0; i < obs.vertices.size(); i++) {
                Point p1 = obs.vertices[i];
                Point p2 = obs.vertices[(i+1)%obs.vertices.size()];

                double ua, ub;
                double denom = (p2.y - p1.y) * (b.x - a.x) - (p2.x - p1.x) * (b.y - a.y);
                if (denom == 0) continue;

                ua = ((p2.x - p1.x) * (a.y - p1.y) - (p2.y - p1.y) * (a.x - p1.x)) / denom;
                ub = ((b.x - a.x) * (a.y - p1.y) - (b.y - a.y) * (a.x - p1.x)) / denom;

                if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1)
                    return false;
            }
        }
        return true;
    }

public:
    PRM(int num_samples, double connection_radius, double robot_radius)
            : num_samples(num_samples), connection_radius(connection_radius),
              robot_radius(robot_radius), gen(random_device{}()) {}

    void load_obstacles(const vector<Obstacle>& obs) {
        obstacles = obs;
    }

    void build_roadmap(const Point& min_pt, const Point& max_pt) {
        uniform_real_distribution<double> x_dist(min_pt.x, max_pt.x);
        uniform_real_distribution<double> y_dist(min_pt.y, max_pt.y);

        while (nodes.size() < num_samples) {
            Point p(x_dist(gen), y_dist(gen));
            bool collision = false;

            for (const auto& obs : obstacles) {
                if (!obs.vertices.empty() && p.distance(obs.vertices[0]) < robot_radius) {
                    collision = true;
                    break;
                }
            }

            if (!collision) {
                nodes.push_back({p, {}});
            }
        }

        for (size_t i = 0; i < nodes.size(); ++i) {
            for (size_t j = i+1; j < nodes.size(); ++j) {
                if (nodes[i].point.distance(nodes[j].point) <= connection_radius) {
                    if (is_collision_free(nodes[i].point, nodes[j].point)) {
                        nodes[i].neighbors.push_back(j);
                        nodes[j].neighbors.push_back(i);
                    }
                }
            }
        }
    }

    vector<Point> find_path(const Point& start, const Point& goal) const {
        vector<Node> temp_nodes = nodes;
        size_t start_idx = temp_nodes.size();
        temp_nodes.push_back({start, {}});
        size_t goal_idx = temp_nodes.size();
        temp_nodes.push_back({goal, {}});


        for (size_t i = 0; i < temp_nodes.size(); ++i) {
            for (size_t j = 0; j < temp_nodes.size(); ++j) {
                if (i == j) continue;
                if (temp_nodes[i].point.distance(temp_nodes[j].point) <= connection_radius) {
                    if (is_collision_free(temp_nodes[i].point, temp_nodes[j].point)) {
                        temp_nodes[i].neighbors.push_back(j);
                        temp_nodes[j].neighbors.push_back(i);
                    }
                }
            }
        }

        // Dijkstra's algorithm
        vector<double> dist(temp_nodes.size(), numeric_limits<double>::max());
        vector<int> prev(temp_nodes.size(), -1);
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;

        dist[start_idx] = 0;
        pq.emplace(0, start_idx);

        while (!pq.empty()) {
            auto [cost, u] = pq.top();
            pq.pop();

            if (u == goal_idx) break;

            for (size_t v : temp_nodes[u].neighbors) {
                double new_cost = dist[u] + temp_nodes[u].point.distance(temp_nodes[v].point);
                if (new_cost < dist[v]) {
                    dist[v] = new_cost;
                    prev[v] = u;
                    pq.emplace(new_cost, v);
                }
            }
        }

        // Reconstruct path
        vector<Point> path;
        for (size_t at = goal_idx; at != -1; at = prev[at]) {
            if (at < nodes.size()) {
                path.push_back(temp_nodes[at].point);
            }
        }
        reverse(path.begin(), path.end());
        return path;
    }
};

class HungarianAlgorithm {
public:
    double solve(vector<vector<double>> cost_matrix) {
        size_t n = cost_matrix.size();
        size_t m = cost_matrix[0].size();

        vector<double> u(n+1), v(m+1);
        vector<int> p(m+1), way(m+1);

        for (int i=1; i<=n; ++i) {
            p[0] = i;
            int j0 = 0;
            vector<double> minv(m+1, INFINITY);
            vector<bool> used(m+1, false);

            do {
                used[j0] = true;
                int i0 = p[j0];
                double delta = INFINITY;
                int j1;

                for (int j=1; j<=m; ++j) {
                    if (!used[j]) {
                        double cur = cost_matrix[i0-1][j-1] - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }

                for (int j=0; j<=m; ++j) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }

                j0 = j1;
            } while (p[j0] != 0);

            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0);
        }

        double total_cost = -v[0];
        return total_cost;
    }
};

class Scheduler {
private:
    double robot_radius;

    bool is_within_radius(const Point& p1, const Point& p2) const {
        return p1.distance(p2) < 2 * robot_radius;
    }

public:
    Scheduler(double radius) : robot_radius(radius) {}

    vector<int> prioritize_robots(const vector<Point>& starts,
                                  const vector<Point>& goals,
                                  const vector<vector<Point>>& paths) {
        const int n = starts.size();
        vector<vector<bool>> adj(n, vector<bool>(n, false));

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j) continue;

                // Check path i against start j
                bool near_start = any_of(paths[i].begin(), paths[i].end(),
                                         [&](const Point& p){ return is_within_radius(p, starts[j]); });

                // Check path i against goal j
                bool near_goal = any_of(paths[i].begin(), paths[i].end(),
                                        [&](const Point& p){ return is_within_radius(p, goals[j]); });

                if (near_start) adj[j][i] = true;
                if (near_goal) adj[i][j] = true;
            }
        }

        // Kahn’s algorithm for Topological Sorting
        vector<int> in_degree(n, 0);
        queue<int> q;
        vector<int> order;


        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                if (adj[i][j]) in_degree[j]++;

        for (int i = 0; i < n; ++i)
            if (in_degree[i] == 0) q.push(i);

        while (!q.empty()) {
            int u = q.front();
            q.pop();
            order.push_back(u);

            for (int v = 0; v < n; ++v) {
                if (adj[u][v] && --in_degree[v] == 0) {
                    q.push(v);
                }
            }
        }

        return order;
    }

    vector<double> calculate_time_offsets(const vector<int>& priorities,
                                          const vector<vector<Point>>& paths) {
        //TODO:implement a non-trivial offset calculation

        vector<double> offsets(priorities.size(), 0);
        double current_max = 0;
        for (int i : priorities) {
            offsets[i] = current_max;
            double path_length = 0;
            for (size_t j = 1; j < paths[i].size(); ++j) {
                path_length += paths[i][j].distance(paths[i][j-1]);
            }
            current_max += path_length;
        }

        return offsets;
    }
};

class MultiRobotPlanner {
private:
    PRM prm;
    vector<Point> starts;
    vector<Point> goals;
    double robot_radius;

public:
    MultiRobotPlanner(int samples, double conn_radius, double radius)
            : prm(samples, conn_radius, radius), robot_radius(radius) {}

    void load_config(const string& filename) {
        YAML::Node config = YAML::LoadFile(filename);

        Point min_pt(config["environment"]["min"][0].as<double>(),
                     config["environment"]["min"][1].as<double>());
        Point max_pt(config["environment"]["max"][0].as<double>(),
                     config["environment"]["max"][1].as<double>());

        vector<Obstacle> obstacles;
        for (const auto& obs_node : config["environment"]["obstacles"]) {
            Obstacle obs;
            for (const auto& pt_node : obs_node) {
                obs.vertices.emplace_back(
                        pt_node[0].as<double>(),
                        pt_node[1].as<double>()
                );
            }
            obstacles.push_back(obs);
        }

        prm.load_obstacles(obstacles);
        prm.build_roadmap(min_pt, max_pt);


        for (const auto& pt_node : config["robots"]["starts"]) {
            starts.emplace_back(pt_node[0].as<double>(), pt_node[1].as<double>());
        }
        for (const auto& pt_node : config["robots"]["goals"]) {
            goals.emplace_back(pt_node[0].as<double>(), pt_node[1].as<double>());
        }
    }

    void plan_paths() {
        // 1. Compute all possible paths
        vector<vector<Point>> all_paths(starts.size());
        vector<vector<double>> cost_matrix(starts.size(), vector<double>(goals.size()));

        for (size_t i = 0; i < starts.size(); ++i) {
            for (size_t j = 0; j < goals.size(); ++j) {
                auto path = prm.find_path(starts[i], goals[j]);
                all_paths[i] = path;

                double cost = 0;
                for (size_t k = 1; k < path.size(); ++k) {
                    cost += path[k].distance(path[k-1]);
                }
                cost_matrix[i][j] = cost;
            }
        }

        // 2. Solve assignment problem
        HungarianAlgorithm hungarian;
        double total_cost = hungarian.solve(cost_matrix);

        // 3. Prioritize robots
        Scheduler scheduler(robot_radius);
        auto priorities = scheduler.prioritize_robots(starts, goals, all_paths);

        // 4. Calculate time offsets
        auto offsets = scheduler.calculate_time_offsets(priorities, all_paths);

        cout << "Optimal total cost: " << total_cost << endl;
        cout << "Execution order:\n";
        for (int i : priorities) {
            cout << "Robot " << i << " starts at t=" << offsets[i] << endl;
        }
        //TODO:output of paths, visualization, refinement of trajectories
    }
};

int main(int argc, char** argv) {
    if (argc != 2) {
        cerr << "Wrong input" << endl;
        return 1;
    }

    try {
        MultiRobotPlanner planner(1000, 1.5, 0.5);
        planner.load_config(argv[1]);
        planner.plan_paths();
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    return 0;
}