#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <queue>
#include <tuple>
#include <functional>
#include <limits>
#include <set>
#include <dirent.h>
#include <sys/stat.h>
#include <optional>
#include <sstream>

using namespace std;


constexpr double ROBOT_RADIUS = 0.01;
constexpr double OBSTACLE_MARGIN = ROBOT_RADIUS;
constexpr double EPSILON = 0.05;


struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
    double distance(const Point& p) const {
        return hypot(x - p.x, y - p.y);
    }
};

class Graph {
public:
    virtual int find_nearest_node(const Point& p) const = 0;
    virtual vector<Point> get_nodes() const = 0;
    virtual vector<vector<pair<int, double>>> get_edges() const = 0;
    virtual ~Graph() = default;
};


class GridGraph :  public Graph {
private:
    struct GridNode {
        int x;
        int y;
        Point center;
    };

    vector<vector<bool>> grid;
    vector<GridNode> nodes;
    vector<vector<pair<int, double>>> edges;
    int width, height;

    bool is_point_safe(const Point& p) const {
        int x_min = static_cast<int>(floor(p.x - OBSTACLE_MARGIN));
        int x_max = static_cast<int>(ceil(p.x + OBSTACLE_MARGIN));
        int y_min = static_cast<int>(floor(p.y - OBSTACLE_MARGIN));
        int y_max = static_cast<int>(ceil(p.y + OBSTACLE_MARGIN));

        bool is_save = true;
        int obs = 0;

        for (int y = y_min; y < y_max; ++y) {
            for (int x = x_min; x < x_max; ++x) {
                if (x < 0 || y < 0 || x > width || y > height){
                    return false;
                }
                if (grid[y][x]) {
                    is_save = false;
                    obs += 1;
                }

            }
        }
        if ((y_max-y_min) == 2 && (x_max-x_min) == 2 && obs == 1){
            Point centre(x_min + 1, y_min + 1);
            if (p.distance(centre) >= OBSTACLE_MARGIN) {
                is_save = true;
            }

        }
        return is_save;
    }

    bool is_path_safe(const Point& a, const Point& b) const {
        const int steps = 100;
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i)/steps;
            Point p(a.x + t*(b.x - a.x),
                    a.y + t*(b.y - a.y));
            if (!is_point_safe(p)) return false;
        }
        return true;
    }



public:
    GridGraph(const string& mapFile) {
        ifstream file(mapFile);
        string line;
        while (getline(file, line)) {
            grid.push_back(vector<bool>(line.size()));
            for (size_t i = 0; i < line.size(); ++i)
                grid.back()[i] = (line[i] != '.');
        }
        height = grid.size();
        width = grid[0].size();

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (!grid[y][x]) {
                    nodes.push_back({x, y,Point(x + 0.5,
                                                y + 0.5)});
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
                    if (is_path_safe(node_i.center, node_j.center)) {
                        edges[i].emplace_back(j, dist);
                        edges[j].emplace_back(i, dist);
                    }
                }
            }
        }
    }

    int find_nearest_node(const Point& p) const override {
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

    vector<Point> get_nodes() const override{
        vector<Point> result;
        for (const auto& node : nodes)
            result.push_back(node.center);
        return result;
    }

    vector<vector<pair<int, double>>> get_edges() const override {
        return edges;
    }
};


class PRM :  public Graph {
private:
    vector<Point> nodes;
    vector<vector<pair<int, double>>> edges;
    vector<vector<bool>> grid;
    int width, height;

    bool is_point_safe(const Point& p) const {
        int x_min = static_cast<int>(floor(p.x - OBSTACLE_MARGIN));
        int x_max = static_cast<int>(ceil(p.x + OBSTACLE_MARGIN));
        int y_min = static_cast<int>(floor(p.y - OBSTACLE_MARGIN));
        int y_max = static_cast<int>(ceil(p.y + OBSTACLE_MARGIN));

        bool is_save = true;
        int obs = 0;

        for (int y = y_min; y < y_max; ++y) {
            for (int x = x_min; x < x_max; ++x) {
                if (x < 0 || y < 0 || x > width || y > height){
                    return false;
                }
                if (grid[y][x]) {
                    is_save = false;
                    obs += 1;
                }

            }
        }
        if ((y_max-y_min) == 2 && (x_max-x_min) == 2 && obs == 1){
            Point centre(x_min + 1, y_min + 1);
            if (p.distance(centre) >= OBSTACLE_MARGIN) {
                is_save = true;
            }

        }
        return is_save;
    }


    bool is_path_safe(const Point& a, const Point& b) const {
        const int steps = 100;
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i)/steps;
            Point p(a.x + t*(b.x - a.x),
                    a.y + t*(b.y - a.y));
            if (!is_point_safe(p)) return false;
        }
        return true;
    }


public:
    PRM(const string& map_file, const vector<Point>& starts,
        const vector<Point>& goals, int num_nodes, double connection_radius) {
        ifstream file(map_file);
        string line;
        while (getline(file, line)) {
            grid.push_back(vector<bool>(line.size()));
            for (size_t i = 0; i < line.size(); ++i)
                grid.back()[i] = (line[i] != '.');
        }
        height = grid.size();
        width = grid[0].size();

        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<double> distX(0 + OBSTACLE_MARGIN, width - OBSTACLE_MARGIN);
        uniform_real_distribution<double> distY(0 + OBSTACLE_MARGIN, height - OBSTACLE_MARGIN);

        for (const auto& p : starts) {
            if (!is_point_safe(p))
                throw runtime_error("Start position is in collision!");
            nodes.push_back(p);
        }
        for (const auto& p : goals) {
            if (!is_point_safe(p))
                throw runtime_error("Goal position is in collision!");
            nodes.push_back(p);
        }

        while (nodes.size() < num_nodes) {
            Point p(distX(gen), distY(gen));
            int x = static_cast<int>(p.x), y = static_cast<int>(p.y);
            if (is_point_safe(p)) nodes.push_back(p);
        }
        edges.resize(nodes.size());
        for (size_t i = 0; i < nodes.size(); ++i) {
            for (size_t j = i+1; j < nodes.size(); ++j) {
                if (nodes[i].distance(nodes[j]) <= connection_radius &&
                        is_path_safe(nodes[i], nodes[j])) {
                    double dist = nodes[i].distance(nodes[j]);
                    edges[i].emplace_back(j, dist);
                    edges[j].emplace_back(i, dist);
                }
            }
        }
    }


    vector<Point> get_nodes() const override { return nodes; }
    vector<vector<pair<int, double>>> get_edges() const override { return edges; }

    int find_nearest_node(const Point& p) const override {
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
};

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
        throw runtime_error("Не удаётся восстановить путь Дейкстры");
    }
    return path;
}


class HungarianAlgorithm {
public:
    double solve(const vector<vector<double>>& cost_matrix, vector<int>& assignment) {
        int n = cost_matrix.size();
        vector<double> u(n+1), v(n+1);
        vector<int> p(n+1), way(n+1);

        for (int i = 1; i <= n; ++i) {
            p[0] = i;
            vector<double> minv(n+1, numeric_limits<double>::max());
            vector<bool> used(n+1, false);
            int j0 = 0;
            do {
                used[j0] = true;
                int i0 = p[j0];
                double delta = numeric_limits<double>::max();
                int j1;
                for (int j = 1; j <= n; ++j) {
                    if (!used[j]) {
                        double cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
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
                for (int j = 0; j <= n; ++j) {
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
            } while (j0 != 0);
        }

        assignment.resize(n);
        for (int j = 1; j <= n; ++j)
            assignment[p[j]-1] = j-1;

        return -v[0];
    }
};

double distance_to_segment(const Point& p, const Point& a, const Point& b) {
    const double l2 = a.distance(b) * a.distance(b);
    if (l2 == 0.0) return p.distance(a);
    const double t = max(0.0, min(1.0, ((p.x - a.x)*(b.x - a.x) + (p.y - a.y)*(b.y - a.y)) / l2));
    const Point projection(a.x + t*(b.x - a.x), a.y + t*(b.y - a.y));
    return p.distance(projection);
}


double min_distance(const vector<Point>& path, const Point& target) {
    if (path.empty()) return numeric_limits<double>::infinity();

    double min_dist = numeric_limits<double>::max();

    for (size_t i = 1; i < path.size(); ++i) {
        const double dist = distance_to_segment(target, path[i-1], path[i]);
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
            throw runtime_error("Не удаётся построить путь");
        }
        vector<Point> path;
        vector<int> path_dijkstra = dijkstra_path(graph, start_node, goal_node);
        const auto& nodes = graph.get_nodes();
        for (auto & j : path_dijkstra){
            path.push_back(nodes[j]);
        }

        for (int j = 0; j < N; ++j) {
            if (i == j) continue;

            double dist_start = min_distance(path, starts[j]);
            if (dist_start < COLLISION_DISTANCE) {
                adj[j].push_back(i);
            }

            double dist_goal = min_distance(path, goals[assignment[j]]);
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
        throw runtime_error("Не удаётся постройти топологическую сортировку");
    }
    return ans;
}


struct TimedPath {
    vector<Point> points;
    vector<double> timestamps;
    double start_delay = 0.0;
};



struct TrajectoryInfo {
    vector<Point> path;
    vector<double> timestamps;
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


optional<SegmentIntersection> find_segment_intersection(const Point& a1, const Point& a2, const Point& b1, const Point& b2) {
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


PathConflict find_path_conflicts(
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

        double dist = distance_to_segment(goal_a, b1, b2);
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

            auto intersection = find_segment_intersection(a1, a2, b1, b2);
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



vector<TimedPath> schedule_paths(const vector<Point>& starts, const vector<Point>& goals,
                                const vector<int>& assignment, const vector<int>& priority_order,
                                const Graph& graph, double speed = 1.0) {
    vector<TimedPath> paths(priority_order.size());
    vector<TrajectoryInfo> scheduled;

    for (int robot : priority_order) {
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
            auto conflicts = find_path_conflicts(path.points, path.timestamps, other.path, other.timestamps);
            delay = max(delay, conflicts.required_delay);
        }

        path.start_delay = delay;
        for (auto& t : path.timestamps) t += delay;
        scheduled.push_back({path.points, path.timestamps, delay});
    }

    return paths;
}

void save_paths(const vector<TimedPath>& paths, const string& filename) {
    ofstream file(filename);
    for (size_t i = 0; i < paths.size(); ++i) {
        file << "Robot " << i << ":\n";
        file << "Start delay: " << paths[i].start_delay << "\n";
        for (size_t j = 0; j < paths[i].points.size(); ++j) {
            file << paths[i].points[j].x << ","
                 << paths[i].points[j].y << "\n";
        }
    }
}



void read_points(const string& filename, vector<Point>& starts, vector<Point>& goals) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Не существует тестового файла" << endl;
        return;
    }

    string line;
    for (int i = 0; i < 6; ++i) {
        if (!getline(file, line)) {
            return;
        }
    }

    while (getline(file, line)) {
        istringstream str(line);
        string token;
        vector<double> coords;

        while (getline(str, token, ',')) {
            try {
                coords.push_back(stod(token));
            } catch (...) {
                coords.clear();
                break;
            }
        }
        if (coords.size() == 4) {
            starts.emplace_back(coords[0] + 0.5, coords[1] + 0.5);
            goals.emplace_back(coords[2] + 0.5, coords[3] + 0.5);
        }
    }
}


vector<string> get_test_files(const string& dir_path) {
    vector<string> files;
    for (int i = 1; i <= 50; ++i) {
        files.push_back(dir_path + "/test" + to_string(i) + ".txt");
    }
    return files;
}

void process_test(const string& map_path,
                  const string& test_file,
                  double& total_sum,
                  double& max_sum,
                  int& test_count) {

    vector<Point> starts, goals;
    read_points(test_file, starts, goals);

    try {
        //PRM graph(map_path, starts, goals, 2000, 5.0);
        GridGraph graph(map_path);
        vector<vector<double>> cost_matrix(starts.size(), vector<double>(goals.size()));
        for (size_t i = 0; i < starts.size(); ++i) {
            int start_node = graph.find_nearest_node(starts[i]);
            auto dist = dijkstra(graph, start_node);
            for (size_t j = 0; j < goals.size(); ++j) {
                int goal_node = graph.find_nearest_node(goals[j]);
                cost_matrix[i][j] = dist[goal_node];
            }
        }

        HungarianAlgorithm hungarian;
        vector<int> assignment;
        int p = 1;
        vector<vector<double>> transformed(cost_matrix.size());
        for (size_t i = 0; i < cost_matrix.size(); ++i) {
            transformed[i].resize(cost_matrix[i].size());
            for (size_t j = 0; j < cost_matrix[i].size(); ++j) {
                transformed[i][j] = pow(cost_matrix[i][j], p);
            }
        }
        hungarian.solve(transformed, assignment);

        auto priority_order = prioritize_robots(cost_matrix, assignment, starts, goals, graph);
        auto timed_paths = schedule_paths(starts, goals, assignment, priority_order, graph);

        save_paths(timed_paths, "/mapf-2/paths.txt");

        double test_total = 0.0;
        double test_max = 0.0;
        for (const auto& tp : timed_paths) {
            double length = 0.0;
            for (size_t j = 1; j < tp.points.size(); ++j) {
                length += tp.points[j-1].distance(tp.points[j]);
            }
            length += tp.start_delay;
            test_total += length;
            test_max = max(test_max, length);
        }

        total_sum += test_total;
        max_sum += test_max;
        test_count++;

    } catch (const exception& e) {
        cerr << "Error processing " << test_file << ": " << e.what() << endl;
    }
}

int main() {
    const string map_path = "/tests/room-64-64-16/map.txt";
    const string test_dir = "/tests/room-64-64-16/10_agents";
    auto test_files = get_test_files(test_dir);
    double total_sum = 0.0;
    double max_sum = 0.0;
    int test_count = 0;

    for (const auto& test_file : test_files) {
        cout << "Processing: " << test_file << endl;
        process_test(map_path, test_file, total_sum, max_sum, test_count);
    }

    if (test_count > 0) {
        cout << "\n=== Результаты тестирования ===" << endl;
        cout << "Количество успешных тестов: " << test_count << endl;
        cout << "Средняя суммарная длина: " << (total_sum / test_count) << endl;
        cout << "Средняя максимальная длина: " << (max_sum / test_count) << endl;
    } else {
        cerr << "Нет успешно обработанных тестов!" << endl;
    }

    return 0;
}
