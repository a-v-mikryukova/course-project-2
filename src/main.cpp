#include "grid_graph.h"
#include "prm.h"
#include "hungarian_algorithm.h"
#include "path_planning.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/stat.h>

using namespace std;

const string RESULT_DIR = "results";
const string MAP_PATH = "../tests/room-64-64-16/map.txt";


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
    starts.clear();
    goals.clear();

    if (!file) {
        throw runtime_error("Can't open test file: " + filename);
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
        int p = 50;
        vector<vector<double>> transformed(cost_matrix.size());
        for (size_t i = 0; i < cost_matrix.size(); ++i) {
            transformed[i].resize(cost_matrix[i].size());
            for (size_t j = 0; j < cost_matrix[i].size(); ++j) {
                transformed[i][j] = pow(cost_matrix[i][j], p);
            }
        }
        hungarian.solve(transformed, assignment);

        auto priority_order = prioritize_robots(cost_matrix, assignment, starts, goals, graph);
        auto timed_paths = schedulePaths(starts, goals, assignment, priority_order, graph);

        save_paths(timed_paths, RESULT_DIR + "/paths_" + to_string(test_count+1) + ".txt");

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
    const string test_dir = "../tests/room-64-64-16/10_agents";
    auto test_files = get_test_files(test_dir);

    double total_sum = 0.0;
    double max_sum = 0.0;
    int test_count = 0;

    mkdir(RESULT_DIR.c_str(), 0777);

    for (const auto& test_file : test_files) {
        cout << "Processing: " << test_file << endl;
        process_test(MAP_PATH, test_file, total_sum, max_sum, test_count);
    }

    if (test_count > 0) {
        cout << "\n=== Результаты тестирования ===" << endl;
        cout << "Успешно обработано тестов: " << test_count << endl;
        cout << "Средняя суммарная длина: " << (total_sum / test_count) << endl;
        cout << "Средняя максимальная длина: " << (max_sum / test_count) << endl;
    } else {
        cerr << "Нет успешно обработанных тестов!" << endl;
    }

    return 0;
}
