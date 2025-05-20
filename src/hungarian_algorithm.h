#pragma once

#include <vector>
#include <limits>
#include <stdexcept>
#include <algorithm>

class HungarianAlgorithm {
public:
    double solve(const std::vector<std::vector<double>>& cost_matrix,
                 std::vector<int>& assignment) {
        const int n = cost_matrix.size();
        if(n == 0) throw std::invalid_argument("Empty cost matrix");
        std::vector<double> u(n+1, 0);
        std::vector<double> v(n+1, 0);
        std::vector<int> p(n+1, 0);
        std::vector<int> way(n+1, 0);

        for(int i = 1; i <= n; ++i) {
            p[0] = i;
            std::vector<double> minv(n+1, std::numeric_limits<double>::max());
            std::vector<bool> used(n+1, false);
            int j0 = 0;

            do {
                used[j0] = true;
                int i0 = p[j0];
                double delta = std::numeric_limits<double>::max();
                int j1 = 0;

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
            } while(p[j0] != 0);

            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while(j0 != 0);
        }

        assignment.resize(n);
        for(int j = 1; j <= n; ++j)
            assignment[p[j]-1] = j-1;

        return -v[0];
    }
};