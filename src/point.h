#pragma once
#include <cmath>

struct Point {
    double x, y;
    Point(double x = 0, double y = 0);
    double distance(const Point& p) const;
};


inline Point::Point(double x, double y) : x(x), y(y) {}

inline double Point::distance(const Point& p) const {
    return hypot(x - p.x, y - p.y);
}