#pragma once
#include <cmath>

#include "graph.h"

class EvaluateShared {
public:
    static long double distanceBetweenPoints(long double x1, long double y1, long double x2, long double y2) {
        long double xDiff = x1 - x2;
        long double yDiff = y1 - y2;
        return std::sqrt(xDiff*xDiff + yDiff*yDiff);
    }

    static int validateSolutionSchedules(const std::vector<std::vector<size_t>>& solutionSchedules, size_t numCoordinates);

    static long double getDistanceOfScheduleWithReturnHome(const std::vector<size_t>& schedule, const std::vector<Coordinate>& coordinates);

    static long double getSolutionCost(const std::vector<Coordinate>& coordinates, const std::vector<std::vector<size_t>>& solutionSchedules, long double maxMinutes);

    static void outputSolutionSchedules(const std::vector<std::vector<size_t>>& solutionSchedules);
};
