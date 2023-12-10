#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include "coordinate.h"

class EvaluateShared {
public:
    static long double distanceBetweenPoints(long double x1, long double y1, long double x2, long double y2) {
        long double xDiff = x1 - x2;
        long double yDiff = y1 - y2;
        return std::sqrt(xDiff*xDiff + yDiff*yDiff);
    }

    // Redundant method due to lack of time figuring out CPython.
    //
    // Compare this to loadCountOrAssignmentError() method in evaluateShared.py. Returns nonzero on
    // trouble, zero if validation passes.
    static int validateSolutionSchedules(const std::vector<std::vector<size_t>>& solutionSchedules, size_t numCoordinates);

    // Redundant method due to lack of time figuring out CPython.
    //
    // Compare with getDistanceOfScheduleWithReturnHome() method in evaluateShared.py
    static long double getDistanceOfScheduleWithReturnHome(const std::vector<size_t>& schedule, const std::vector<Coordinate>& coordinates);

    // Redundant method due to lack of time figuring out CPython.
    //
    // Compare with getSolutionCost() method in evaluateShared.py, except returns Infinity on error
    // (because that's a large number and we want to minimize the solution cost)
    static long double getSolutionCost(const std::vector<Coordinate>& coordinates, const std::vector<std::vector<size_t>>& solutionSchedules, long double maxMinutes);

    // Outputs a claimed solutionSchedules in the manner we expect evaluateShared.py to see it in 
    static void outputSolutionSchedules(const std::vector<std::vector<size_t>>& solutionSchedules);

    static void outputScheduleToLog(std::ofstream* log, const std::vector<std::vector<size_t>>& solutionSchedules);
};
