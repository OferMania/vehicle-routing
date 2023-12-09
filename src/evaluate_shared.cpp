#include "evaluate_shared.h"

#include <unordered_set>
#include <iostream>
#include <limits>

int EvaluateShared::validateSolutionSchedules(const std::vector<std::vector<size_t>>& solutionSchedules, size_t numCoordinates) {
    size_t numLoads = numCoordinates - 1;   // HQ isn't a load, but all other coordinates are

    std::unordered_set<size_t> solutionLoadIds;
    for (const auto& schedule : solutionSchedules) {
        for (size_t loadId : schedule) {
            if (solutionLoadIds.find(loadId) != solutionLoadIds.end()) {
                // load loadId was included in at least two driver schedules
                return 2;
            }
            solutionLoadIds.insert(loadId);
        }
    }

    if (solutionLoadIds.size() != numLoads) {
        // the solution load count isn't equal to the known load count (ie numLoads)
        return 1;
    }

    for (size_t loadId = 1; loadId <= numLoads; ++loadId) {
        if (solutionLoadIds.find(loadId) == solutionLoadIds.end()) {
            // load loadId was not assigned to a driver
            return 3;
        }
    }

    // Yay! We passed validations!
    return 0;
}


long double EvaluateShared::getDistanceOfScheduleWithReturnHome(const std::vector<size_t>& schedule, const std::vector<Coordinate>& coordinates)
{
    long double distance = 0;
    long double currentX = 0;
    long double currentY = 0;
    for (size_t loadId : schedule) {
        const Coordinate& load = coordinates[loadId];
        // to pickup
        distance += distanceBetweenPoints(currentX, currentY, load.pickupX, load.pickupY);
        currentX = load.pickupX;
        currentY = load.pickupY;
        // to dropoff
        distance += distanceBetweenPoints(currentX, currentY, load.dropOffX, load.dropOffY);
        currentX = load.dropOffX;
        currentY = load.dropOffY;
    }
    distance += distanceBetweenPoints(currentX, currentY, 0, 0);
    return distance;
}

long double EvaluateShared::getSolutionCost(const std::vector<Coordinate>& coordinates, const std::vector<std::vector<size_t>>& solutionSchedules, long double maxMinutes) {
    long double totalDrivenMinutes = 0;
    for (size_t ii = 0; ii < solutionSchedules.size(); ++ii) {
        const auto& schedule = solutionSchedules[ii];
        long double scheduleMinutes = getDistanceOfScheduleWithReturnHome(schedule, coordinates);
        if (scheduleMinutes > maxMinutes) {
            // TODO: error, log it here!!!
            return std::numeric_limits<long double>::infinity();
        }
        totalDrivenMinutes += scheduleMinutes;
    }
    return 500.L*solutionSchedules.size() + totalDrivenMinutes;
}

void EvaluateShared::outputSolutionSchedules(const std::vector<std::vector<size_t>>& solutionSchedules) {
    for (const auto& schedule : solutionSchedules) {
        std::cout << "[";
        bool seenFirst = false;
        for (size_t loadId : schedule) {
            if (seenFirst) {
                std::cout << ",";
            }
            std::cout << loadId;
            seenFirst = true;
        }
        std::cout << "]" << std::endl;
    }
}