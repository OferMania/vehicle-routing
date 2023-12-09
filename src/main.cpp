#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "evaluate_shared.h"
#include "graph.h"
#include <limits>

int main(int argc, char** argv) {
    std::ofstream logstream("debug-log.out");
    logstream << "Vehicle Routing Debug Log" << std::endl;

    if (argc < 2) {
        logstream << "argc needs to be at least 2, found: " << argc << std::endl;
        std::cout << "Name of input txt file must be supplied" << std::endl;
        return 1;
    }
    std::ifstream infile(argv[1]);
    std::string line;
    std::vector<std::string> lines;
    while (std::getline(infile, line))
    {
        lines.push_back(line);
    }
    infile.close();

    long double maxMinutes = 12*60;
    Graph g(lines, &logstream, maxMinutes);
    g.build();
    g.debug();

    // TODO: Enable the stuff to try...
    // // Try out these solutions
    // std::vector<std::pair<Probs, int>> stuff_to_try = {
    //     {Probs(1, 0, 0, 0, 0), 1},  // do this once (deterministic solution): always go to HQ, each worker never delivers more than 1 load
    //     {Probs(0, 1, 0, 0, 0), 1},  // do this once (deterministic solution): always greedily deliver the nearest load with a single driver, maximizes load per driver
    //     {Probs(0, 0, 1, 0, 0), 1},  // do this once (deterministic solution): always greedily deliver the nearest load that's father from HQ (falls back to nearest load), maximizes load per driver
    //     {Probs(0, 0, 0, 1, 0), 20}, // do this 20x: always go to weighted nearest neighbor if possible, with closer neighbors having higher probability
    //     {Probs(0, 0, 0, 0, 1), 20}, // do this 20x: always go to a random neighbor if possible
    //     {Probs(10, 90, 100, 0, 0), 20}, // do this 20x: greedily deliver nearest load with a chance to return early to HQ
    //     {Probs(10, 0, 0, 190, 0), 20}, // do this 20x: weighted neighbor with a chance to return early to HQ
    //     {Probs(10, 0, 0, 0, 190), 20}, // do this 20x: random with chance to return early to HQ
    //     {Probs(10, 45, 45, 100, 0), 20}, // do this 20x: weighted btw nearest neighbor vs weighted nearest with a chance to return early to HQ
    //     {Probs(10, 45, 45, 0, 100), 20}, // do this 20x: weighted btw nearest neighbor vs random neighbor with a chance to return early to HQ
    //     {Probs(100, 16, 16, 18, 50), 20}, // do this 20x: bail to HQ half the time, random neighbor quarter of the time, otherwise other schemes
    // };
    // 
    // long double lowest_cost = std::numeric_limits<long double>::infinity();

    std::vector<Coordinate> coordinates = g.getCoordinates();
    std::vector<std::vector<size_t>> best_solution;
    
    for (size_t ii = 1; ii < lines.size(); ++ii) {
        best_solution.push_back({ii});
    }
    if (EvaluateShared::validateSolutionSchedules(best_solution, g.numCoordinates()) != 0) {
        best_solution.clear();
    } else {
        // TODO: Enable when we're using this variable...
        // lowest_cost = EvaluateShared::getSolutionCost(coordinates, best_solution, maxMinutes);
    }

    // TODO: Enable stuff to try
    // for (const auto& try_it : stuff_to_try) {
    //     const Probs& probs = try_it.first;
    //     int num_times = try_it.second;
    //     for (int ii = 0; ii < num_times; ++ii) {
    //         auto candidate_solution = g.plan_paths(probs);
    //         int status = EvaluateShared::validateSolutionSchedules(candidate_solution, g.numCoordinates());
    //         if (status == 0) {
    //             long double candidate_cost = EvaluateShared::getSolutionCost(coordinates, candidate_solution, maxMinutes);
    //             if (candidate_cost < lowest_cost) {
    //                 best_solution = candidate_solution;
    //                 lowest_cost = candidate_cost;
    //             }
    //         }
    //     }
    // }

    // output our best answer!!!
    EvaluateShared::outputSolutionSchedules(best_solution);

    logstream.close();
    return 0;
}