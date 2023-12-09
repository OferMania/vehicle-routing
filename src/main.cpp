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

    // NOTE: Considering 64-bit random generator, but worried about runtime, so sticking to standard mt19937 for now
    std::random_device rd;
    std::mt19937 gen(rd());

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

    std::vector<std::pair<Probs, int>> stuff_to_try = {
        // {Probs(&gen, 1, 0, 0, 0, 0), 1},  // do this once (deterministic solution): always go to HQ, each worker never delivers more than 1 load
        {Probs(&gen, 0, 1, 0, 0, 0, false), 1},  // do this once (deterministic solution): always greedily deliver the nearest load with a single driver, maximizes load per driver
        {Probs(&gen, 0, 0, 1, 0, 0, false), 1},  // do this once (deterministic solution): always greedily deliver the nearest load that's father from HQ (falls back to nearest load), maximizes load per driver
        {Probs(&gen, 0, 0, 0, 1, 0, false), 60}, // do this 60x: always go to weighted nearest neighbor if possible, with closer neighbors having higher probability
        {Probs(&gen, 0, 0, 0, 0, 1, false), 60}, // do this 60x: always go to a random neighbor if possible
        {Probs(&gen, 10, 90, 100, 0, 0, false), 60}, // do this 60x: greedily deliver nearest load with a chance to return early to HQ
        {Probs(&gen, 10, 0, 0, 190, 0, false), 60}, // do this 60x: weighted neighbor with a chance to return early to HQ
        {Probs(&gen, 10, 0, 0, 0, 190, false), 60}, // do this 60x: random with chance to return early to HQ
        {Probs(&gen, 10, 45, 45, 100, 0, false), 30}, // do this 30x: weighted btw nearest neighbor vs weighted nearest with a chance to return early to HQ
        {Probs(&gen, 10, 45, 45, 0, 100, false), 30}, // do this 30x: weighted btw nearest neighbor vs random neighbor with a chance to return early to HQ
        {Probs(&gen, 100, 16, 16, 18, 50, false), 30}, // do this 30x: bail to HQ half the time, random neighbor quarter of the time, otherwise other schemes

        {Probs(&gen, 0, 1, 0, 0, 0, true), 10},  // deterministic nearest neighbor w 10 different starting points, maximize load per driver
        {Probs(&gen, 0, 0, 1, 0, 0, true), 10},  // deterministic nearest load that's father from HQ, but 10 different starting points
        {Probs(&gen, 10, 90, 0, 0, 0, true), 10},  // 10x nearest neighbors, 10% chance of early exit, different starting points
        {Probs(&gen, 10, 0, 90, 0, 0, true), 10},  // 10x nearest neighbors farther from HG, 10% chance of early exit, different starting points
        {Probs(&gen, 10, 45, 45, 100, 0, true), 30}, // do this 30x: different starting points, but weighted btw nearest neighbor vs weighted nearest with a chance to return early to HQ
        {Probs(&gen, 10, 45, 45, 0, 100, true), 30}, // do this 30x: different starting points, but weighted btw nearest neighbor vs random neighbor with a chance to return early to HQ
        {Probs(&gen, 100, 16, 16, 18, 50, true), 30}, // do this 30x: different starting points, but bail to HQ half the time, random neighbor quarter of the time, otherwise other schemes

    };
    
    long double lowest_cost = std::numeric_limits<long double>::infinity();

    std::vector<Coordinate> coordinates = g.getCoordinates();
    std::vector<std::vector<size_t>> best_solution;
    
    for (size_t ii = 1; ii < lines.size(); ++ii) {
        best_solution.push_back({ii});
    }
    if (EvaluateShared::validateSolutionSchedules(best_solution, g.numCoordinates()) != 0) {
        logstream << "fallback solution failed, bailing" << std::endl;
        logstream.close();
        return 1;
    } else {
        lowest_cost = EvaluateShared::getSolutionCost(coordinates, best_solution, maxMinutes);
    }
    logstream << "lowest_cost = " << lowest_cost << std::endl;

    for (auto& try_it : stuff_to_try) {
        Probs& probs = try_it.first;
        int num_times = try_it.second;
        for (int ii = 0; ii < num_times; ++ii) {
            auto candidate_solution = g.plan_paths(probs);

            logstream << "Considering candidate solution:" << std::endl;
            EvaluateShared::outputScheduleToLog(&logstream, candidate_solution);

            int status = EvaluateShared::validateSolutionSchedules(candidate_solution, g.numCoordinates());
            if (status == 0) {
                logstream << "Candidate passes validation" << std::endl;
                long double candidate_cost = EvaluateShared::getSolutionCost(coordinates, candidate_solution, maxMinutes);
                logstream << "candidate_cost = " << candidate_cost << std::endl;
                if (candidate_cost < lowest_cost) {
                    logstream << "We found a new best solution" << std::endl;
                    logstream << "Probs is " << probs.to_string() << std::endl;
                    best_solution = candidate_solution;
                    lowest_cost = candidate_cost;

                    logstream << "lowest_cost = " << lowest_cost << std::endl;
                }
            } else {
                logstream << "Candidate fails validation" << std::endl;
            }
        }
    }

    // output our best answer!!!
    EvaluateShared::outputSolutionSchedules(best_solution);

    logstream.close();
    return 0;
}