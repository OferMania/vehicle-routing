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
#if LOGGING
    logstream << "Vehicle Routing Debug Log" << std::endl;
#endif

    if (argc < 2) {
#if LOGGING
        logstream << "argc needs to be at least 2, found: " << argc << std::endl;
#endif
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

#if LOGGING
    g.debug();
    int many = 60;
    int some = 30;
    int few = 10;
#else
    int many = 360;
    int some = 180;
    int few = 60;
#endif


    std::vector<std::pair<Probs, int>> stuff_to_try = {
        // {Probs(&gen, 1, 0, 0, 0, 0), 1},  // do this once (deterministic solution): always go to HQ, each worker never delivers more than 1 load
        {Probs(&gen, 0, 1, 0, 0, 0, false), 1},  // do this once (deterministic solution): always greedily deliver the nearest load with a single driver, maximizes load per driver
        {Probs(&gen, 0, 0, 1, 0, 0, false), 1},  // do this once (deterministic solution): always greedily deliver the nearest load that's father from HQ (falls back to nearest load), maximizes load per driver
        {Probs(&gen, 0, 0, 0, 1, 0, false), many}, // do this many times: always go to weighted nearest neighbor if possible, with closer neighbors having higher probability
        {Probs(&gen, 0, 0, 0, 0, 1, false), many}, // do this many times: always go to a random neighbor if possible
        {Probs(&gen, 10, 90, 100, 0, 0, false), many}, // do this many times: greedily deliver nearest load with a chance to return early to HQ
        {Probs(&gen, 10, 0, 0, 190, 0, false), many}, // do this many times: weighted neighbor with a chance to return early to HQ
        {Probs(&gen, 10, 0, 0, 0, 190, false), many}, // do this many times: random with chance to return early to HQ
        {Probs(&gen, 10, 45, 45, 100, 0, false), some}, // do this some number of times: weighted btw nearest neighbor vs weighted nearest with a chance to return early to HQ
        {Probs(&gen, 10, 45, 45, 0, 100, false), some}, // do this some number of times: weighted btw nearest neighbor vs random neighbor with a chance to return early to HQ
        {Probs(&gen, 100, 16, 16, 18, 50, false), some}, // do this some number of times: bail to HQ half the time, random neighbor quarter of the time, otherwise other schemes

        {Probs(&gen, 0, 1, 0, 0, 0, true), few},  // few times deterministic nearest neighbor w different starting points, maximize load per driver
        {Probs(&gen, 0, 0, 1, 0, 0, true), few},  // few times deterministic nearest load that's father from HQ, but different starting points
        {Probs(&gen, 10, 90, 0, 0, 0, true), few},  // few times nearest neighbors, 10% chance of early exit, different starting points
        {Probs(&gen, 10, 0, 90, 0, 0, true), few},  // few times nearest neighbors farther from HG, 10% chance of early exit, different starting points
        {Probs(&gen, 10, 45, 45, 100, 0, true), some}, // do this some number of times: different starting points, but weighted btw nearest neighbor vs weighted nearest with a chance to return early to HQ
        {Probs(&gen, 10, 45, 45, 0, 100, true), some}, // do this some number of times: different starting points, but weighted btw nearest neighbor vs random neighbor with a chance to return early to HQ
        {Probs(&gen, 100, 16, 16, 18, 50, true), some}, // do this some number of times: different starting points, but bail to HQ half the time, random neighbor quarter of the time, otherwise other schemes

    };
    
    long double lowest_cost = std::numeric_limits<long double>::infinity();

    std::vector<Coordinate> coordinates = g.getCoordinates();
    std::vector<std::vector<size_t>> best_solution;
    
    // Try a solution that involves giving 1 load to each worker
    for (size_t ii = 1; ii < lines.size(); ++ii) {
        best_solution.push_back({ii});
    }
    if (EvaluateShared::validateSolutionSchedules(best_solution, g.numCoordinates()) != 0) {
        // Uh oh that failed validation??? That's not good. Exit with error.
#if LOGGING
        logstream << "fallback solution failed, bailing" << std::endl;
#endif
        logstream.close();
        return 1;
    } else {
        // Awesome! It worked! That's our lowest cost we will use against the stuff_to_try attempts mentioned above.
        lowest_cost = EvaluateShared::getSolutionCost(coordinates, best_solution, maxMinutes);
    }

#if LOGGING
    logstream << "lowest_cost = " << lowest_cost << std::endl;
#endif

    // For each item in stuff_to_try, run w the probs parameters a # of times specified by num_times (there's randomness involved)
    // Anytime we get something better than the best_solution, we keep that solution
    for (auto& try_it : stuff_to_try) {
        Probs& probs = try_it.first;
        int num_times = try_it.second;
        for (int ii = 0; ii < num_times; ++ii) {
            auto candidate_solution = g.plan_paths(probs);
#if LOGGING
            logstream << "Considering candidate solution:" << std::endl;
            EvaluateShared::outputScheduleToLog(&logstream, candidate_solution);
#endif

            int status = EvaluateShared::validateSolutionSchedules(candidate_solution, g.numCoordinates());
            if (status == 0) {
                long double candidate_cost = EvaluateShared::getSolutionCost(coordinates, candidate_solution, maxMinutes);
#if LOGGING
                logstream << "Candidate passes validation" << std::endl;
                logstream << "candidate_cost = " << candidate_cost << std::endl;
#endif

                if (candidate_cost < lowest_cost) {
                    best_solution = candidate_solution;
                    lowest_cost = candidate_cost;
#if LOGGING
                    logstream << "We found a new best solution" << std::endl;
                    logstream << "Probs is " << probs.to_string() << std::endl;
                    logstream << "lowest_cost = " << lowest_cost << std::endl;
#endif
                }
            } else {
#if LOGGING
                logstream << "Candidate fails validation" << std::endl;
#endif
            }
        }
    }

    // output our best answer!!!
    EvaluateShared::outputSolutionSchedules(best_solution);

    logstream.close();
    return 0;
}
