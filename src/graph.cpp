#include <bits/stdc++.h> 

#include "evaluate_shared.h"

#include "graph.h"

void Graph::build() {
    build_coordinates();
    build_distance_matrix();
}

void Graph::debug() {
    _log->precision(20);
    *_log << "Lines: " << std::endl;
    for (const auto& line : _lines) {
        *_log << line << std::endl;
    }
    *_log << "Coordinates: " << std::endl;
    for (const auto& coord : _coordinates) {
        *_log << "Pickup = (" 
            << coord.pickupX << ", " << coord.pickupY << "), Dropoff = (" 
            << coord.dropOffX << ", " << coord.dropOffY << ")" << std::endl;
    }
}

void Graph::build_coordinates() {
    if (_lines.size() < 2) {
        // There's no graph to build. We need at least 2 lines to build a graph
        return;
    }

    _coordinates = std::vector(_lines.size(), Coordinate());

    for (size_t ii = 1; ii < _lines.size(); ++ii) {
        std::string line = _lines[ii];
        std::stringstream ss(line);
        std::vector<std::string> args;
        std::string piece;

        // split the line by space to build args
        while (std::getline(ss, piece, ' ')) {
            args.push_back(piece);
        }

        if (args.size() < 3) {
            // We need at least 3 args for this to be a valid route
            *_log << "We need at least 3 args on this line: " << line << std::endl;
            continue;
        }

        if (args[1].size() < 3 || args[2].size() < 3) {
            // because args[1] and args[2] are expected to be in format (x,y), we expect each of them to have at least 3 characters
            *_log << "Malformed coordinate format on line: " << line << std::endl;
            continue;
        }

        bool startParen1 = (!args[1].empty() && args[1][0] == '(');
        bool endParen1 = (!args[1].empty() && args[1][args[1].size()-1] == ')');
        size_t length1 = args[1].size() - (startParen1 ? 1 : 0) - (endParen1 ? 1 : 0);
        bool startParen2 = (!args[2].empty() && args[2][0] == '(');
        bool endParen2 = (!args[2].empty() && args[2][args[2].size()-1] == ')');
        size_t length2 = args[2].size() - (startParen2 ? 1 : 0) - (endParen2 ? 1 : 0);
        std::string coord1 = args[1].substr(startParen1 ? 1 : 0, length1);
        std::string coord2 = args[2].substr(startParen2 ? 1 : 0, length2);

        std::stringstream ssc1(coord1);
        std::vector<std::string> coords1;
        while (std::getline(ssc1, piece, ',')) {
            coords1.push_back(piece);
        }

        std::stringstream ssc2(coord2);
        std::vector<std::string> coords2;
        while (std::getline(ssc2, piece, ',')) {
            coords2.push_back(piece);
        }

        if (coords1.size() < 2 || coords2.size() < 2) {
            // We need X and Y for coords. Too few pieces for coords1 or coords2 is therefore a problem
            *_log << "Malformed dimensions on line: " << line << std::endl;
            continue;
        }

        size_t index = static_cast<size_t>(std::stoi(args[0]));
        // Sigh, index is typically equivalent to ii, but we try not to take any chances
        if (index < 0 || index >= _lines.size()) {
            *_log << "Invalid index " << index << " found on line: " << line << std::endl;
            continue;
        }

        Coordinate* coord = &_coordinates[index];
        coord->pickupX = std::stold(coords1[0]);
        coord->pickupY = std::stold(coords1[1]);
        coord->dropOffX = std::stold(coords2[0]);
        coord->dropOffY = std::stold(coords2[1]);
    }
}

void Graph::build_distance_matrix() {
    for (size_t ii = 0; ii < _coordinates.size(); ++ii) {
        _distance_matrix.push_back(std::vector<long double>(_coordinates.size(), 0));
    }

    for (size_t to_load = 0; to_load < _coordinates.size(); ++to_load) {
        const Coordinate* to_coord = &_coordinates[to_load];
        long double to_load_pickup_dropoff_distance = EvaluateShared::distanceBetweenPoints(to_coord->pickupX, to_coord->pickupY, to_coord->dropOffX, to_coord->dropOffY);
        for (size_t from_load = 0; from_load < _coordinates.size(); ++from_load) {
            if (from_load == to_load) {
                continue;
            }
            const Coordinate* from_coord = &_coordinates[from_load];
            // measure from_load dropOff to to_load pickup

            // matrix[from_load][to_load] measures time FROM from_load's dropOff to to_load's pickup PLUS to_load's pickup to to_load's dropOff
            // ie matrix[from_load][to_load] is the total distance traveled after finishing at from_load to performing ALL work of to_load immediately after

            long double switch_from_to = EvaluateShared::distanceBetweenPoints(from_coord->dropOffX, from_coord->dropOffY, to_coord->pickupX, to_coord->pickupY);
            _distance_matrix[from_load][to_load] = switch_from_to + to_load_pickup_dropoff_distance;
        }
    }
}

std::vector<std::vector<size_t>> Graph::plan_paths(const Probs& probs) {
    std::unordered_set<size_t> loads;
    for (size_t ii = 1; ii < _coordinates.size(); ++ii) {
        loads.insert(ii);
    }

    std::vector<std::vector<size_t>> solution;
    while (!loads.empty()) {
        std::vector<size_t> path = plan_path_for_driver(loads, probs);
        if (path.empty()) {
            // Should not happen, indicates a problem...
            return std::vector<std::vector<size_t>>();
        }
        // add path to solution
        solution.push_back(path);
        // remove everything in path from loads
        for (size_t item : path) {
            loads.erase(item);
        }
    }
    return solution;
}

std::vector<size_t> Graph::plan_path_for_driver(const std::unordered_set<size_t>& candidate_loads, const Probs& probs) {
    std::unordered_set<size_t> loads = candidate_loads;
    size_t current_load = 0; // we start at HQ
    std::vector<size_t> fallback;  // this is always a solution that gets us back within the _max_minutes
    long double fallback_minutes = 0;

    std::vector<size_t> cumulative;  // this is always a solution that gets us back within the _max_minutes
    long double cumulative_minutes = 0;

    while (cumulative_minutes < _max_minutes) {
        std::vector<long double>* current_distances = &_distance_matrix[current_load];
        std::unordered_set<size_t> reachable_loads;
        // only consider neighbors where cumulative_minutes + current_distances->at(load) < _distance_matrix
        // if reachable_loads is empty, consider going home, unless it exceeds the distance limit, then use the fallback solution and we're done!
        // if current node can travel to HQ without exceeding max minutes, then update fallback and fallback_minutes (former is cumulative, latter is cumulative_minutes + current_distances->at(0))
        // select a scheme, ie Scheme scheme = probs.select_scheme(_r, at_home);
        // if scheme cannot be chosen at all, use the fallback and we're done, otherwise
        // pick a next_load via probs.implement_scheme(_r, loads);
        // update cumulative and cumulative_minutes, cumulative appends next_load, and cumulative_minutes += current_distances->at(next_load)
        // update current_load to be next_load & remove current_load from loads
        // if current_load just got set to zero, check the cumulative_minutes, if it's under _max_minutes, then use cumulative and we're done
    }

    // if we got here, use the fallback
}
