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
        if (line.empty()) {
            continue;
        }
        std::stringstream ss(line);
        std::vector<std::string> args;
        std::string piece;

        // split the line by space to build args
        while (std::getline(ss, piece, ' ')) {
            args.push_back(piece);
        }

        if (args.size() < 3) {
            // We need at least 3 args for this to be a valid route
#if LOGGING
            *_log << "We need at least 3 args on this line: " << line << std::endl;
#endif
            continue;
        }

        if (args[1].size() < 3 || args[2].size() < 3) {
            // because args[1] and args[2] are expected to be in format (x,y), we expect each of them to have at least 3 characters
#if LOGGING
            *_log << "Malformed coordinate format on line: " << line << std::endl;
#endif
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
#if LOGGING
            *_log << "Malformed dimensions on line: " << line << std::endl;
#endif
            continue;
        }

        size_t index = static_cast<size_t>(std::stoi(args[0]));
        // Sigh, index is typically equivalent to ii, but we try not to take any chances
        if (index < 0 || index >= _lines.size()) {
#if LOGGING
            *_log << "Invalid index " << index << " found on line: " << line << std::endl;
#endif
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

std::vector<std::vector<size_t>> Graph::plan_paths(Probs& probs) {
#if LOGGING
    *_log << "Running plan_paths with: " << probs.to_string() << std::endl;
#endif

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

std::vector<size_t> Graph::plan_path_for_driver(const std::unordered_set<size_t>& candidate_loads, Probs& probs) {
#if LOGGING
    *_log << "Running plan_path_for_driver with loads: ";
    for (size_t load_id : candidate_loads) {
        *_log << load_id << ", ";
    }
    *_log << std::endl;
#endif

    std::unordered_set<size_t> loads = candidate_loads;
    const std::vector<long double>* hq_distances = &_distance_matrix[0];
    size_t current_load = 0; // we start at HQ
    std::vector<size_t> fallback;  // this is always a solution that gets us back within the _max_minutes
    long double fallback_minutes __attribute__((unused)) = 0;  // variable may be unread, but we still want to track it - corresponds to fallback

    std::vector<size_t> cumulative;  // where we want to explore so far if possible. Note we might not be able to return to HQ & only detect if AFTER visiting a node
    long double cumulative_minutes = 0; // minutes for the solution

    while (cumulative_minutes < _max_minutes) {
#if LOGGING
        *_log << "Current load: " << current_load << std::endl;
        *_log << "Cumulative minutes: " << cumulative_minutes << std::endl;
#endif

        const std::vector<long double>* current_distances = &_distance_matrix[current_load];

        // Check if we can return to HQ from where we're at. If yes, that's the new fallback
        // solution in case we later make a cumulative that cannot proceed (due to returning
        // to HQ exceeding max_minutes)
        bool canReturnToHq = (cumulative_minutes + current_distances->at(0) < _max_minutes);
        if (canReturnToHq) {
            fallback = cumulative;
            fallback_minutes = cumulative_minutes + current_distances->at(0);

#if LOGGING
            *_log << "Can return to hq from current_node, updating fallback" << std::endl;
            *_log << "fallback_minutes = " << fallback_minutes << std::endl;
#endif
        }

        // For the loadId's in loads, none of which are HQ, and NONE of which match current_load, only consider loadId's we can reach from the current_load without exceeding max_minutes
        std::unordered_set<size_t> reachable_loads;
        for (size_t loadId : loads) {
            if ((loadId != current_load) && cumulative_minutes + current_distances->at(loadId) < _max_minutes) {
                reachable_loads.insert(loadId);
            }
        }

        // Do we have any new reachable_loads to take on? If not, use the fallback.
        if (reachable_loads.empty()) {
#if LOGGING
            *_log << "No more reachable loads, using fallback" << std::endl;
#endif
            return fallback;
        }

        // select a scheme, note the if current_load is 0 (aka HQ), then we avoid returning home, aka probHome is ignored. Otherwise probHome is considered.
        Scheme scheme = probs.select_scheme(/* at_hq = */ (current_load == 0));

#if LOGGING
        *_log << "Scheme chosen is: " << static_cast<int>(scheme) << std::endl;
#endif

        // This shouldn't happen, but just in case... If a scheme cannot be chosen, ie it's unknown, then use the fallback and we're done
        if (scheme == Scheme::Unknown) {
#if LOGGING
            *_log << "Unknown scheme chosen, using fallback" << std::endl;
#endif
            return fallback;
        }

        // pick a next_load to visit
        size_t next_load = probs.implement_scheme_and_select_next_load(scheme, reachable_loads, current_distances, hq_distances);

#if LOGGING
        *_log << "Next load has been chosen to be: " << next_load << std::endl;
#endif

        // next_load got chosen, update cumulative, cumulative_minutes, loads, and current_load
        if (next_load != 0) {
            // We have a new load to consider for the driver's path, update the cumulative stats, then update loads and current_load
            cumulative.push_back(next_load);
            cumulative_minutes += current_distances->at(next_load);

            loads.erase(next_load);
            current_load = next_load;
        } else {
            // We got instructed to go to HQ after visiting a series of non-HQ nodes. Don't update cumulative (it's implied) or loads (which doesn't have zero), but do update the cumulative_minutes and current_load. This means the path for the driver is done
            cumulative_minutes += current_distances->at(next_load);
            current_load = next_load;
            break;
        }
    }

    // if we got here, it might've been deliberate, so check if we can return to HQ from where we're at without exceeding max_minutes
    // (if we're already at HQ, then _distance_matrix[current_load][0] is zero). If we CAN return to HQ, that's our new fallback, otherwise,
    // keep existing fallback path
    if ((cumulative_minutes < _max_minutes) && (cumulative_minutes + _distance_matrix[current_load][0] < _max_minutes)) {
        fallback = cumulative;
        fallback_minutes = cumulative_minutes + _distance_matrix[current_load][0];

#if LOGGING
        *_log << "Done iterating, but cumulative minutes plus distance to HQ doesn't exceed max minutes, so updating fallback" << std::endl;
        *_log << "fallback_minutes = " << fallback_minutes << std::endl;
#endif
    }

    // If we reach here, we found a longest possible fallback path for the driver without exceeding _max_minutes
    // Note our paths generally favor utilizing existing drivers as far as we can, but high probHq gives us some leeway for scenarios where
    // the distance between loads is high enough to consider favoring more drivers.
#if LOGGING
    *_log << "Iterated as far as we can go w this driver, outputting the fallback path that's within max minutes" << std::endl;
#endif
    return fallback;
}
