#pragma once

#include <fstream>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include "coordinate.h"
#include "scheme.h"

class Graph {
public:
    Graph(const std::vector<std::string>& lines, std::ofstream* log, long double max_minutes)
    : _lines(lines)
    , _log(log)
    , _max_minutes(max_minutes) {}

    void build();

    void debug();

    std::vector<std::vector<size_t>> plan_paths(Probs& probs);

    size_t numCoordinates() const {
        return _coordinates.size();
    }

    std::vector<Coordinate> getCoordinates() const {
        return _coordinates;
    }

private:
    void build_coordinates();
    void build_distance_matrix();

    std::vector<size_t> plan_path_for_driver(const std::unordered_set<size_t>& loads, Probs& probs);

    std::vector<std::string> _lines;
    std::ofstream* _log;
    long double _max_minutes;

    std::vector<Coordinate> _coordinates;
    std::vector<std::vector<long double>> _distance_matrix;
};
