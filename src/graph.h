#pragma once

#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>

struct Coordinate {
    long double pickupX;
    long double pickupY;
    long double dropOffX;
    long double dropOffY;

    Coordinate()
    : pickupX(0)
    , pickupY(0)
    , dropOffX(0)
    , dropOffY(0) {}
};

enum class Scheme {
    Unknown,
    Home,
    GreedyNearest,
    OnwayNearest,
    WeightedNearest,
    Random,
};

struct Probs {
    int probHome;
    int probGreedyNearest;
    int probOnwayNearest;
    int probWeightedNearest;
    int probRandom;

    Probs(int h, int g, int o, int w, int r)
    : probHome(h)
    , probGreedyNearest(g)
    , probOnwayNearest(o)
    , probWeightedNearest(w)
    , probRandom(r) {}
};

class Graph {
public:
    Graph(const std::vector<std::string>& lines, std::ofstream* log, long double max_minutes)
    : _lines(lines)
    , _log(log)
    , _max_minutes(max_minutes) {}

    void build();

    void debug();

    std::vector<std::vector<size_t>> plan_paths(const Probs& probs);
    std::vector<size_t> plan_path_for_driver(const std::unordered_set<size_t>& loads, const Probs& probs);

    size_t numCoordinates() const {
        return _coordinates.size();
    }

    std::vector<Coordinate> getCoordinates() const {
        return _coordinates;
    }

private:
    void build_coordinates();
    void build_distance_matrix();

    std::vector<std::string> _lines;
    std::ofstream* _log;
    long double _max_minutes;

    std::vector<Coordinate> _coordinates;
    std::vector<std::vector<long double>> _distance_matrix;
};
