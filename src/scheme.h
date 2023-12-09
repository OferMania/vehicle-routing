#pragma once

#include <cstddef>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

enum class Scheme {
    Unknown,
    Home,
    GreedyNearest,
    OnwayNearest,
    WeightedNearest,
    Random,
};

class Probs {
public:
    Probs(std::mt19937* gen, int h, int g, int o, int w, int r)
    : generator(gen)
    , probHq(h)
    , probGreedyNearest(g)
    , probOnwayNearest(o)
    , probWeightedNearest(w)
    , probRandom(r) {
        init_goalposts();
    }

    // select a scheme. Note that we ignore probHq if we're already at HQ, which only happens at the start. Otherwise, it is considered.
    Scheme select_scheme(bool at_hq);

    // implements scheme scheme and returns the load to do next, which is either one of the items in reachable_loads or zero (in the event we chose to deliberately return to HQ)
    size_t implement_scheme_and_select_next_load(Scheme scheme, const std::unordered_set<size_t>& reachable_loads, const std::vector<long double>* current_distances, const std::vector<long double>* hq_distances);

    std::string to_string();

private:
    std::mt19937* generator;
    int probHq;
    int probGreedyNearest;
    int probOnwayNearest;
    int probWeightedNearest;
    int probRandom;

    std::vector<int> _regular_goalposts;
    std::vector<int> _nonhq_goalposts;
    int _regular_sum;
    int _nonhq_sum;

    void init_goalposts();
    size_t select_hq();
    size_t select_nearest(const std::unordered_set<size_t>& reachable_loads, const std::vector<long double>* current_distances);
    size_t select_onway_nearest(const std::unordered_set<size_t>& reachable_loads, const std::vector<long double>* current_distances, const std::vector<long double>* hq_distances);
    size_t select_weighted_nearest(const std::unordered_set<size_t>& reachable_loads, const std::vector<long double>* current_distances);
    size_t select_random(const std::unordered_set<size_t>& reachable_loads);
};
