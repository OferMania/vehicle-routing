#include "scheme.h"

#include <limits>

void Probs::init_goalposts() {
    _regular_goalposts.push_back(0);
    int sum = 0;
    int nohq_sum = 0;

    // HQ
    sum += probHq;
    _regular_goalposts.push_back(sum);

    // Greedy nearest
    sum += probGreedyNearest;
    nohq_sum += probGreedyNearest;
    _regular_goalposts.push_back(sum);
    _nonhq_goalposts.push_back(nohq_sum);

    // Onway nearest
    sum += probOnwayNearest;
    nohq_sum += probOnwayNearest;
    _regular_goalposts.push_back(sum);
    _nonhq_goalposts.push_back(nohq_sum);

    // Greedy nearest
    sum += probWeightedNearest;
    nohq_sum += probWeightedNearest;
    _regular_goalposts.push_back(sum);
    _nonhq_goalposts.push_back(nohq_sum);

    // Greedy nearest
    sum += probRandom;
    nohq_sum += probRandom;
    _regular_goalposts.push_back(sum);
    _nonhq_goalposts.push_back(nohq_sum);

    _regular_sum = sum;
    _nonhq_sum = nohq_sum;
}

Scheme Probs::select_scheme(bool at_hq) {
    if (!generator) {
        // shouldn't happen, but if generator is null, scheme is unknown
        return Scheme::Unknown;
    }

    if (at_hq) {
        std::uniform_int_distribution<int> distribution(1, _nonhq_sum);
        int random_num = distribution(*generator);
        size_t chosen_index = _nonhq_goalposts.size();  // default to an invalid choice
        for (size_t ii = 0; ii < _nonhq_goalposts.size(); ++ii) {
            if (random_num <= _nonhq_goalposts[ii]) {
                chosen_index = ii;
                break;
            }
        }
        if (chosen_index == _nonhq_goalposts.size()) {
            // We cannot stay at HQ, so just do the greedy nearest
            return Scheme::GreedyNearest;
        }
        return static_cast<Scheme>(chosen_index+2);
    } else {
        std::uniform_int_distribution<int> distribution(1, _regular_sum);
        int random_num = distribution(*generator);
        size_t chosen_index = _regular_goalposts.size();  // default to an invalid choice
        for (size_t ii = 0; ii < _regular_goalposts.size(); ++ii) {
            if (random_num <= _regular_goalposts[ii]) {
                chosen_index = ii;
                break;
            }
        }
        if (chosen_index == _regular_goalposts.size()) {
            // shouldn't happen, means out random number is outside the goalposts
            return Scheme::Unknown;
        }
        return static_cast<Scheme>(chosen_index+1);
    }
}

size_t Probs::implement_scheme_and_select_next_load(Scheme scheme, const std::unordered_set<size_t>& reachable_loads, const std::vector<long double>* current_distances, const std::vector<long double>* hq_distances) {
    switch (scheme) {
        case Scheme::Home:
        {
            return select_hq();
        }
        case Scheme::GreedyNearest:
        {
            return select_nearest(reachable_loads, current_distances);
        }
        case Scheme::OnwayNearest:
        {
            return select_onway_nearest(reachable_loads, current_distances, hq_distances);
        }
        case Scheme::WeightedNearest:
        {
            return select_weighted_nearest(reachable_loads, current_distances);
        }
        case Scheme::Random:
        {
            return select_random(reachable_loads);
        }
        case Scheme::Unknown:
        default:
        {
            // shouldn't happen, but if it does, we go to HQ
            return 0;
        }
    }
}

size_t Probs::select_hq() {
    return 0;
}

size_t Probs::select_nearest(const std::unordered_set<size_t>& reachable_loads, const std::vector<long double>* current_distances) {
    if (reachable_loads.empty()) {
        // If there's no reachable loads, go to HQ
        return 0;
    } else if (reachable_loads.size() == 1) {
        // If there's only 1 reachable load, that's the answer
        auto it = reachable_loads.cbegin();
        return *it;
    }
    size_t best_load_id = 0;
    long double nearest_distance = std::numeric_limits<long double>::infinity();
    for (size_t load_id : reachable_loads) {
        if (current_distances->at(load_id) < nearest_distance) {
            best_load_id = load_id;
            nearest_distance = current_distances->at(load_id);
        }
    }
    return best_load_id;
}

size_t Probs::select_onway_nearest(const std::unordered_set<size_t>& reachable_loads, const std::vector<long double>* current_distances, const std::vector<long double>* hq_distances) {
    if (reachable_loads.empty()) {
        // If there's no reachable loads, go to HQ
        return 0;
    } else if (reachable_loads.size() == 1) {
        // If there's only 1 reachable load, that's the answer
        auto it = reachable_loads.cbegin();
        return *it;
    }

    // Onway nearest is a node that's closer to us than HQ. If all nodes are closer to HQ, we fallback to regular nearest
    size_t best_load_id = 0;
    long double nearest_distance = std::numeric_limits<long double>::infinity();
    for (size_t load_id : reachable_loads) {
        if ((current_distances->at(load_id) < hq_distances->at(load_id)) && (current_distances->at(load_id) < nearest_distance)) {
            best_load_id = load_id;
            nearest_distance = current_distances->at(load_id);
        }
    }
    // We want to save the trouble of making a new driver if possible, by picking something closer to us than hq, but if that cannot be done, just pick the closest to us overall instead
    if (best_load_id == 0) {
        return select_nearest(reachable_loads, current_distances);
    }

    return best_load_id;
}

size_t Probs::select_weighted_nearest(const std::unordered_set<size_t>& reachable_loads, const std::vector<long double>* current_distances) {
    if (!generator) {
        return 0;
    }
    if (reachable_loads.empty()) {
        // If there's no reachable loads, go to HQ
        return 0;
    } else if (reachable_loads.size() == 1) {
        // If there's only 1 reachable load, that's the answer
        auto it = reachable_loads.cbegin();
        return *it;
    }

    std::vector<size_t> random_access_loads(reachable_loads.cbegin(), reachable_loads.cend());
    // get the sum
    long double sum = 0;
    for (size_t load_id : random_access_loads) {
        sum += current_distances->at(load_id);
    }
    // find the weights, lower distances means higher weights
    std::vector<long double> weights;
    for (size_t load_id : random_access_loads) {
        weights.push_back(sum - current_distances->at(load_id));
    }
    // sum of the weights is sum * n-1, where n is number of elements. This can be proven mathematically
    long double weights_sum = sum * (random_access_loads.size() - 1);
    long double sum_so_far = 0;
    std::vector<long double> goalposts;
    for (size_t index = 0; index < weights.size(); ++index) {
        sum_so_far += weights[index];
        goalposts.push_back(sum_so_far);
    }

    std::uniform_real_distribution<long double> distribution(0, weights_sum);
    long double choice = distribution(*generator);
    size_t chosen_index = goalposts.size();
    for (size_t index = 0; index < goalposts.size(); ++index) {
        if (choice < goalposts[index]) {
            chosen_index = index;
            break;
        }
    }

    if (chosen_index == goalposts.size()) {
        // shouldn't happen
        return 0;
    }

    return random_access_loads[chosen_index];
}

size_t Probs::select_random(const std::unordered_set<size_t>& reachable_loads) {
    if (!generator) {
        return 0;
    }
    if (reachable_loads.empty()) {
        // If there's no reachable loads, go to HQ
        return 0;
    } else if (reachable_loads.size() == 1) {
        // If there's only 1 reachable load, that's the answer
        auto it = reachable_loads.cbegin();
        return *it;
    }

    std::vector<size_t> random_access_loads(reachable_loads.cbegin(), reachable_loads.cend());
    std::uniform_int_distribution<size_t> distribution(0, random_access_loads.size() - 1);
    size_t index = distribution(*generator);
    return random_access_loads[index];
}

std::string Probs::to_string() {
    std::string result = "";
    result += std::to_string(probHq);
    result += ",";
    result += std::to_string(probGreedyNearest);
    result += ",";
    result += std::to_string(probOnwayNearest);
    result += ",";
    result += std::to_string(probWeightedNearest);
    result += ",";
    result += std::to_string(probRandom);
    result += ",";
    return result;
}
