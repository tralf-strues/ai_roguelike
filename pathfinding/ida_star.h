#pragma once

#include <cassert>
#include <limits>
#include <optional>
#include <vector>
#include <functional>

namespace detail {

struct SearchResult {
    bool found;
    float t;
};

template <class Node, class HeuristicFunc, class CostFunc, class SuccessorsFunc>
SearchResult Search(const HeuristicFunc &h, const CostFunc &cost,
                    const SuccessorsFunc &successors, std::vector<Node> &path,
                    const Node &to, float g, float bound) {
    Node& node = path.back();

    float f = g + h(node, to);
    if (f > bound) {
        return SearchResult{false, f};
    }

    if (node == to) {
        return SearchResult{true, f};
    }

    float min = std::numeric_limits<float>::infinity();
    for (const auto& successor : successors(node)) {
        if (std::find(path.begin(), path.end(), successor) == path.end()) {
            path.push_back(successor);

            auto result = Search(h, cost, successors, path, to, g + cost(node, successor), bound);
            if (result.found) {
                return result;
            }

            if (result.t < min) {
                min = result.t;
            }

            path.pop_back();
        }
    }

    return SearchResult{false, min};
}

} // namespace detail

template <class Node, class HeuristicFunc, class CostFunc, class SuccessorsFunc>
std::optional<float> IdaStar(const HeuristicFunc &h, const CostFunc &cost,
                             const SuccessorsFunc &successors, const Node &from,
                             const Node &to, std::vector<Node> &path) {
    float bound = h(from, to);
    path.push_back(from);

    while (true) {
        auto result = detail::Search(h, cost, successors, path, to, 0, bound);

        if (result.found) {
            return result.t;
        }

        if (std::isinf(result.t)) {
            return std::nullopt;
        }

        bound = result.t;
    };

    assert(false);
    return std::nullopt;
}
