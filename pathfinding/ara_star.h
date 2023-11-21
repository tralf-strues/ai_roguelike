#pragma once

#include <cassert>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <vector>

template <class Node, class NodeHash>
class AraStar {
 public:
  class NodeCompare {
   public:
    using size_type = std::size_t;
    using value_type = Node;
    using reference = value_type&;
    using const_reference = const value_type&;
    using pointer = value_type*;
    using const_pointer = const value_type*;

   public:
    NodeCompare(AraStar &alg) : alg_(alg) {}

    bool operator()(const Node &lhs, const Node &rhs) const {
      return alg_.GetF(lhs, lhs) > alg_.GetF(rhs, lhs);
    }

   private:
    AraStar &alg_;
  };

 public:
  static constexpr float kInfinity = std::numeric_limits<float>::max();

  using IndexFunc      = std::function<size_t(const Node& node)>;
  using HeuristicFunc  = std::function<float(const Node& from, const Node& goal)>;
  using CostFunc       = std::function<float(const Node& from, const Node& to)>;
  using SuccessorsFunc = std::function<std::vector<Node>(const Node& node)>;

  using Queue          = std::priority_queue<Node, std::vector<Node>, NodeCompare>;

 public:
   AraStar(const IndexFunc &index, const HeuristicFunc &heuristic,
           const CostFunc &cost, const SuccessorsFunc &successors)
       : index_(index), heuristic_(heuristic), cost_(cost),
         successors_(successors), open_(NodeCompare{*this}),
         visited_(NodeCompare{*this}) {}

   void FindPath(size_t nodes_count, const Node &from, const Node &to,
                 float epsilon) {
    epsilon_ = epsilon;
    Reset(nodes_count);
    SetG(from, 0.0f);

    open_.push(from);
    visited_.insert(from);
    ImprovePath(to);

    float cur_epsilon = CurrentEpsilon(to);
    while (cur_epsilon > 1.0f) {
      DecreaseEpsilon();

      for (const auto &node : incons_) {
        open_.push(node);
      }

      closed_.clear();

      ImprovePath(to);

      cur_epsilon = CurrentEpsilon(to);
    }
  }

  float GetF(const Node& node, const Node& to) const {
    return GetG(node) + epsilon_ * heuristic_(node, to);
  }

  float GetG(const Node& node) const {
    return g_[index_(node)];
  }

  const std::set<Node, NodeCompare>& GetVisited() const {
    return visited_;
  }

  std::vector<Node> ReconstructPath(const Node& from, const Node& to) const {
    std::vector<Node> path;

    Node current = to;
    while (current != from) {
      path.push_back(current);

      if (!came_from_.contains(current)) {
        assert(false);
        return {};
      }

      current = came_from_.at(current);
    }

    path.push_back(from);
    std::reverse(path.begin(), path.end());

    return path;
  }

 private:
  void SetG(const Node& node, float value) {
    g_[index_(node)] = value;
  }

  void Reset(size_t nodes_count) {
    assert(nodes_count > 0);
    nodes_count_ = nodes_count;
    
    g_.resize(nodes_count_);
    for (auto& g : g_) {
      g = kInfinity;
    }

    while (!open_.empty()) {
      open_.pop();
    }

    closed_.clear();
    incons_.clear();
    came_from_.clear();
    visited_.clear();
  }

  void ImprovePath(const Node& to) {
    while (!open_.empty() && GetF(to, to) > GetF(open_.top(), to)) {
      auto s = open_.top();
      open_.pop();

      closed_.insert(s);

      for (const auto& successor : successors_(s)) {
        visited_.insert(successor);

        if (GetG(successor) > GetG(s) + cost_(s, successor)) {
          SetG(successor, GetG(s) + cost_(s, successor));
          came_from_[successor] = s;

          if (!closed_.contains(successor)) {
            open_.push(successor);
          } else {
            incons_.push_back(successor);
          }
        }
      }
    }
  }

  void DecreaseEpsilon() {
    epsilon_ -= 0.1f;
  }

  float CurrentEpsilon(const Node& to) {
    float min_incons = kInfinity;
    for (const auto& node : incons_) {
      if (GetF(node, to) < GetF(to, to)) {
        min_incons = GetF(node, to);
      }
    }

    const float denom = std::min(GetF(open_.top(), to), min_incons);
    return std::min(epsilon_, GetG(to) / denom);
  }

 private:
  IndexFunc      index_;
  HeuristicFunc  heuristic_;
  CostFunc       cost_;
  SuccessorsFunc successors_;

  size_t nodes_count_{0};
  float epsilon_{2.0f};       

  std::vector<float> g_;
  Queue open_;
  std::set<Node> closed_;
  std::vector<Node> incons_;

  std::unordered_map<Node, Node, NodeHash> came_from_;
  std::set<Node, NodeCompare> visited_;
};