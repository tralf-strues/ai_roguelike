#include "aiLibrary.h"
#include "ecsTypes.h"
#include "aiUtils.h"
#include "math.h"
#include "raylib.h"
#include "blackboard.h"
#include <algorithm>
#include <random>
#include <iostream>

struct CompoundNode : public BehNode
{
  std::vector<BehNode*> nodes;

  virtual ~CompoundNode()
  {
    for (BehNode *node : nodes)
      delete node;
    nodes.clear();
  }

  CompoundNode &pushNode(BehNode *node)
  {
    nodes.push_back(node);
    return *this;
  }
};

struct Sequence : public CompoundNode
{
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    for (BehNode *node : nodes)
    {
      BehResult res = node->update(ecs, entity, bb);
      if (res != BEH_SUCCESS)
        return res;
    }
    return BEH_SUCCESS;
  }
};

struct Selector : public CompoundNode
{
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    for (BehNode *node : nodes)
    {
      BehResult res = node->update(ecs, entity, bb);
      if (res != BEH_FAIL)
        return res;
    }
    return BEH_FAIL;
  }
};

struct UtilitySelector : public BehNode
{
  std::vector<std::pair<BehNode*, utility_function>> utilityNodes;

  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    std::vector<std::pair<float, size_t>> utilityScores;
    for (size_t i = 0; i < utilityNodes.size(); ++i)
    {
      const float utilityScore = utilityNodes[i].second(bb);
      utilityScores.push_back(std::make_pair(utilityScore, i));
    }
    std::sort(utilityScores.begin(), utilityScores.end(), [](auto &lhs, auto &rhs)
    {
      return lhs.first > rhs.first;
    });
    for (const std::pair<float, size_t> &node : utilityScores)
    {
      size_t nodeIdx = node.second;
      BehResult res = utilityNodes[nodeIdx].first->update(ecs, entity, bb);
      if (res != BEH_FAIL) {
        // std::cout << "UtilitySelector: Decided on " << node.second << std::endl;
        return res;
      }
    }
    return BEH_FAIL;
  }
};

template <class D, class W>
void WeightedShuffle(D first, D last, W first_weight, W last_weight) {
  static std::default_random_engine g;

  while (first != last && first_weight != last_weight) {
    std::discrete_distribution dd(first_weight, last_weight);
    
    auto i = dd(g);
    if (i) {
      std::iter_swap(first, std::next(first, i));
      std::iter_swap(first_weight, std::next(first_weight, i));
    }
    
    ++first;
    ++first_weight;
  }
}

struct UtilitySelectorWeightedRandom : public UtilitySelector
{
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    std::vector<float> utilityScores;
    std::vector<BehNode*> contributingNodes;

    for (size_t i = 0; i < utilityNodes.size(); ++i) {
      const float utilityScore = utilityNodes[i].second(bb);

      if (utilityScore > 0.f) {
        utilityScores.push_back(utilityScore);
      }
    }

    WeightedShuffle(contributingNodes.begin(), contributingNodes.end(),
                    utilityScores.begin(), utilityScores.end());

    for (BehNode* node : contributingNodes) {
      BehResult res = node->update(ecs, entity, bb);

      if (res != BEH_FAIL) {
        return res;
      }
    }
    return BEH_FAIL;
  }
};

struct UtilitySelectorInertial : public UtilitySelector {
  UtilitySelectorInertial(float inertiaAmount, float damp)
      : inertiaAmount(inertiaAmount), damp(damp) {}

  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override {
    std::vector<std::pair<float, size_t>> utilityScores;

    for (size_t i = 0; i < utilityNodes.size(); ++i) {
      const float utilityScore = utilityNodes[i].second(bb) + perNodeInertia[i];
      utilityScores.push_back(std::make_pair(utilityScore, i));
    }

    // std::cout << "UtilitySelectorInertial: scores [";
    // for (const auto& utility : utilityScores) {
    //   std::cout << utility.first << ", ";
    // }
    // std::cout << "]\n";

    // std::cout << "UtilitySelectorInertial: inertias [";
    // for (float inertia : perNodeInertia) {
    //   std::cout << inertia << ", ";
    // }
    // std::cout << "]\n";

    std::sort(utilityScores.begin(), utilityScores.end(), [](auto &lhs, auto &rhs) {
      return lhs.first > rhs.first;
    });

    for (const std::pair<float, size_t> &node : utilityScores) {
      size_t nodeIdx = node.second;

      BehResult res = utilityNodes[nodeIdx].first->update(ecs, entity, bb);
      if (res != BEH_FAIL) {
        // std::cout << "UtilitySelectorInertial: Decided on " << node.second << std::endl;

        // Update inertia
        if (nodeIdx != prevNode) {
          perNodeInertia[nodeIdx] += inertiaAmount;
        }

        for (size_t i = 0; i < perNodeInertia.size(); ++i) {
          if (i != nodeIdx) {
            perNodeInertia[i] = std::max(0.f, perNodeInertia[i] - damp);
          }
        }

        prevNode = nodeIdx;

        return res;
      }
    }

    return BEH_FAIL;
  }

  std::vector<float> perNodeInertia;
  float inertiaAmount;
  float damp;

 private:
  size_t prevNode = -1U;
};

struct MoveToEntity : public BehNode
{
  size_t entityBb = size_t(-1); // wraps to 0xff...
  MoveToEntity(flecs::entity entity, const char *bb_name)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a, const Position &pos)
    {
      flecs::entity targetEntity = bb.get<flecs::entity>(entityBb);
      if (!targetEntity.is_alive())
      {
        res = BEH_FAIL;
        return;
      }
      targetEntity.get([&](const Position &target_pos)
      {
        if (pos != target_pos)
        {
          a.action = move_towards(pos, target_pos);
          res = BEH_RUNNING;
        }
        else
          res = BEH_SUCCESS;
      });
    });
    return res;
  }
};

struct IsLowHp : public BehNode
{
  float threshold = 0.f;
  IsLowHp(float thres) : threshold(thres) {}

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &) override
  {
    BehResult res = BEH_SUCCESS;
    entity.get([&](const Hitpoints &hp)
    {
      res = hp.hitpoints < threshold ? BEH_SUCCESS : BEH_FAIL;
    });
    return res;
  }
};

struct FindEnemy : public BehNode
{
  size_t entityBb = size_t(-1);
  float distance = 0;
  FindEnemy(flecs::entity entity, float in_dist, const char *bb_name) : distance(in_dist)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }
  BehResult update(flecs::world &ecs, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_FAIL;
    static auto enemiesQuery = ecs.query<const Position, const Team>();
    entity.set([&](const Position &pos, const Team &t)
    {
      flecs::entity closestEnemy;
      float closestDist = FLT_MAX;
      Position closestPos;
      enemiesQuery.each([&](flecs::entity enemy, const Position &epos, const Team &et)
      {
        if (t.team == et.team)
          return;
        float curDist = dist(epos, pos);
        if (curDist < closestDist)
        {
          closestDist = curDist;
          closestPos = epos;
          closestEnemy = enemy;
        }
      });
      if (ecs.is_valid(closestEnemy) && closestDist <= distance)
      {
        bb.set<flecs::entity>(entityBb, closestEnemy);
        res = BEH_SUCCESS;
      }
    });
    return res;
  }
};

struct Flee : public BehNode
{
  size_t entityBb = size_t(-1);
  Flee(flecs::entity entity, const char *bb_name)
  {
    entityBb = reg_entity_blackboard_var<flecs::entity>(entity, bb_name);
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a, const Position &pos)
    {
      flecs::entity targetEntity = bb.get<flecs::entity>(entityBb);
      if (!targetEntity.is_alive())
      {
        res = BEH_FAIL;
        return;
      }
      targetEntity.get([&](const Position &target_pos)
      {
        a.action = inverse_move(move_towards(pos, target_pos));
      });
    });
    return res;
  }
};

struct Patrol : public BehNode
{
  size_t pposBb = size_t(-1);
  float patrolDist = 1.f;
  Patrol(flecs::entity entity, float patrol_dist, const char *bb_name)
    : patrolDist(patrol_dist)
  {
    pposBb = reg_entity_blackboard_var<Position>(entity, bb_name);
    entity.set([&](Blackboard &bb, const Position &pos)
    {
      bb.set<Position>(pposBb, pos);
    });
  }

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a, const Position &pos)
    {
      Position patrolPos = bb.get<Position>(pposBb);
      if (dist(pos, patrolPos) > patrolDist)
      {
        a.action = move_towards(pos, patrolPos);
      }
      else
      {
        a.action = GetRandomValue(EA_MOVE_START, EA_MOVE_END - 1); // do a random walk
      }
    });
    return res;
  }
};

struct PatchUp : public BehNode
{
  float hpThreshold = 100.f;
  PatchUp(float threshold) : hpThreshold(threshold) {}

  BehResult update(flecs::world &, flecs::entity entity, Blackboard &) override
  {
    BehResult res = BEH_SUCCESS;
    entity.set([&](Action &a, Hitpoints &hp)
    {
      if (hp.hitpoints >= hpThreshold)
        return;
      res = BEH_RUNNING;
      a.action = EA_HEAL_SELF;
    });
    return res;
  }
};

Actions random_move_weighted(Actions prev)
{
  int weights[EA_MOVE_END - EA_MOVE_START];

  constexpr int L_IDX = EA_MOVE_LEFT  - EA_MOVE_START;
  constexpr int R_IDX = EA_MOVE_RIGHT - EA_MOVE_START;
  constexpr int D_IDX = EA_MOVE_DOWN  - EA_MOVE_START;
  constexpr int U_IDX = EA_MOVE_UP    - EA_MOVE_START;

  switch (prev) {
    case EA_MOVE_LEFT: {
      weights[L_IDX] = 75;
      weights[R_IDX] = 5;
      weights[D_IDX] = 10;
      weights[U_IDX] = 10;
      break;
    }
    case EA_MOVE_RIGHT: {
      weights[L_IDX] = 5;
      weights[R_IDX] = 75;
      weights[D_IDX] = 10;
      weights[U_IDX] = 10;
      break;
    }
    case EA_MOVE_DOWN: {
      weights[L_IDX] = 10;
      weights[R_IDX] = 10;
      weights[D_IDX] = 75;
      weights[U_IDX] = 5;
      break;
    }
    case EA_MOVE_UP: {
      weights[L_IDX] = 10;
      weights[R_IDX] = 10;
      weights[D_IDX] = 5;
      weights[U_IDX] = 75;
      break;
    }
    default: {
      weights[L_IDX] = 20;
      weights[R_IDX] = 20;
      weights[D_IDX] = 20;
      weights[U_IDX] = 20;
    }
  };

  int sum = 0;
  for(int i = 0; i < 4; i++) {
   sum += weights[i];
  }

  int rnd = GetRandomValue(0, sum - 1);
  for (int i = 0; i < 4; i++) {
    if (rnd < weights[i]) {
      return static_cast<Actions>(i + EA_MOVE_START);
    }

    rnd -= weights[i];
  }

  return EA_MOVE_LEFT;
};

struct RandomExplore : public BehNode
{
  BehResult update(flecs::world &, flecs::entity entity, Blackboard &) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a)
    {
      a.action = random_move_weighted(static_cast<Actions>(a.action));
      // std::cout << "Decided to move " << a.action << std::endl;
    });
    return res;
  }
};

struct ReturnToBase : public BehNode
{
  BehResult update(flecs::world &, flecs::entity entity, Blackboard &bb) override
  {
    BehResult res = BEH_RUNNING;
    entity.set([&](Action &a, const Position &pos)
    {
      a.action = move_towards(pos, bb.get<Position>("base"));
    });
    return res;
  }
};

BehNode *sequence(const std::vector<BehNode*> &nodes)
{
  Sequence *seq = new Sequence;
  for (BehNode *node : nodes)
    seq->pushNode(node);
  return seq;
}

BehNode *selector(const std::vector<BehNode*> &nodes)
{
  Selector *sel = new Selector;
  for (BehNode *node : nodes)
    sel->pushNode(node);
  return sel;
}

BehNode *utility_selector(const std::vector<std::pair<BehNode*, utility_function>> &nodes)
{
  UtilitySelector *usel = new UtilitySelector;
  usel->utilityNodes = std::move(nodes);
  return usel;
}

BehNode *utility_selector_weighted_random(const std::vector<std::pair<BehNode*, utility_function>> &nodes)
{
  UtilitySelectorWeightedRandom *usel = new UtilitySelectorWeightedRandom;
  usel->utilityNodes = std::move(nodes);
  return usel;
}

BehNode *utility_selector_inertial(
    const std::vector<std::pair<BehNode *, utility_function>> &nodes,
    float inertiaAmount, float damp) {
  UtilitySelectorInertial *usel = new UtilitySelectorInertial(inertiaAmount, damp);
  usel->utilityNodes = std::move(nodes);
  usel->perNodeInertia.resize(usel->utilityNodes.size());
  return usel;
}

BehNode *move_to_entity(flecs::entity entity, const char *bb_name)
{
  return new MoveToEntity(entity, bb_name);
}

BehNode *is_low_hp(float thres)
{
  return new IsLowHp(thres);
}

BehNode *find_enemy(flecs::entity entity, float dist, const char *bb_name)
{
  return new FindEnemy(entity, dist, bb_name);
}

BehNode *flee(flecs::entity entity, const char *bb_name)
{
  return new Flee(entity, bb_name);
}

BehNode *patrol(flecs::entity entity, float patrol_dist, const char *bb_name)
{
  return new Patrol(entity, patrol_dist, bb_name);
}

BehNode *patch_up(float thres)
{
  return new PatchUp(thres);
}

BehNode *random_explore()
{
  return new RandomExplore();
}

BehNode *return_to_base()
{
  return new ReturnToBase();
}


