#include "pathfinder.h"
#include "dungeonUtils.h"
#include "math.h"
#include <algorithm>
#include <limits>

constexpr int32_t kTileSize = 10;

float heuristic(IVec2 lhs, IVec2 rhs)
{
  return sqrtf(sqr(float(lhs.x - rhs.x)) + sqr(float(lhs.y - rhs.y)));
};

template<typename T>
static size_t coord_to_idx(T x, T y, size_t w)
{
  return size_t(y) * w + size_t(x);
}

static std::vector<IVec2> reconstruct_path(std::vector<IVec2> prev, IVec2 to, size_t width)
{
  IVec2 curPos = to;
  std::vector<IVec2> res = {curPos};
  while (prev[coord_to_idx(curPos.x, curPos.y, width)] != IVec2{-1, -1})
  {
    curPos = prev[coord_to_idx(curPos.x, curPos.y, width)];
    res.insert(res.begin(), curPos);
  }
  return res;
}

static std::vector<IVec2> find_path_a_star(const DungeonData &dd, IVec2 from, IVec2 to,
                                           IVec2 lim_min, IVec2 lim_max)
{
  if (from.x < 0 || from.y < 0 || from.x >= int(dd.width) || from.y >= int(dd.height))
    return std::vector<IVec2>();
  size_t inpSize = dd.width * dd.height;

  std::vector<float> g(inpSize, std::numeric_limits<float>::max());
  std::vector<float> f(inpSize, std::numeric_limits<float>::max());
  std::vector<IVec2> prev(inpSize, IVec2{-1,-1});

  auto getG = [&](IVec2 p) -> float { return g[coord_to_idx(p.x, p.y, dd.width)]; };
  auto getF = [&](IVec2 p) -> float { return f[coord_to_idx(p.x, p.y, dd.width)]; };

  g[coord_to_idx(from.x, from.y, dd.width)] = 0;
  f[coord_to_idx(from.x, from.y, dd.width)] = heuristic(from, to);

  std::vector<IVec2> openList = {from};
  std::vector<IVec2> closedList;

  while (!openList.empty())
  {
    size_t bestIdx = 0;
    float bestScore = getF(openList[0]);
    for (size_t i = 1; i < openList.size(); ++i)
    {
      float score = getF(openList[i]);
      if (score < bestScore)
      {
        bestIdx = i;
        bestScore = score;
      }
    }
    if (openList[bestIdx] == to)
      return reconstruct_path(prev, to, dd.width);
    IVec2 curPos = openList[bestIdx];
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end())
      continue;
    size_t idx = coord_to_idx(curPos.x, curPos.y, dd.width);
    closedList.emplace_back(curPos);
    auto checkNeighbour = [&](IVec2 p)
    {
      // out of bounds
      if (p.x < lim_min.x || p.y < lim_min.y || p.x >= lim_max.x || p.y >= lim_max.y)
        return;
      size_t idx = coord_to_idx(p.x, p.y, dd.width);
      // not empty
      if (dd.tiles[idx] == dungeon::wall)
        return;
      float edgeWeight = 1.f;
      float gScore = getG(curPos) + 1.f * edgeWeight; // we're exactly 1 unit away
      if (gScore < getG(p))
      {
        prev[idx] = curPos;
        g[idx] = gScore;
        f[idx] = gScore + heuristic(p, to);
      }
      bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
      if (!found)
        openList.emplace_back(p);
    };
    checkNeighbour(IVec2{curPos.x + 1, curPos.y + 0});
    checkNeighbour(IVec2{curPos.x - 1, curPos.y + 0});
    checkNeighbour(IVec2{curPos.x + 0, curPos.y + 1});
    checkNeighbour(IVec2{curPos.x + 0, curPos.y - 1});
  }
  // empty path
  return std::vector<IVec2>();
}


void prebuild_map(flecs::world &ecs)
{
  auto mapQuery = ecs.query<const DungeonData>();

  ecs.defer([&]()
  {
    mapQuery.each([&](flecs::entity e, const DungeonData &dd)
    {
      // go through each super tile
      const size_t width = dd.width / kTileSize;
      const size_t height = dd.height / kTileSize;

      auto check_border = [&](size_t xx, size_t yy,
                              size_t dir_x, size_t dir_y,
                              int offs_x, int offs_y,
                              std::vector<PathPortal> &portals)
      {
        int spanFrom = -1;
        int spanTo = -1;
        for (size_t i = 0; i < kTileSize; ++i)
        {
          size_t x = xx * kTileSize + i * dir_x;
          size_t y = yy * kTileSize + i * dir_y;
          size_t nx = x + offs_x;
          size_t ny = y + offs_y;
          if (dd.tiles[y * dd.width + x] != dungeon::wall &&
              dd.tiles[ny * dd.width + nx] != dungeon::wall)
          {
            if (spanFrom < 0)
              spanFrom = i;
            spanTo = i;
          }
          else if (spanFrom >= 0)
          {
            // write span
            portals.push_back({xx * kTileSize + spanFrom * dir_x + offs_x,
                               yy * kTileSize + spanFrom * dir_y + offs_y,
                               xx * kTileSize + spanTo * dir_x,
                               yy * kTileSize + spanTo * dir_y});
            spanFrom = -1;
          }
        }
        if (spanFrom >= 0)
        {
          portals.push_back({xx * kTileSize + spanFrom * dir_x + offs_x,
                             yy * kTileSize + spanFrom * dir_y + offs_y,
                             xx * kTileSize + spanTo * dir_x,
                             yy * kTileSize + spanTo * dir_y});
        }
      };

      std::vector<PathPortal> portals;
      std::vector<std::vector<size_t>> tilePortalsIndices;

      auto push_portals = [&](size_t x, size_t y,
                              int offs_x, int offs_y,
                              const std::vector<PathPortal> &new_portals)
      {
        for (const PathPortal &portal : new_portals)
        {
          size_t idx = portals.size();
          portals.push_back(portal);
          tilePortalsIndices[y * width + x].push_back(idx);
          tilePortalsIndices[(y + offs_y) * width + x + offs_x].push_back(idx);
        }
      };
      for (size_t y = 0; y < height; ++y)
        for (size_t x = 0; x < width; ++x)
        {
          tilePortalsIndices.push_back(std::vector<size_t>{});
          // check top
          if (y > 0)
          {
            std::vector<PathPortal> topPortals;
            check_border(x, y, 1, 0, 0, -1, topPortals);
            push_portals(x, y, 0, -1, topPortals);
          }
          // left
          if (x > 0)
          {
            std::vector<PathPortal> leftPortals;
            check_border(x, y, 0, 1, -1, 0, leftPortals);
            push_portals(x, y, -1, 0, leftPortals);
          }
        }
      for (size_t tidx = 0; tidx < tilePortalsIndices.size(); ++tidx)
      {
        const std::vector<size_t> &indices = tilePortalsIndices[tidx];
        size_t x = tidx % width;
        size_t y = tidx / width;
        IVec2 limMin{int((x + 0) * kTileSize), int((y + 0) * kTileSize)};
        IVec2 limMax{int((x + 1) * kTileSize), int((y + 1) * kTileSize)};
        for (size_t i = 0; i < indices.size(); ++i)
        {
          PathPortal &firstPortal = portals[indices[i]];
          for (size_t j = i + 1; j < indices.size(); ++j)
          {
            PathPortal &secondPortal = portals[indices[j]];
            // check path from i to j
            // check each position (to find closest dist) (could be made more optimal)
            bool noPath = false;
            size_t minDist = 0xffffffff;
            for (size_t fromY = std::max(firstPortal.startY, size_t(limMin.y));
                        fromY <= std::min(firstPortal.endY, size_t(limMax.y - 1)) && !noPath; ++fromY)
            {
              for (size_t fromX = std::max(firstPortal.startX, size_t(limMin.x));
                          fromX <= std::min(firstPortal.endX, size_t(limMax.x - 1)) && !noPath; ++fromX)
              {
                for (size_t toY = std::max(secondPortal.startY, size_t(limMin.y));
                            toY <= std::min(secondPortal.endY, size_t(limMax.y - 1)) && !noPath; ++toY)
                {
                  for (size_t toX = std::max(secondPortal.startX, size_t(limMin.x));
                              toX <= std::min(secondPortal.endX, size_t(limMax.x - 1)) && !noPath; ++toX)
                  {
                    IVec2 from{int(fromX), int(fromY)};
                    IVec2 to{int(toX), int(toY)};
                    std::vector<IVec2> path = find_path_a_star(dd, from, to, limMin, limMax);
                    if (path.empty() && from != to)
                    {
                      noPath = true; // if we found that there's no path at all - we can break out
                      break;
                    }
                    minDist = std::min(minDist, path.size());
                  }
                }
              }
            }
            // write pathable data and length
            if (noPath)
              continue;
            firstPortal.conns.push_back({indices[j], float(minDist)});
            secondPortal.conns.push_back({indices[i], float(minDist)});
          }
        }
      }
      e.set(DungeonPortals{kTileSize, portals, tilePortalsIndices});
    });
  });
}

std::vector<IVec2> find_path(const DungeonData &dd, const DungeonPortals& dp, IVec2 from, IVec2 to) {
  if (from == to) {
    return {};
  }

  const IVec2 fromTile = from / kTileSize;
  const IVec2 toTile = to / kTileSize;

  if (fromTile == toTile) {
    auto path = find_path_a_star(dd, from, to, fromTile * kTileSize, (fromTile + IVec2{1}) * kTileSize);

    if (!path.empty()) {
      return path;
    }
  }

  std::vector<IVec2> nodes;
  for (const auto& portal : dp.portals) {
    nodes.push_back(IVec2{static_cast<int>(portal.startX + portal.endX), static_cast<int>(portal.startY + portal.endY)} / 2);
  }

  nodes.push_back(from);
  nodes.push_back(to);

  const size_t portalsCount = dp.portals.size();
  const size_t nodesCount = nodes.size();
  const size_t fromIdx = nodesCount - 2;
  const size_t toIdx = nodesCount - 1;
  constexpr float kCostMax = std::numeric_limits<float>::max();

  std::vector<std::vector<float>> costMatrix(nodesCount, std::vector<float>(nodesCount, kCostMax));

  for (size_t pointIdx = fromIdx; pointIdx < nodes.size(); ++pointIdx) {
    for (size_t portalIdx = 0; portalIdx < portalsCount; ++portalIdx) {
      if (dist(nodes[pointIdx], nodes[portalIdx]) <= 2 * kTileSize) {
        auto path = find_path_a_star(dd, nodes[pointIdx], nodes[portalIdx], fromTile * kTileSize, (fromTile + IVec2{1}) * kTileSize);
        costMatrix[pointIdx][portalIdx] = path.size();
        costMatrix[portalIdx][pointIdx] = path.size();
      }
    }
  }

  /* A* on the graph */
  constexpr size_t kInvalidIdx = std::numeric_limits<size_t>::max();

  std::vector<float> g(nodesCount, kCostMax);
  std::vector<float> f(nodesCount, kCostMax);
  std::vector<size_t> prev(nodesCount, kInvalidIdx);

  auto graphHeuristic = [&](size_t from, size_t to) {
    return dist(nodes[from], nodes[to]);
  };

  auto reconstructPath = [&]() {
    size_t cur = toIdx;
    std::vector<size_t> path = {cur};

    while (prev[cur] != kInvalidIdx) {
      cur = prev[cur];
      path.insert(path.begin(), cur);
    }

    return path;
  };

  g[fromIdx] = 0;
  f[fromIdx] = graphHeuristic(fromIdx, toIdx);

  std::vector<size_t> openList = {fromIdx};
  std::vector<size_t> closedList;

  std::vector<size_t> nodesPath;

  while (!openList.empty()) {
    size_t bestIdx = 0;
    float bestScore = f[openList[0]];
    for (size_t i = 1; i < openList.size(); ++i) {
      float score = f[openList[i]];
      if (score < bestScore) {
        bestIdx = i;
        bestScore = score;
      }
    }

    if (openList[bestIdx] == toIdx) {
      nodesPath = reconstructPath();
    }

    size_t cur = openList[bestIdx];
    // IVec2 curPos = nodes[cur];
    
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), cur) != closedList.end()) {
      continue;
    }
    
    // size_t idx = coord_to_idx(curPos.x, curPos.y, dd.width);
    closedList.emplace_back(cur);
    
    auto checkNeighbour = [&](size_t p) {
      // // out of bounds
      // if (p.x < lim_min.x || p.y < lim_min.y || p.x >= lim_max.x || p.y >= lim_max.y)
      //   return;

      // size_t idx = coord_to_idx(p.x, p.y, dd.width);
      
      // // not empty
      // if (dd.tiles[idx] == dungeon::wall)
      //   return;

      float gScore = g[cur] + costMatrix[cur][p];

      if (gScore < g[p]) {
        prev[p] = cur;
        g[p] = gScore;
        f[p] = gScore + graphHeuristic(p, toIdx);
      }

      bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
      if (!found) {
        openList.emplace_back(p);
      }
    };

    for (size_t p = 0; p < costMatrix[cur].size(); ++p) {
      if (costMatrix[cur][p] != kCostMax) {
        checkNeighbour(p);
      }
    }
  }

  /* Reconstruct path from nodes to actual tiles */
  size_t cur = toIdx;
  std::vector<IVec2> tilePath = {nodes[cur]};

  while (prev[cur] != kInvalidIdx) {
    size_t prevNode = prev[cur];

    IVec2 limMin, limMax;
    limMin = (nodes[cur] / kTileSize) * kTileSize;
    limMax = ((nodes[cur] + IVec2{1}) / kTileSize) * kTileSize;

    limMin = std::min(limMin, (nodes[prevNode] / kTileSize) * kTileSize);
    limMax = std::max(limMax, ((nodes[cur] + IVec2{1}) / kTileSize) * kTileSize);

    auto partialPath = find_path_a_star(dd, nodes[cur], nodes[prevNode], limMin, limMax);
    tilePath.insert(tilePath.end(), partialPath.begin(), partialPath.end());
  }

  std::reverse(tilePath.begin(), tilePath.end());

  return tilePath;
}
