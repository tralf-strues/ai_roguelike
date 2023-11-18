#include "dijkstraMapGen.h"
#include "ecsTypes.h"
#include "dungeonUtils.h"

template<typename Callable>
static void query_dungeon_data(flecs::world &ecs, Callable c)
{
  static auto dungeonDataQuery = ecs.query<const DungeonData>();

  dungeonDataQuery.each(c);
}

template<typename Callable>
static void query_characters_positions(flecs::world &ecs, Callable c)
{
  static auto characterPositionQuery = ecs.query<const Position, const Team>();

  characterPositionQuery.each(c);
}

constexpr float invalid_tile_value = 1e5f;

static void init_tiles(std::vector<float> &map, const DungeonData &dd)
{
  map.resize(dd.width * dd.height);
  for (float &v : map)
    v = invalid_tile_value;
}

// scan version, could be implemented as Dijkstra version as well
static void process_dmap(std::vector<float> &map, const DungeonData &dd)
{
  bool done = false;
  auto getMapAt = [&](size_t x, size_t y, float def)
  {
    if (x < dd.width && y < dd.width && dd.tiles[y * dd.width + x] == dungeon::floor)
      return map[y * dd.width + x];
    return def;
  };
  auto getMinNei = [&](size_t x, size_t y)
  {
    float val = map[y * dd.width + x];
    val = std::min(val, getMapAt(x - 1, y + 0, val));
    val = std::min(val, getMapAt(x + 1, y + 0, val));
    val = std::min(val, getMapAt(x + 0, y - 1, val));
    val = std::min(val, getMapAt(x + 0, y + 1, val));
    return val;
  };
  while (!done)
  {
    done = true;
    for (size_t y = 0; y < dd.height; ++y)
      for (size_t x = 0; x < dd.width; ++x)
      {
        const size_t i = y * dd.width + x;
        if (dd.tiles[i] != dungeon::floor)
          continue;
        const float myVal = getMapAt(x, y, invalid_tile_value);
        const float minVal = getMinNei(x, y);
        if (minVal < myVal - 1.f)
        {
          map[i] = minVal + 1.f;
          done = false;
        }
      }
  }
}

Position GetNearestVisible(const DungeonData& dd, const Position& from, const Position& to)
{
  Position cur = from;
  Position lastWalkable = cur;
  while (cur != to)
  {
    auto IsWalkable = [&](const Position& position) -> bool {
      return dd.tiles[position.y * dd.width + position.x] != dungeon::wall;
    };
    
    if (!IsWalkable(cur)) {
      break;
    }

    Position delta = to - cur;
    if (abs(delta.x) > abs(delta.y))
      cur.x += delta.x > 0 ? 1 : -1;
    else
      cur.y += delta.y > 0 ? 1 : -1;

    if (IsWalkable(cur)) {
      lastWalkable = cur;
    }
  }

  return lastWalkable;
}

void dmaps::gen_player_approach_map(flecs::world &ecs, std::vector<float> &map, int range)
{
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    init_tiles(map, dd);
    query_characters_positions(ecs, [&](const Position &pos, const Team &t)
    {
      if (t.team != 0) // player team hardcode
        return;

      for (int tx = -range; tx <= range; ++tx) {
        auto determine = [&](int ty) {
          Position nearest = GetNearestVisible(dd, pos, pos + Position{tx, ty});
          map[nearest.y * dd.width + nearest.x] = 0.f;
        };

        int ty = range - abs(tx);
        determine(ty);
        determine(-ty);
      }
    });
    process_dmap(map, dd);
  });
}

void dmaps::gen_player_flee_map(flecs::world &ecs, std::vector<float> &map)
{
  gen_player_approach_map(ecs, map);
  for (float &v : map)
    if (v < invalid_tile_value)
      v *= -1.2f;
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    process_dmap(map, dd);
  });
}

void dmaps::gen_hive_pack_map(flecs::world &ecs, std::vector<float> &map)
{
  static auto hiveQuery = ecs.query<const Position, const Hive>();
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    init_tiles(map, dd);
    hiveQuery.each([&](const Position &pos, const Hive &)
    {
      map[pos.y * dd.width + pos.x] = 0.f;
    });
    process_dmap(map, dd);
  });
}

void dmaps::gen_explore_map(flecs::world &ecs, std::vector<float> &map)
{
  static auto tileQuery = ecs.query<const Position, const Explored>();
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    init_tiles(map, dd);
    tileQuery.each([&](const Position &pos, const Explored &explored)
    {
      if (!explored.value && dungeon::is_tile_walkable(ecs, pos)) {
        map[pos.y * dd.width + pos.x] = 0.f;
      }
    });
    process_dmap(map, dd);
  });
}

void dmaps::gen_ally_map(flecs::world &ecs, std::vector<float> &map, flecs::entity target)
{
  static auto allyQuery = ecs.query<const Position, const Team>();
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    init_tiles(map, dd);
    target.get([&](const Team &targetTeam) {
      allyQuery.each([&](flecs::entity e, const Position &pos, const Team &team)
      {
        if (e != target && team.team == targetTeam.team) {
          map[pos.y * dd.width + pos.x] = 0.f;
        }
      });
    });
    process_dmap(map, dd);
  });
}
