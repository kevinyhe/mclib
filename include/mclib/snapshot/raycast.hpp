// mclib
#pragma once

#include "mclib/snapshot/collision_map.hpp"

#include <cstdint>
#include <vector>

namespace snapshot {

float cross(const Vec2& a, const Vec2& b);
Vec2 sub(const Vec2& a, const Vec2& b);
Vec2 add(const Vec2& a, const Vec2& b);
Vec2 mul(const Vec2& a, float s);
float dot(const Vec2& a, const Vec2& b);
float norm(const Vec2& a);

Vec2 unit_from_heading_deg(float heading_deg);

std::vector<RayHit> raycast_all(const Vec2& origin,
                                const Vec2& dir_unit,
                                float max_range_in,
                                std::uint32_t allow_mask);

}  // namespace snapshot
