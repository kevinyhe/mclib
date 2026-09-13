// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
