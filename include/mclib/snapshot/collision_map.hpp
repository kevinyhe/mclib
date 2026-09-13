// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/snapshot/types.hpp"

#include <array>
#include <cstdint>

namespace snapshot {

inline constexpr float FIELD_SIZE_IN = 144.0f;

inline constexpr std::uint32_t MAP_PERIMETER = 1u << 0;
inline constexpr std::uint32_t MAP_LONG_GOALS = 1u << 1;
inline constexpr std::uint32_t MAP_LONG_GOAL_BRACES = 1u << 2;
inline constexpr std::uint32_t MAP_CENTER_GOAL_POS45 = 1u << 3;
inline constexpr std::uint32_t MAP_CENTER_GOAL_NEG45 = 1u << 4;
inline constexpr std::uint32_t MAP_MATCHLOADERS = 1u << 5;

inline constexpr std::uint32_t MAP_LONG_GOALS_ALL =
    MAP_LONG_GOALS | MAP_LONG_GOAL_BRACES;
inline constexpr std::uint32_t MAP_CENTER_GOALS =
    MAP_CENTER_GOAL_POS45 | MAP_CENTER_GOAL_NEG45;
inline constexpr std::uint32_t MAP_ALL =
    MAP_PERIMETER | MAP_LONG_GOALS_ALL | MAP_CENTER_GOALS | MAP_MATCHLOADERS;

inline constexpr std::array<FieldSegment, 12> TERMINAL_FIELD_SEGMENTS{{
    {{0.0f, 0.0f}, {FIELD_SIZE_IN, 0.0f}, MAP_PERIMETER},
    {{FIELD_SIZE_IN, 0.0f}, {FIELD_SIZE_IN, FIELD_SIZE_IN}, MAP_PERIMETER},
    {{FIELD_SIZE_IN, FIELD_SIZE_IN}, {0.0f, FIELD_SIZE_IN}, MAP_PERIMETER},
    {{0.0f, FIELD_SIZE_IN}, {0.0f, 0.0f}, MAP_PERIMETER},
    {{18.0f, 36.0f}, {18.0f, 108.0f}, MAP_LONG_GOALS},
    {{126.0f, 36.0f}, {126.0f, 108.0f}, MAP_LONG_GOALS},
    {{18.0f, 36.0f}, {30.0f, 36.0f}, MAP_LONG_GOAL_BRACES},
    {{126.0f, 108.0f}, {114.0f, 108.0f}, MAP_LONG_GOAL_BRACES},
    {{60.0f, 60.0f}, {84.0f, 84.0f}, MAP_CENTER_GOAL_POS45},
    {{84.0f, 60.0f}, {60.0f, 84.0f}, MAP_CENTER_GOAL_NEG45},
    {{0.0f, 48.0f}, {12.0f, 48.0f}, MAP_MATCHLOADERS},
    {{132.0f, 96.0f}, {144.0f, 96.0f}, MAP_MATCHLOADERS},
}};

}  // namespace snapshot
