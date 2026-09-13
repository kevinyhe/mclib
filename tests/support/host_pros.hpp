#pragma once
#include <cstdint>

namespace mclib::test {
// PROS competition flags: disabled=1, autonomous=2, connected=4.
void setCompetitionStatus(std::uint8_t status);
}
