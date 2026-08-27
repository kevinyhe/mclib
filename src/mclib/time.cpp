// mclib
#include "mclib/time.hpp"

#include "pros/rtos.hpp"

#include <cstdint>

namespace mclib {
namespace time {

std::uint32_t systemMillis() { return pros::millis(); }

}  // namespace time
}  // namespace mclib
