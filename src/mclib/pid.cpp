// mclib
#include "mclib/pid.hpp"

// PIDController is a template, so its definitions live in the header. This
// translation unit exists to hold the one instantiation the whole library
// uses: PIDController<double, double>, spelled `PID` everywhere else.
//
// The matching `extern template` declaration in pid.hpp stops every other
// translation unit from emitting its own copy, which means each of them has an
// undefined reference to the symbols defined here and the linker is forced to
// pull pid.cpp.o out of libmclib.a. Deleting this file - or letting it become
// header-only boilerplate with no strong symbols - would let the archive
// member be dropped at link time.
template class PIDController<double, double>;
