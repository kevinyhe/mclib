# // mclib
################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=-Wno-deprecated-declarations
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=
EXTRA_INCDIR+=$(ROOT)/eigen

C_STANDARD?=gnu11
CXX_STANDARD?=gnu++20

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=1
LIBNAME:=mclib
VERSION:=0.1.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# Files that get distributed to every user beyond the compiled archive.
TEMPLATE_FILES=$(INCDIR)/mclib/*.hpp \
	$(INCDIR)/mclib/auton/*.hpp \
	$(INCDIR)/mclib/chassis/*.hpp \
	$(INCDIR)/mclib/command/*.h \
	$(INCDIR)/mclib/control/*.hpp \
	$(INCDIR)/mclib/device/*.hpp \
	$(INCDIR)/mclib/mechanism/*.hpp \
	$(INCDIR)/mclib/path/*.hpp \
	$(INCDIR)/mclib/snapshot/*.hpp \
	$(INCDIR)/mclib/telemetry/*.hpp \
	$(INCDIR)/mclib/units/*.hpp \
	$(shell find $(INCDIR)/Eigen -type f)

.DEFAULT_GOAL=quick

################################################################################
############################# Host-side unit tests #############################
# `make test` compiles every tests/*.cpp into its own host binary with the
# system g++ and runs it. It uses the SDK headers but never the ARM toolchain.
# tests/ lives outside src/, so the firmware build never sees it.
TESTDIR:=$(ROOT)/tests
TESTBINDIR:=$(BINDIR)/tests
HOST_CXX?=g++
# MCLIB_HOST_BUILD picks std::mutex over pros::Mutex in mclib/sync.hpp. The
# PROS headers are on the include path even here, so the switch has to be an
# explicit macro rather than an availability check.
HOST_CXXFLAGS?=-std=$(CXX_STANDARD) -I$(INCDIR) -I$(TESTDIR) -DMCLIB_HOST_BUILD -Wall -Wextra -g -O1
HOST_LDFLAGS?=-pthread

# Library sources and host stand-ins linked into every test.
#
# tests/support/ holds host-only stand-ins for what the library gets from
# libpros. It is not globbed into TEST_SRCS because `make test` only globs
# tests/*.cpp, so nothing in there is mistaken for a test.
#   host_time.cpp       mclib::time::systemMillis()
#   host_pros.cpp       the competition-status calls CommandScheduler makes
#   host_pneumatic.cpp  mclib::device::Pneumatic, in place of device/pneumatic.cpp
#   host_devices.cpp    pros::Motor, MotorGroup, Imu, Controller and Mutex
# Each class has one stand-in, in one of these files. A test that
# needs a new call adds it there rather than writing its own.
#
# Nothing in HOST_TEST_SRC may define pros::millis(), pros::delay() or
# pros::Task: motion_safety_test defines its own clock, and
# auton_selector_test gets one from host_screen.cpp. Those two have their own
# link rules below.
HOST_TEST_SRC:=$(SRCDIR)/mclib/auton/time_budget.cpp \
	$(SRCDIR)/mclib/math.cpp \
	$(SRCDIR)/mclib/utils.cpp \
	$(SRCDIR)/mclib/chassis/chassis_math.cpp \
	$(SRCDIR)/mclib/control/scaling.cpp \
	$(SRCDIR)/mclib/control/motion_config.cpp \
	$(SRCDIR)/mclib/control/drive_curve.cpp \
	$(SRCDIR)/mclib/chassis/holonomic_math.cpp \
	$(SRCDIR)/mclib/control/motion_math.cpp \
	$(SRCDIR)/mclib/control/robot_state.cpp \
	$(SRCDIR)/mclib/control/odometry.cpp \
	$(SRCDIR)/mclib/control/profile.cpp \
	$(SRCDIR)/mclib/control/feedforward.cpp \
	$(SRCDIR)/mclib/control/ramsete.cpp \
	$(SRCDIR)/mclib/control/pose_filter.cpp \
	$(SRCDIR)/mclib/control/holonomic_follower.cpp \
	$(SRCDIR)/mclib/pid.cpp \
	$(SRCDIR)/mclib/path/path.cpp \
	$(SRCDIR)/mclib/path/spline.cpp \
	$(SRCDIR)/mclib/path/pure_pursuit.cpp \
	$(SRCDIR)/mclib/path/trajectory.cpp \
	$(SRCDIR)/mclib/snapshot/raycast.cpp \
	$(SRCDIR)/mclib/snapshot/snapshot_pose.cpp \
	$(SRCDIR)/mclib/command/subsystem.cpp \
	$(SRCDIR)/mclib/mechanism/position_mechanism.cpp \
	$(SRCDIR)/mclib/mechanism/homing_mechanism.cpp \
	$(SRCDIR)/mclib/mechanism/auto_trigger_mechanism.cpp \
	$(SRCDIR)/mclib/mechanism/mechanism_manager.cpp \
	$(SRCDIR)/mclib/mechanism/velocity_mechanism.cpp \
	$(SRCDIR)/mclib/mechanism/toggle_mechanism.cpp \
	$(SRCDIR)/mclib/mechanism/toggle_group_mechanism.cpp \
	$(SRCDIR)/mclib/mechanism/motor_subsystem.cpp \
	$(SRCDIR)/mclib/mechanism/pneumatic_subsystem.cpp \
	$(SRCDIR)/mclib/mechanism/conveyor_mechanism.cpp \
	$(SRCDIR)/mclib/mechanism/pto_mechanism.cpp \
	$(SRCDIR)/mclib/chassis/holonomic_chassis.cpp \
	$(SRCDIR)/mclib/chassis/holonomic_controller.cpp \
	$(SRCDIR)/mclib/device/motor.cpp \
	$(SRCDIR)/mclib/device/motor_group.cpp \
	$(SRCDIR)/mclib/device/inertial.cpp \
	$(SRCDIR)/mclib/device/controller.cpp \
	$(SRCDIR)/mclib/device/types.cpp \
	$(SRCDIR)/mclib/telemetry/sd_sink.cpp \
	$(SRCDIR)/mclib/telemetry/file_sink.cpp \
	$(TESTDIR)/support/host_time.cpp \
	$(TESTDIR)/support/host_pros.cpp \
	$(TESTDIR)/support/host_pneumatic.cpp \
	$(TESTDIR)/support/host_devices.cpp

# One test per file: any tests/*.cpp with its own int main() returning 0 on
# success. No registration, no framework.
TEST_SRCS:=$(wildcard $(TESTDIR)/*.cpp)
TEST_BINS:=$(patsubst $(TESTDIR)/%.cpp,$(TESTBINDIR)/%,$(TEST_SRCS))

# The library sources are compiled once into objects and linked into every
# test binary. They used to be recompiled from source for each of the ~30
# tests, which made `make test` a five-minute wait; now `make -j test` builds
# the objects in parallel and links each test in well under a minute.
HOST_OBJDIR:=$(TESTBINDIR)/obj
HOST_TEST_OBJS:=$(patsubst %.cpp,$(HOST_OBJDIR)/%.o,$(HOST_TEST_SRC))
HOST_DEPS:=$(HOST_TEST_OBJS:.o=.d) $(TEST_BINS:=.d)

$(HOST_OBJDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling $<"
	@$(HOST_CXX) $(HOST_CXXFLAGS) -MMD -MP -c -o $@ $<

$(TESTBINDIR)/%: $(TESTDIR)/%.cpp $(HOST_TEST_OBJS)
	@mkdir -p $(TESTBINDIR)
	@echo "Linking $@"
	@$(HOST_CXX) $(HOST_CXXFLAGS) -MMD -MP -MF $@.d -o $@ $< $(HOST_TEST_OBJS) $(HOST_LDFLAGS)

# Only the hardware-loop regression supplies a PROS clock and DriveHardware.
MOTION_TEST_OBJS:=$(HOST_OBJDIR)/$(SRCDIR)/mclib/control/motion.o \
	$(HOST_OBJDIR)/$(SRCDIR)/mclib/control/chassis_io.o
$(TESTBINDIR)/motion_safety_test: $(TESTDIR)/motion_safety_test.cpp $(HOST_TEST_OBJS) $(MOTION_TEST_OBJS)
	@mkdir -p $(dir $@)
	@$(HOST_CXX) $(HOST_CXXFLAGS) -MMD -MP -MF $@.d -o $@ $< $(HOST_TEST_OBJS) $(MOTION_TEST_OBJS) $(HOST_LDFLAGS)

# AsyncMotion runs the real motion loops on a std::thread against a wall clock
# the test supplies.
$(TESTBINDIR)/async_motion_test: $(TESTDIR)/async_motion_test.cpp $(HOST_TEST_OBJS) $(MOTION_TEST_OBJS) $(HOST_OBJDIR)/$(SRCDIR)/mclib/control/async_motion.o
	@mkdir -p $(dir $@)
	@echo "Linking $@"
	@$(HOST_CXX) $(HOST_CXXFLAGS) -MMD -MP -MF $@.d -o $@ $< $(HOST_TEST_OBJS) $(MOTION_TEST_OBJS) $(HOST_OBJDIR)/$(SRCDIR)/mclib/control/async_motion.o $(HOST_LDFLAGS)

-include $(MOTION_TEST_OBJS:.o=.d)

# Only the auton selector test links the real selector, over the screen,
# clock and task stand-ins in tests/support/host_screen.cpp. That file
# defines pros::millis(), which motion_safety_test defines too, so it stays
# out of HOST_TEST_SRC.
SELECTOR_TEST_OBJS:=$(HOST_OBJDIR)/$(SRCDIR)/mclib/auton/selector.o \
	$(HOST_OBJDIR)/$(TESTDIR)/support/host_screen.o
$(TESTBINDIR)/auton_selector_test: $(TESTDIR)/auton_selector_test.cpp $(HOST_TEST_OBJS) $(SELECTOR_TEST_OBJS)
	@mkdir -p $(dir $@)
	@echo "Linking $@"
	@$(HOST_CXX) $(HOST_CXXFLAGS) -MMD -MP -MF $@.d -o $@ $< $(HOST_TEST_OBJS) $(SELECTOR_TEST_OBJS) $(HOST_LDFLAGS)

-include $(SELECTOR_TEST_OBJS:.o=.d)

-include $(HOST_DEPS)

.PHONY: test test-build
test-build: $(TEST_BINS)

test: test-build
	@if [ -z "$(strip $(TEST_SRCS))" ]; then echo "No tests found in $(TESTDIR)"; exit 1; fi
	@failed=""; passed=0; \
	for bin in $(TEST_BINS); do \
	  name=`basename $$bin`; \
	  echo "== $$name"; \
	  if $$bin; then passed=`expr $$passed + 1`; else failed="$$failed $$name"; fi; \
	done; \
	echo; \
	if [ -n "$$failed" ]; then \
	  echo "FAILED TESTS:$$failed"; \
	  exit 1; \
	fi; \
	echo "All $$passed test(s) passed."

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
