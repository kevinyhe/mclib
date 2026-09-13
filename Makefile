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

# Library sources that compile without PROS headers, linked into every test.
# Expected to grow as more of src/ is made PROS-free.
#
# tests/support/ holds host-only stand-ins for the PROS-backed parts of the
# library - systemMillis() and the PROS competition-status calls the
# CommandScheduler makes. It is not globbed into TEST_SRCS
# because `make test` only globs tests/*.cpp, so nothing in there is mistaken
# for a test.
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
	$(SRCDIR)/mclib/pid.cpp \
	$(SRCDIR)/mclib/path/path.cpp \
	$(SRCDIR)/mclib/path/spline.cpp \
	$(SRCDIR)/mclib/path/pure_pursuit.cpp \
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
	$(SRCDIR)/mclib/mechanism/pneumatic_subsystem.cpp \
	$(SRCDIR)/mclib/telemetry/sd_sink.cpp \
	$(SRCDIR)/mclib/telemetry/file_sink.cpp \
	$(TESTDIR)/support/host_time.cpp \
	$(TESTDIR)/support/host_pros.cpp \
	$(TESTDIR)/support/host_pneumatic.cpp

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

-include $(MOTION_TEST_OBJS:.o=.d)

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
