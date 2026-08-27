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
	$(INCDIR)/mclib/snapshot/*.hpp \
	$(INCDIR)/mclib/telemetry/*.hpp \
	$(INCDIR)/mclib/units/*.hpp

.DEFAULT_GOAL=quick

################################################################################
############################# Host-side unit tests #############################
# `make test` compiles every tests/*.cpp into its own host binary with the
# system g++ and runs it. It never touches the ARM toolchain or PROS headers.
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
# library - currently just systemMillis(). It is not globbed into TEST_SRCS
# because `make test` only globs tests/*.cpp, so nothing in there is mistaken
# for a test.
HOST_TEST_SRC:=$(SRCDIR)/mclib/math.cpp \
	$(SRCDIR)/mclib/utils.cpp \
	$(SRCDIR)/mclib/control/scaling.cpp \
	$(SRCDIR)/mclib/control/robot_state.cpp \
	$(SRCDIR)/mclib/control/odometry.cpp \
	$(SRCDIR)/mclib/pid.cpp \
	$(TESTDIR)/support/host_time.cpp

# One test per file: any tests/*.cpp with its own int main() returning 0 on
# success. No registration, no framework.
TEST_SRCS:=$(wildcard $(TESTDIR)/*.cpp)

.PHONY: test
test:
	@mkdir -p $(TESTBINDIR)
	@if [ -z "$(strip $(TEST_SRCS))" ]; then echo "No tests found in $(TESTDIR)"; exit 1; fi
	@failed=""; passed=0; \
	for src in $(TEST_SRCS); do \
	  name=`basename $$src .cpp`; \
	  bin=$(TESTBINDIR)/$$name; \
	  echo "== $$name"; \
	  if ! $(HOST_CXX) $(HOST_CXXFLAGS) -o $$bin $$src $(HOST_TEST_SRC) $(HOST_LDFLAGS); then \
	    echo "FAIL $$name (compile error)"; \
	    failed="$$failed $$name"; \
	    continue; \
	  fi; \
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
