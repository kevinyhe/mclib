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
	$(INCDIR)/mclib/units/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
