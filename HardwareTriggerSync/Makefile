################################################################################
# HardwareTriggerSync Makefile
################################################################################
PROJECT_ROOT=../../
OPT_INC = ${PROJECT_ROOT}/common/make/common_spin.mk
-include ${OPT_INC}

################################################################################
# Key paths and settings
################################################################################
CFLAGS += -std=c++11
ifeq ($(wildcard ${OPT_INC}),)
CXX = g++ ${CFLAGS}
ODIR  = .obj/build${D}
SDIR  = .
MKDIR = mkdir -p
PLATFORM = $(shell uname)
ifeq ($(PLATFORM),Darwin)
OS = mac
endif
endif

OUTPUTNAME = HardwareTriggerSync${D}
OUTDIR = ../../bin

################################################################################
# Dependencies
################################################################################
# Spinnaker deps
SPINNAKER_LIB = -L../../lib -lSpinnaker${D} ${SPIN_DEPS}

################################################################################
# Master inc/lib/obj/dep settings
################################################################################
_OBJ = HardwareTriggerSync.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
INC = -I../../include
ifneq ($(OS),mac)
INC += -I/opt/spinnaker/include
LIB += -Wl,-Bdynamic ${SPINNAKER_LIB}
LIB += -Wl,-rpath-link=../../lib
else
INC += -I/usr/local/include/spinnaker
LIB += -rpath ../../lib/
LIB += ${SPINNAKER_LIB}
endif

################################################################################
# Rules/recipes
################################################################################
# Final binary
${OUTPUTNAME}: ${OBJ}
	${CXX} -o ${OUTPUTNAME} ${OBJ} ${LIB}
	mv ${OUTPUTNAME} ${OUTDIR}

# Intermediate object files
${OBJ}: ${ODIR}/%.o : ${SDIR}/%.cpp
	@${MKDIR} ${ODIR}
	${CXX} ${CFLAGS} ${INC} -Wall -D LINUX -c $< -o $@

# Clean up intermediate objects
clean_obj:
	rm -f ${OBJ}
	@echo "intermediate objects cleaned up!"

# Clean up everything.
clean: clean_obj
	rm -f ${OUTDIR}/${OUTPUTNAME}
	@echo "all cleaned up!"
