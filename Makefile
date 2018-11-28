#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

#VERBOSE := 1

PROJECT_NAME := esp32-poolmon

include $(IDF_PATH)/make/project.mk

BUILD_TIMESTAMP := $(shell date +"%Y%m%d-%H%M%S")
GIT_COMMIT := $(shell git describe --match=NeVeRmAtCh --always --abbrev=14 --dirty)

CFLAGS += -DBUILD_TIMESTAMP=\"$(BUILD_TIMESTAMP)\" -DGIT_COMMIT=\"$(GIT_COMMIT)\"

# attempt to force a recompile if these variables change (doesn't work yet):
#define DEPENDABLE_VAR
#
#.PHONY: phony
#$1: phony
#    @if [[ `cat $1 2>&1` != '$($1)' ]]; then \
#        echo -n $($1) > $1 ; \
#    fi
#
#endef
#
##declare BUILD_TIMESTAMP to be dependable
#$(eval $(call DEPENDABLE_VAR,BUILD_TIMESTAMP))
#$(eval $(call DEPENDABLE_VAR,GIT_COMMIT))
#
#main/resources.c: BUILD_TIMESTAMP GIT_COMMIT
