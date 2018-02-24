#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

#VERBOSE := 1

PROJECT_NAME := esp32-poolmon

include $(IDF_PATH)/make/project.mk

BUILD_TIMESTAMP := $(shell date +"%Y%m%d-%H%M%S")
CFLAGS += -DBUILD_TIMESTAMP=\"$(BUILD_TIMESTAMP)\"
