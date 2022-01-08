#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := WaterRowerLinkPlus

COMPONENT_ADD_INCLUDEDIRS := components/include

set(ENV{EXTRA_CXXFLAGS} -Wmissing-field-initializers -Werror=switch)

include $(IDF_PATH)/make/project.mk

