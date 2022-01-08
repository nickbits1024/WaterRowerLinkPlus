#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

CFLAGS += -Wmissing-field-initializers
CFLAGS += -Werror=switch
CXXFLAGS += -Wmissing-field-initializers
CXXFLAGS += -Werror=switch
CPPFLAGS += -Wmissing-field-initializers
CPPFLAGS += -Werror=switch

set(alkf)