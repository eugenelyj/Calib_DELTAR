CC := gcc

# Set to TRUE to enable logging functions.
LOG_ENABLE = FALSE

CORE_INCLUDE_PATHS = -Iuld-driver/inc
PLATFORM_INCLUDE_PATHS = -Iplatform

BASE_CFLAGS = -Wall -Werror -Wno-missing-braces -Wno-unused-result -Wno-unused-variable -Wno-int-conversion -Wno-unused-but-set-variable
CFLAGS_RELEASE = -Os -g0
#CFLAGS_RELEASE += -DSTMVL53L5CX_KERNEL

LIB_CORE_SOURCES =\
	$(wildcard uld-driver/src/*.c)

LIB_PLATFORM_SOURCES =\
	$(wildcard platform/*.c)


LIB_SOURCES := $(LIB_CORE_SOURCES) $(LIB_PLATFORM_SOURCES)
INCLUDE_PATH = $(CORE_INCLUDE_PATHS) $(PLATFORM_INCLUDE_PATHS)

CFLAGS = $(BASE_CFLAGS) $(CFLAGS_RELEASE) $(INCLUDE_PATH)

all:
	mkdir bin
	$(CC) $(CFLAGS) $(LIB_FLAGS) -o bin/long_record ./src/long_record.c $(LIB_CORE_SOURCES) $(LIB_PLATFORM_SOURCES)
	$(CC) $(CFLAGS) $(LIB_FLAGS) -o bin/short_record ./src/short_record.c $(LIB_CORE_SOURCES) $(LIB_PLATFORM_SOURCES)

clean:
	rm -f bin/long_record bin/short_record
