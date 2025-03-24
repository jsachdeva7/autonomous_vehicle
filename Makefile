# Makefile for compiling the Webots autonomous vehicle controller

# Handle spaces in the WEBOTS_HOME path (for compatibility)
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH ?= $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

# Source files
SRC = src/autonomous_vehicle.c src/pid.c src/devices.c src/control.c src/init.c src/lane_keeping.c src/traffic_detection.c

# Object files (corresponding .o files for the source files)
OBJ = $(patsubst src/%, build/release/%, $(SRC:.c=.o))

# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -O2 
CFLAGS += -I$(WEBOTS_HOME_PATH)/include/controller/c -I. # Webots controller headers
CFLAGS += -I"C:/Users/Jagat Sachdeva/AppData/Local/Programs/Python/Python312/include" # Python headers

# Linker flags
LDFLAGS = -L"C:/Users/Jagat Sachdeva/AppData/Local/Programs/Python/Python312/libs" -lpython312 # Link against Python 3.12
LDFLAGS += -L"C:/Program Files/Webots/lib/controller" -lController -ldriver -lcar

# Output executable
TARGET = build/release/autonomous_vehicle.exe

# Include Webots global Makefile for necessary rules and variables
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Default build target
all: $(TARGET)

# Linking step
$(TARGET): $(OBJ)
	$(CC) $^ -o $@ $(LDFLAGS)

# Compilation step (ensures object files go to build/release/)
build/release/%.o: src/%.c
	@mkdir -p build/release
	$(CC) $(CFLAGS) -c $< -o $@
