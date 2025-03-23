# Makefile for compiling the Webots autonomous vehicle controller

# Handle spaces in the WEBOTS_HOME path (for compatibility)
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH ?= $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

# Source files
SRC = autonomous_vehicle.c pid.c devices.c

# Object files (corresponding .o files for the source files)
OBJ = build/release/autonomous_vehicle.o build/release/pid.o build/release/devices.o

# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -O2 
CFLAGS += -I$(WEBOTS_HOME_PATH)/include/controller/c -I. # Webots controller headers
CFLAGS += -I"C:/Users/Jagat Sachdeva/AppData/Local/Programs/Python/Python312/include" # Python headers

# Linker flags
LDFLAGS = -L"C:/Users/Jagat Sachdeva/AppData/Local/Programs/Python/Python312/libs" -lpython312 # Link against Python 3.12
LIBRARIES = -ldriver -lcar
LDFLAGS += $(LIBRARIES)

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
build/release/%.o: %.c
	@mkdir -p build/release
	$(CC) $(CFLAGS) -c $< -o $@
