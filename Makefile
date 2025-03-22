null :=
space := $(null) $(null)
WEBOTS_HOME_PATH ?= $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

# Source files
SRC = autonomous_vehicle.c helper.c pid.c

# Object files
OBJ = build/release/autonomous_vehicle.o build/release/helper.o build/release/pid.o

# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -O2 -I$(WEBOTS_HOME_PATH)/include/controller/c -I.
CFLAGS += -I"C:/Users/Jagat Sachdeva/AppData/Local/Programs/Python/Python312/include"

# Linker flags
LDFLAGS = -L"C:/Users/Jagat Sachdeva/AppData/Local/Programs/Python/Python312/libs" -lpython312

# Libraries
LIBRARIES = -ldriver -lcar
LDFLAGS += $(LIBRARIES)

# Output executable
TARGET = build/release/autonomous_vehicle.exe

# Include Webots global Makefile
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Build rule
all: $(TARGET)

# Linking step
$(TARGET): $(OBJ)
	$(CC) $^ -o $@ $(LDFLAGS)

# Compilation step (ensures object files go to build/release/)
build/release/%.o: %.c
	@mkdir -p build/release
	$(CC) $(CFLAGS) -c $< -o $@

