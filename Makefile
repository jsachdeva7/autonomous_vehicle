# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Webots Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH ?= $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

# Source files
SRC = autonomous_vehicle.c helper.c

# Object files
OBJ = build/release/autonomous_vehicle.o build/release/helper.o

# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -O2 -I$(WEBOTS_HOME_PATH)/include/controller/c -I.

# Libraries
LIBRARIES = -ldriver -lcar

# Output executable
TARGET = build/release/autonomous_vehicle.exe

# Include Webots global Makefile
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Build rule
all: $(TARGET)

# Linking step
$(TARGET): $(OBJ)
	$(CC) $^ -o $@ $(LIBRARIES)

# Compilation step (ensures object files go to build/release/)
build/release/%.o: %.c
	@mkdir -p build/release
	$(CC) $(CFLAGS) -c $< -o $@

