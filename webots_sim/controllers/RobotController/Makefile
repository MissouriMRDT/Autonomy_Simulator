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

### Generic Makefile.include for Webots controllers, physics plugins, robot
### window libraries, remote control libraries and other libraries
### to be used with GNU make
###
### Platforms: Windows, macOS, Linux
### Languages: C, C++
###
### Authors: Olivier Michel, Yvan Bourquin, Fabien Rohrer
###          Edmund Ronald, Sergei Poskriakov
###
###-----------------------------------------------------------------------------
###
### This file is meant to be included from the Makefile files located in the
### Webots projects subdirectories. It is possible to set a number of variables
### to customize the build process, i.e., add source files, compilation flags,
### include paths, libraries, etc. These variables should be set in your local
### Makefile just before including this Makefile.include. This Makefile.include
### should never be modified.
###
### Here is a description of the variables you may set in your local Makefile:
###
### ---- C Sources ----
### if your program uses several C source files:
### C_SOURCES = my_plugin.c my_clever_algo.c my_graphics.c
###
### ---- C++ Sources ----
### if your program uses several C++ source files:
### CXX_SOURCES = my_plugin.cc my_clever_algo.cpp my_graphics.c++
###
### ---- Compilation options ----
### if special compilation flags are necessary:
### CFLAGS = -Wno-unused-result
###
### ---- Linked libraries ----
### if your program needs additional libraries:
### INCLUDE = -I"/my_library_path/include"
### LIBRARIES = -L"/path/to/my/library" -lmy_library -lmy_other_library
###
### ---- Linking options ----
### if special linking flags are needed:
### LFLAGS = -s
###
### ---- Webots included libraries ----
### if you want to use the Webots C API in your C++ controller program:
### USE_C_API = true
###
### ---- Debug mode ----
### if you want to display the gcc command line for compilation and link, as
### well as the rm command details used for cleaning:
### VERBOSE = 1
###
###-----------------------------------------------------------------------------

# Recursively find .cpp files in current directory.
CXX_SOURCES = $(shell find $(SRC_DIR) -name '*.cpp')
## Link Libraries.
INCLUDE = -I"./RoveComm_CPP/include/"
LIBRARIES = -L"./RoveComm_CPP/lib" -lRoveComm_CPP


# Download and run the script package if it doesn't exist.
ROVECOMM_VERSION = v24.2.0
ROVECOMM_INSTALLER = RoveComm_CPP_AMD64.sh

ifeq ($(wildcard RoveComm_CPP_AMD64.sh),)
    $(info Downloading RoveComm installer...)
    $(shell curl -LOk https://github.com/MissouriMRDT/RoveComm_CPP/releases/download/$(ROVECOMM_VERSION)/RoveComm_CPP_AMD64.sh)
endif

ifeq ($(wildcard RoveComm_CPP),)
    $(info Installing RoveComm...)
    _tmp := $(shell sh ./RoveComm_CPP_AMD64.sh --skip-license)
    _tmp := $(shell mkdir -p RoveComm_CPP/)
    _tmp := $(shell mv external 'include' lib RoveComm_CPP/)
else
    $(info RoveComm is already installed.)
endif

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include