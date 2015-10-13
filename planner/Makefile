# Copyright 2015 Ciara Kamahele-Sanfratello
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# This Makefile is specific to Mac OS X with a Homebrew installation, but should
# be simple to adapt to other systems.
#
# You need to have clang and the gflags library (https://code.google.com/p/gflags/)
# installed ("brew install gflags" or "sudo apt-get install libgflags-dev").
#
# The "release" target uses optimization flags. Make sure to get rid of
# previously built object files when switching between non-release and release
# builds: "make clean release"

SHELL = /bin/bash
CXX = clang++
CXXFLAGS = -std=c++11 -Wall -Werror -I. -I/usr/local/include -g
#OPTFLAGS = -O3 -march=native -DNDEBUG
OPTFLAGS = -O3 -march=native
LDFLAGS = -L/usr/local/lib -lgflags
TARGET = main

PROBLEMS = kitchen,rocksample,gripper
SOURCES = $(shell echo {.,${PROBLEMS}}/*.cc)
HEADERS = $(shell echo {.,${PROBLEMS}}/*.h)
OBJECTS = $(SOURCES:.cc=.o)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS) $(LDFLAGS)

release: CXXFLAGS += $(OPTFLAGS)
release: all

clean:
	-rm -f $(TARGET) $(OBJECTS)

%.o: %.cc $(HEADERS)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.PHONY : all release
