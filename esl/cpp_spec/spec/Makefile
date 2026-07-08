# ============================================================================
# Makefile for AHRS Project - C++ Version
# ============================================================================

# Directories
SRC_DIR = src
INC_DIR = include

# Compiler
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++14 -O2 -I$(SRC_DIR) -I$(INC_DIR)
LDFLAGS = -lm

# ============================================================================
# Targets
# ============================================================================

# Main pipeline (main.cpp)
TARGET_MAIN = ahrs_pipeline
SOURCES_MAIN = $(SRC_DIR)/main.cpp $(SRC_DIR)/ahrs_hw.cpp

# Profile realtime (main_profile_realtime.cpp)
TARGET_PROFILE = main_profile_realtime
SOURCES_PROFILE = $(SRC_DIR)/main_profile_realtime.cpp $(SRC_DIR)/ahrs_hw.cpp

# CSV to binary converter
TARGET_CSV = csv_to_bin
SOURCES_CSV = $(SRC_DIR)/csv_to_bin.cpp

# ============================================================================
# Rules
# ============================================================================

.PHONY: all clean run help

# Default: build all programs
all: $(TARGET_MAIN) $(TARGET_PROFILE) $(TARGET_CSV)

# Build specific targets
$(TARGET_MAIN): $(SOURCES_MAIN)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(TARGET_PROFILE): $(SOURCES_PROFILE)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(TARGET_CSV): $(SOURCES_CSV)
	$(CXX) $(CXXFLAGS) $^ -o $@

# Run main pipeline
run: $(TARGET_MAIN)
	./$(TARGET_MAIN)

# Run profile version
run-profile: $(TARGET_PROFILE)
	./$(TARGET_PROFILE)

# Clean build files
clean:
	rm -f $(TARGET_MAIN) $(TARGET_PROFILE) $(TARGET_CSV)
	rm -f *.o
	rm -f sensor_data.bin sensor_data.csv orientation_data.csv

# Help
help:
	@echo "Available targets:"
	@echo "  make             - Build all programs"
	@echo "  make run         - Build and run main pipeline"
	@echo "  make run-profile - Build and run profiling version"
	@echo "  make clean       - Remove executables"