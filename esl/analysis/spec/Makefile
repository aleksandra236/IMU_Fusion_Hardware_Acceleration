# ============================================================================
# Makefile for AHRS Project - C++ Version with SystemC
# ============================================================================

# -------------------------
# Paths / directories
# -------------------------
SRC_DIR     = src
INCLUDE_DIR = include

# -------------------------
# SystemC Configuration (use /usr/local version!)
# -------------------------
SYSTEMC_INC = /usr/local/systemc/include
SYSTEMC_LIB = /usr/local/systemc/lib/libsystemc.so

# -------------------------
# Compiler flags
# -------------------------
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -O2 \
           -I$(SYSTEMC_INC) -I$(INCLUDE_DIR) \
           -DSC_INCLUDE_FX

# Link flags: link the absolute .so to avoid picking /usr/lib version
LDFLAGS  = $(SYSTEMC_LIB) -lm -Wl,-rpath,/usr/local/systemc/lib

# ============================================================================
# Targets
# ============================================================================

# Main pipeline (main.cpp)
TARGET_MAIN   = ahrs_pipeline
SOURCES_MAIN  = $(SRC_DIR)/main.cpp $(SRC_DIR)/ahrs_hw.cpp

# Profile realtime (main_profile_realtime.cpp)
TARGET_PROFILE  = main_profile_realtime
SOURCES_PROFILE = $(SRC_DIR)/main_profile_realtime.cpp $(SRC_DIR)/ahrs_hw.cpp

# Continuous processing (main_continuous.cpp)
TARGET_CONTINUOUS  = main_continuous
SOURCES_CONTINUOUS = $(SRC_DIR)/main_continuous.cpp $(SRC_DIR)/ahrs_hw.cpp

# Generate test data
TARGET_GEN  = generate_test_data
SOURCES_GEN = $(SRC_DIR)/generate_test_data.cpp

# CSV to binary converter
TARGET_CSV  = csv_to_bin
SOURCES_CSV = $(SRC_DIR)/csv_to_bin.cpp

# ============================================================================
# Rules
# ============================================================================

.PHONY: all clean run help test run-profile run-continuous

all: $(TARGET_MAIN) $(TARGET_PROFILE) $(TARGET_CONTINUOUS) $(TARGET_GEN) $(TARGET_CSV)

$(TARGET_MAIN): $(SOURCES_MAIN)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(TARGET_PROFILE): $(SOURCES_PROFILE)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(TARGET_CONTINUOUS): $(SOURCES_CONTINUOUS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(TARGET_GEN): $(SOURCES_GEN)
	$(CXX) $(CXXFLAGS) $^ -o $@ -lm

$(TARGET_CSV): $(SOURCES_CSV)
	$(CXX) $(CXXFLAGS) $^ -o $@

run: $(TARGET_MAIN)
	./$(TARGET_MAIN)

run-profile: $(TARGET_PROFILE)
	./$(TARGET_PROFILE)

run-continuous: $(TARGET_CONTINUOUS)
	./$(TARGET_CONTINUOUS)

test: $(TARGET_GEN) $(TARGET_MAIN)
	./$(TARGET_GEN) 5000
	./$(TARGET_MAIN) sensor_data.bin orientation_data.csv

clean:
	rm -f $(TARGET_MAIN) $(TARGET_PROFILE) $(TARGET_CONTINUOUS) $(TARGET_GEN) $(TARGET_CSV)
	rm -f *.o

help:
	@echo "Available targets:"
	@echo "  make                 - Build all programs"
	@echo "  make run             - Build and run main pipeline"
	@echo "  make run-profile     - Build and run profiling version"
	@echo "  make run-continuous  - Build and run continuous version"
	@echo "  make test            - Generate data and run main pipeline"
	@echo "  make clean           - Remove executables"
