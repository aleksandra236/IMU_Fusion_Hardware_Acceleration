#	AHRS Virtual Platform Makefile
#	Usage:
#	  make       - compiles the virtual platform
#	  make run   - runs the simulation
#	  make clean - removes all binaries and objects

CC = g++
SYSTEMC_HOME = /usr/local/systemc


# Compiler flags
CFLAGS = -std=c++17 -Wall -Wextra -DSC_INCLUDE_FX

INCLUDES = -I$(SYSTEMC_HOME)/include -I.
LDFLAGS = -L$(SYSTEMC_HOME)/lib -Wl,-rpath,$(SYSTEMC_HOME)/lib
LIBS = -lsystemc -lpthread -lm

# Source files (exclude standalone tools and HLS file)
SOURCE_CPP = $(filter-out compare.cpp full_compare.cpp ahrs_ip_hls.cpp, $(wildcard *.cpp))
SOURCE_HPP = $(wildcard *.hpp)

# Target executable
TARGET = ahrs_vp

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SOURCE_CPP) $(SOURCE_HPP)
	@echo "Compiling AHRS Virtual Platform..."
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(SOURCE_CPP) $(LDFLAGS) $(LIBS)
	@echo "Build successful: $(TARGET)"

run: $(TARGET)
	@echo "Running AHRS Virtual Platform..."
	@echo "================================"
	./$(TARGET)

clean:
	@echo "Cleaning up..."
	rm -f $(TARGET) compare full_compare *.o
	@echo "Clean complete."

# Standalone comparison tool: float reference vs VP sc_fixed output
# Usage:
#   First run the VP:    make run          (produces vp_output.csv)
#   Then compare:        make compare && ./compare
compare: compare.cpp
	@echo "Building comparison tool..."
	$(CC) -std=c++17 -O2 compare.cpp -o compare -lm
	@echo "Done. Run './compare' after './ahrs_vp' to see the comparison."

# Help target
help:
	@echo "AHRS Virtual Platform Build System"
	@echo "==================================="
	@echo "Targets:"
	@echo "  make       - Build the virtual platform"
	@echo "  make run   - Build and run simulation"
	@echo "  make clean - Remove build artifacts"
	@echo ""
	@echo "SystemC: $(SYSTEMC_HOME)"