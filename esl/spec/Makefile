# Makefile for AHRS project with HW/SW partitioning
#
# Structure:
#   src/main.c    - pipeline + SW sections (1-4, 9-11)
#   src/ahrs_hw.c - HW sections (5-8)

CC = gcc
CFLAGS_DEBUG = -Wall -Wextra -std=c99 -O0 -g
CFLAGS_RELEASE = -Wall -Wextra -std=c99 -O2
LDFLAGS = -lm

# Source directory
SRCDIR = src
INCDIR = include

# Data file to process (change this to use different sensor data)
DATA_FILE = sensor_data_short.csv

all: csv_to_bin ahrs_pipeline

# Main pipeline (production)
ahrs_pipeline: $(SRCDIR)/main.c $(SRCDIR)/ahrs_hw.c $(INCDIR)/ahrs_hw.h
	$(CC) $(CFLAGS_RELEASE) -I$(SRCDIR) -I$(INCDIR) -o $@ $(SRCDIR)/main.c $(SRCDIR)/ahrs_hw.c $(LDFLAGS)
	@echo "✓ ahrs_pipeline"

# Real-time profiling version (no memory overhead)
ahrs_profile_rt: $(SRCDIR)/main_profile_realtime.c $(SRCDIR)/ahrs_hw.c $(INCDIR)/ahrs_hw.h
	$(CC) $(CFLAGS_DEBUG) -I$(SRCDIR) -I$(INCDIR) -o $@ $(SRCDIR)/main_profile_realtime.c $(SRCDIR)/ahrs_hw.c $(LDFLAGS)
	@echo "✓ ahrs_profile_rt (real-time simulation)"

# Embedded-data version (no file I/O at run-time, clean profiling target)
ahrs_embedded: $(SRCDIR)/main_embedded.c $(SRCDIR)/ahrs_hw.c $(INCDIR)/ahrs_hw.h $(INCDIR)/sensor_data_embedded.h
	$(CC) $(CFLAGS_RELEASE) -I$(SRCDIR) -I$(INCDIR) -o $@ $(SRCDIR)/main_embedded.c $(SRCDIR)/ahrs_hw.c $(LDFLAGS)
	@echo "✓ ahrs_embedded (compile-time sensor data, zero file I/O)"

# Embedded-data version with debug symbols for profiling tools (gprof / valgrind)
ahrs_embedded_profile: $(SRCDIR)/main_embedded.c $(SRCDIR)/ahrs_hw.c $(INCDIR)/ahrs_hw.h $(INCDIR)/sensor_data_embedded.h
	$(CC) $(CFLAGS_DEBUG) -pg -I$(SRCDIR) -I$(INCDIR) -o $@ $(SRCDIR)/main_embedded.c $(SRCDIR)/ahrs_hw.c $(LDFLAGS)
	@echo "✓ ahrs_embedded_profile (debug + gprof instrumentation)"

# CSV converter
csv_to_bin: $(SRCDIR)/csv_to_bin.c
	$(CC) $(CFLAGS_RELEASE) -o $@ $<
	@echo "✓ csv_to_bin"

# Test data generator
generate_test_data: $(SRCDIR)/generate_test_data.c
	$(CC) $(CFLAGS_RELEASE) -o $@ $< $(LDFLAGS)
	@echo "✓ generate_test_data"

# ============================================================================
# WORKFLOW
# ============================================================================

run: csv_to_bin ahrs_pipeline
	@echo ""
	@echo "=== CSV -> BIN ==="
	./csv_to_bin ../data/$(DATA_FILE) sensor_data.bin
	@echo ""
	@echo "=== AHRS processing ==="
	./ahrs_pipeline sensor_data.bin orientation_data.csv
	@echo ""
	@echo "=== Visualization ==="
	python3 $(SRCDIR)/visualize_motion.py orientation_data.csv
	@echo "Output files: ../data/euler_angles.png, ../data/orientation_trajectory.png, ../data/motion_animation.gif"

profile-rt: ahrs_profile_rt
	@echo ""
	@echo "=== Callgrind REAL-TIME (no memory overhead) ==="
	valgrind --tool=callgrind --callgrind-out-file=callgrind.out ./ahrs_profile_rt
	@echo ""
	@echo "Open: kcachegrind callgrind.out"

clean:
	rm -f csv_to_bin ahrs_pipeline ahrs_profile_rt generate_test_data
	rm -f sensor_data.bin orientation_data.csv
	rm -f callgrind.out*

.PHONY: all run profile-rt clean
