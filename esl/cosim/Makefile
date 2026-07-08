# Makefile - mesana simulacija (SystemC VP + VHDL ahrs_sec678) u Xcelium 19.03

HDL_DIR    ?= ../../rtl/hdl
VHDL_SRCS  = $(HDL_DIR)/ahrs_sec678.vhd

SC_SRCS = sc_main.cpp ahrs_ip.cpp cpu.cpp bram.cpp spi.cpp \
          sensor.cpp ddr.cpp interconnect.cpp

CXX_FLAGS = -std=c++17 -DCOSIM -DSC_INCLUDE_FX -DSC_INCLUDE_DYNAMIC_PROCESSES

.PHONY: all gui batch clean help

all: gui

gui:
xmsc_run -64bit -sc_main -gui -access +rwc \
    -sysc-cflags "$(CXX_FLAGS)" $(SC_SRCS) $(VHDL_SRCS)

batch:
xmsc_run -64bit -sc_main -access +rwc \
    -sysc-cflags "$(CXX_FLAGS)" $(SC_SRCS) $(VHDL_SRCS)

sc_ref:
xmsc_run -64bit -sc_main -gui -access +rwc \
    -sysc-cflags "-std=c++17 -DSC_INCLUDE_FX -DSC_INCLUDE_DYNAMIC_PROCESSES" \
    $(SC_SRCS)

clean:
rm -rf xcelium.d xrun.log xrun.history xmsim.history xmelab.history \
       *.shm *.diag *.log INCA_libs ncsim.log waves.shm \
       .simvision sst_dump.txt cov_work

help:
@echo "  make gui     - build i pokreni u SimVision GUI (COSIM)"
@echo "  make batch   - build i pokreni bez GUI (COSIM)"
@echo "  make sc_ref  - pure-SC referenca (bez HDL)"
@echo "  make clean   - obrisi generisane fajlove"
