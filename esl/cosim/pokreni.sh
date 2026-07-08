
#!/bin/bash

xmsc_run -64bit -sc_main -gui -access +rwc -DCOSIM -DSC_INCLUDE_FX -DSC_INCLUDE_DYNAMIC_PROCESSES -Wcxx,-std=c++17 -v93 sc_main.cpp ahrs_ip.cpp cpu.cpp bram.cpp spi.cpp sensor.cpp ddr.cpp interconnect.cpp ../../rtl/hdl/ahrs_sec678.vhd

