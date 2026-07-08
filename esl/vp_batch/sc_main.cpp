#include "defines.hpp"
#include "sensor.hpp"
#include "spi.hpp"
#include "cpu.hpp"
#include "ddr.hpp"
#include "interconnect.hpp"
#include "bram.hpp"
#include "ahrs_ip.hpp"

// Global event definitions (declared extern in defines.hpp)
sc_core::sc_event ahrs_done;
sc_core::sc_event sensor_ready;

int sc_main(int argc, char* argv[])
{
    std::cout << "\n";
    std::cout << "================================================" << std::endl;
    std::cout << "   AHRS Virtual Platform - SystemC/TLM-2.0"      << std::endl;
    std::cout << "   Batch size : " << HW_BATCH_SIZE << " samples" << std::endl;
    std::cout << "   Sample rate: " << SAMPLE_RATE_HZ << " Hz"     << std::endl;
    std::cout << "   Max batches: " << MAX_BATCHES                  << std::endl;
    std::cout << "================================================\n" << std::endl;

    // ── Instantiate modules ──────────────────────────────────────────────────
    std::cout << "[SC_MAIN] Creating modules..." << std::endl;

    const char *csv_path = "../analysis/data/sensor_data_short.csv";

    Sensor      sensor("sensor", csv_path, MAX_BATCHES * HW_BATCH_SIZE);
    SPI         spi("spi");
    CPU         cpu("cpu");
    DDR         ddr("ddr");
    Interconnect interconnect("interconnect");
    Bram        bram("bram");
    AHRS_IP     ahrs_ip("ahrs_ip");

    // ── Bind sockets (matches block diagram) ────────────────────────────────
    std::cout << "\n[SC_MAIN] Binding sockets..." << std::endl;

    // SPI ← reads from → Sensor
    spi.isock_sensor.bind(sensor.tsock);

    // Interconnect → reads raw data from → SPI (tsock_interconnect)
    interconnect.isock_spi.bind(spi.tsock_interconnect);

    // CPU ↔ Interconnect (control & BRAM access)
    cpu.isock_interconnect.bind(interconnect.tsock_cpu);

    // CPU ↔ DDR (direct, private memory)
    cpu.isock_ddr.bind(ddr.tsock);

    // Interconnect → BRAM Port A  (CPU/SPI data path)
    interconnect.isock_bram.bind(bram.tsock_a);

    // Interconnect → AHRS IP      (control registers)
    interconnect.isock_ip.bind(ahrs_ip.tsock_interconnect);

    // AHRS IP → BRAM Port B       (direct data access, bypasses Interconnect)
    ahrs_ip.isock_bram.bind(bram.tsock_b);

    std::cout << "\n[SC_MAIN] Connection summary:" << std::endl;
    std::cout << "  Sensor      ← SPI (isock_sensor)" << std::endl;
    std::cout << "  Interconnect → SPI (isock_spi)" << std::endl;
    std::cout << "  CPU         → Interconnect (tsock_cpu)" << std::endl;
    std::cout << "  CPU         ↔ DDR (direct)" << std::endl;
    std::cout << "  Interconnect → BRAM Port A" << std::endl;
    std::cout << "  Interconnect → AHRS IP" << std::endl;
    std::cout << "  AHRS IP     → BRAM Port B (direct)" << std::endl;
    std::cout << "  AHRS IP     → CPU (ahrs_done interrupt / sc_event)" << std::endl;

    std::cout << "\n[SC_MAIN] Starting simulation...\n" << std::endl;

    sc_core::sc_start();

    std::cout << "\n[SC_MAIN] Simulation finished at "
              << sc_core::sc_time_stamp() << std::endl;

    // ── Dump normalized output quaternions to CSV for comparison ─────────────
    ddr.dump_csv("vp_output.csv", MAX_BATCHES * HW_BATCH_SIZE);

    std::cout << "================================================\n" << std::endl;

    return 0;
}
