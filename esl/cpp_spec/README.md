# cpp_spec/ — C++ Floating-Point Reference Implementation

C++ verzija AHRS algoritma sa istom HW/SW particijom kao C99 `spec/`.  
Služi kao **referenca za poređenje** sa fixed-point implementacijom — pošto C++ `float` nema razlike od C99 `float`, rezultati sa `spec/` trebaju biti identični.

---

## Struktura

```
cpp_spec/
├── spec/
│   ├── Makefile
│   ├── include/
│   │   └── ahrs_hw.h              # Isti interfejs kao u spec/
│   └── src/
│       ├── main.cpp               # Produkcija: čita .bin, piše orientation CSV
│       ├── main_profile_realtime.cpp  # Profajliranje sa simuliranim podacima
│       ├── ahrs_hw.cpp            # HW akcelerator (Sekcije 5-8)
│       ├── csv_to_bin.cpp         # Konvertor CSV → binarni format
│       └── visualize_motion.py   # 3D vizualizacija (ista skripta kao u spec/)
└── data/
    ├── sensor_data_full.csv       # Ulazni senzorski podaci (isti kao data/sensor_data_full.csv)
    └── sensor_data_short.csv      # Kraći skup za testiranje
```

---

## Build i pokretanje

```bash
cd spec
make

# Konvertuj CSV u binarni
./csv_to_bin ../data/sensor_data_full.csv sensor_data.bin

# Pokreni
./ahrs_pipeline sensor_data.bin orientation_out.csv
```

---

## Napomena

Generisani fajlovi (`ref_full.csv`, `ref_short.csv`, `float_sensor_*_results.txt`, binarni) nisu praćeni u git-u — generišu se lokalnim pokretanjem.

Referentni ulazni podaci se nalaze u [`../../data/sensor_data_full.csv`](../../data/sensor_data_full.csv).
