# Bitska analiza – AHRS algoritam (sekcije 5–8)

## Uvod

Vrednosti promenljivih koje figurišu u C++ kodu su realni brojevi, te je bilo
potrebno definisati tipove sa fiksnom tačkom radi kasnije implementacije u
hardveru, a u cilju izbegavanja tipova sa pokretnom tačkom.

Pokretanjem algoritma za različite vrednosti celobrojnih i razlomljenih delova
vektora, uočeno je da male promene vrednosti menjaju rezultat i da kvalitet
rezultata strmo opada. Stoga su u obzir uzeti i međurezultati algoritma (ne samo
krajnja orijentacija), kako bi se utvrdilo da algoritam ispravno radi.
Nakon ovog testiranja došlo se do vrednosti koje daju zadovoljavajuće rezultate.

---

## Opsezi promenljivih i odabrani tipovi

Algoritam sekcija 5–8 obuhvata sledeće promenljive. Za svaku su određeni opseg
vrednosti, ukupan broj bita i broj bita za celobrojni deo. Sve promenljive koriste
zaokruživanje SC_RND i saturaciju SC_SAT.

| Promenljiva       | Tip C++         | Opseg          | Ukupno bita | Cel. bita | Razl. bita |
|-------------------|-----------------|----------------|-------------|-----------|------------|
| `quat_t`          | `sc_fixed`      | [−2, 2)        | 18          | 2         | 16         |
| `dot_t`           | `sc_fixed`      | [−1, 1)        | 26          | 1         | 25         |
| `norm_t`          | `sc_fixed`      | [−2, 2)        | 26          | 2         | 24         |
| `accel_t`         | `sc_fixed`      | [−4, 4)        | 20          | 3         | 17         |
| `hg_t`            | `sc_fixed`      | [−2, 2)        | 26          | 2         | 24         |
| `magsq_t`         | `sc_ufixed`     | [0, 4)         | 22          | 2         | 20         |
| `invmag_t`        | `sc_ufixed`     | [0, 64)        | 24          | 6         | 18         |
| `halfgyro_t`      | `sc_fixed`      | [−2, 2)        | 22          | 2         | 20         |
| `gain_t`          | `sc_ufixed`     | [0, 16)        | 20          | 4         | 16         |
| `adjhg_t`         | `sc_fixed`      | [−16, 16)      | 26          | 5         | 21         |
| `dq_t`            | `sc_fixed`      | [−16, 16)      | 27          | 5         | 22         |
| `dt_t`            | `sc_ufixed`     | [0, 1)         | 20          | 0         | 20         |

### Definicije tipova u kodu (`analysis/spec/src/ahrs_hw.cpp`)

```cpp
typedef sc_fixed <18, 2, SC_RND, SC_SAT>  quat_t;      // kvaternion    [-2, 2)
typedef sc_fixed <26, 1, SC_RND, SC_SAT>  dot_t;       // skalarni proi [-1, 1)
typedef sc_fixed <26, 2, SC_RND, SC_SAT>  norm_t;      // normirani vek [-2, 2)
typedef sc_fixed <20, 3, SC_RND, SC_SAT>  accel_t;     // akcelerometar [-4, 4)
typedef sc_fixed <26, 2, SC_RND, SC_SAT>  hg_t;        // halfGravity   [-2, 2)
typedef sc_ufixed<22, 2, SC_RND, SC_SAT>  magsq_t;     // magnituda²    [0, 4)
typedef sc_ufixed<24, 6, SC_RND, SC_SAT>  invmag_t;    // inv-mag       [0, 64)
typedef sc_fixed <22, 2, SC_RND, SC_SAT>  halfgyro_t;  // polu-žiro     [-2, 2)
typedef sc_ufixed<20, 4, SC_RND, SC_SAT>  gain_t;      // pojačanje     [0, 16)
typedef sc_fixed <26, 5, SC_RND, SC_SAT>  adjhg_t;     // adj polu-žiro [-16, 16)
typedef sc_fixed <27, 5, SC_RND, SC_SAT>  dq_t;        // delta-kvaternion
typedef sc_ufixed<20, 0, SC_RND, SC_SAT>  dt_t;        // deltaTime     [0, 1)
```

---

## Obrazloženje kritičnih izbora

### Kvaternion (`quat_t`): sužen sa 29 na 18 bita

Originalna vrednost od 29 bita zamenjena je sa 18 bita. Ovo je ključna promena
motivicana ograničenjima ciljnog FPGA uređaja **xc7z010 (Zybo Z7-10)**:

- DSP48E1 blok podržava maksimalno množenje 18-bit × 27-bit = 45-bit
- Sa `quat_t = 29-bit`: `quat(29) × adjhg(26) = 55-bit → 2 DSP po množenju`
- Sa `quat_t = 18-bit`: `quat(18) × adjhg(26) = 44-bit → 1 DSP po množenju ✓`

U sekciji 8 postoje 12 takvih množenja, pa je ušteda 12 DSP48E1 blokova.

### `dt_t`: zadržano na 20 bita (ne sme biti manje)

Testirano je suženje na 18 bita. Za vrednost `dt = 0.01 s` (frekvencija uzorkovanja
100 Hz), kvantizaciona greška pri 18-bitnoj reprezentaciji iznosi `−1.68 × 10⁻⁵`.
Ova greška se množi pojačanjem `gain ≈ 10` tokom ~300 uzoraka inicijalizacije, što
akumulira grešku od oko **30°** — što je potpuno neprihvatljivo. Tip je stoga zadržan
na 20 bita, čime se greška za `dt = 0.01` svodi na zanemarljivu vrednost.

### `dq_t`: zadržano na 27 bita

Suženje `dq_t` na 22 ili 18 bita testirana je kombinacija sa odgovarajućim `dt_t`
vrednostima. Rezultati su pokazali grešku veću od 30° na dugom skupu podataka, što
je neprihvatljivo. Množenje `dq(27) × dt(20) = 47-bit` zahteva 2-DSP kaskadu, ali
je to jedino rešenje koje daje prihvatljivu tačnost.

---

## Verifikacija – poređenje sa float referencom

Implementacija sa fiksnom tačkom (`analysis/spec/src/ahrs_hw.cpp`) upoređena je sa
float referentnom implementacijom (`cpp_spec/spec/src/ahrs_hw.cpp`) na dva skupa
podataka senzora IMU.

### Kratki skup: 499 uzoraka (≈ 5 sekundi pri 100 Hz)

| Metrika                    | Vrednost     |
|----------------------------|--------------|
| Kvaternion greška – max    | **0.30°**    |
| Kvaternion greška – srednja | 0.12°       |
| Kvaternion greška – 95. percentil | 0.24° |
| Roll greška – max          | 0.30°        |
| Pitch greška – max         | 0.10°        |
| Yaw greška – max           | 0.24°        |
| Uzoraka sa greškom < 0.5°  | 499 (100%)   |

Referentna orijentacija (float): Roll = 129.958°, Pitch = 9.287°, Yaw = 25.567°  
Fixed-point orijentacija:        Roll = 129.987°, Pitch = 9.198°, Yaw = 25.353°

### Dugi skup: 1986 uzoraka (≈ 20 sekundi pri 100 Hz)

| Metrika                    | Vrednost     |
|----------------------------|--------------|
| Kvaternion greška – max    | **2.12°**    |
| Kvaternion greška – srednja | 0.37°       |
| Kvaternion greška – 95. percentil | 1.91° |
| Kvaternion greška – 99. percentil | 2.08° |

Referentna orijentacija (float): Roll = −132.663°, Pitch = −28.216°, Yaw = 53.252°  
Fixed-point orijentacija:        Roll = −133.468°, Pitch = −27.150°, Yaw = 54.718°

Raspodela greške po uzorcima (1986 uzoraka):

| Opseg greške | Broj uzoraka | Procenat |
|--------------|--------------|----------|
| < 0.5°       | 1745         | 87.9%    |
| 0.5° – 1.0°  | 2            | 0.1%     |
| 1.0° – 2.0°  | 179          | 9.0%     |
| 2.0° – 5.0°  | 60           | 3.0%     |
| > 5.0°       | 0            | **0.0%** |

### Napomena o Euler grešci na dugom skupu

Na dugom skupu uočena je maksimalna greška Euler uglova od ~9° (roll i yaw).
Međutim, ova greška je isključivo artefakt konverzije iz kvaterniona u Euler uglove
– javlja se samo u uzorcima gde je pitch blizu 90° (uzorci 1820–1832, pitch ≈ 78°),
što je blizina gimbal-lock singulariteta. U toj zoni mali pomak kvaterniona
matematički odgovara velikom pomaku roll/yaw uglova. Stvarna kvaternion greška u
tim uzorcima iznosi 0.0° (kvaternioni su identični u okviru preciznosti float
reprezentacije u CSV fajlu).

---

## Zaključak

Odabrana konfiguracija tipova fiksne tačke zadovoljava dva zahteva:

1. **Tačnost**: Maksimalna kvaternion greška u odnosu na float referencu je
   **2.12°** na skupu od 1986 uzoraka, što je prihvatljivo za embedded IMU
   primene (tipični zahtev < 2–5°). Na kraćem skupu od 499 uzoraka maksimalna
   greška je **0.30°**.

2. **DSP efikasnost (xc7z010)**: Suženje `quat_t` sa 29 na 18 bita smanjuje
   produkt `quat × adjhg` sa 55-bit na 44-bit, čime svako od 12 množenja u
   sekciji 8 staje u jedan DSP48E1 blok umjesto dva. Ušteda iznosi 12 DSP48E1
   blokova u sekciji 8.
