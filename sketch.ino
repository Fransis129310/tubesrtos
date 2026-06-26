/* ============================================================================
 *  VENDING MACHINE - ESP32 + FreeRTOS        (v2.1 - + INSTRUMENTASI PENGUKURAN)
 *  Komponen : Keypad 4x4, LCD I2C 16x2, RFID RC522, Servo, Push Button (ISR)
 *
 *  >>> Versi ini = kode asli Anda + blok pengukuran metrik real-time.
 *  >>> Semua tambahan diberi tanda komentar  // [UKUR]
 *  >>> Saat Run di Wokwi, Serial Monitor mencetak ringkasan tiap 5 detik
 *      yang langsung bisa disalin ke Tabel 6.2 laporan.
 * ========================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <MFRC522.h>

/* ============================================================================
 * [FIX] SEMUA DEFINISI TIPE DITARUH DI ATAS (sebelum fungsi apa pun)
 *  Ini mencegah error "State_t was not declared" akibat auto-prototype Arduino.
 * ========================================================================== */
typedef enum { EV_KEY, EV_RFID_OK, EV_RFID_SALAH } JenisEvent_t;
typedef struct {
  JenisEvent_t jenis;
  char         tombol;
} Event_t;

typedef enum {
  ST_STANDBY, ST_PILIH_KODE, ST_KODE_SALAH, ST_TANYA_LAGI,
  ST_PEMBAYARAN, ST_KARTU_SALAH, ST_KONFIRMASI, ST_SERVO_BUKA, ST_BERJAGA,
  ST_ADMIN_MENU, ST_EDIT_KODE, ST_EDIT_HARGA, ST_EDIT_SELESAI,
  ST_DEBUG_MENU, ST_DEBUG_JALAN
} State_t;

/* ---------------------------- KONFIGURASI PIN ----------------------------- */
#define PIN_SERVO    16
#define PIN_RFID_SS  5
#define PIN_RFID_RST 17
#define PIN_TOMBOL   35      // push button admin (input-only, sumber INTERRUPT)
// SPI RFID otomatis: SCK=18, MISO=19, MOSI=23 (VSPI default)
// LCD I2C otomatis : SDA=21, SCL=22

const byte JUMLAH_BARIS = 4, JUMLAH_KOLOM = 4;
char petaTombol[JUMLAH_BARIS][JUMLAH_KOLOM] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte pinBaris[JUMLAH_BARIS] = { 32, 33, 25, 26 };
byte pinKolom[JUMLAH_KOLOM] = { 27, 14, 12, 13 };

/* ------------------------- PARAMETER FLEKSIBEL ---------------------------- */
const uint8_t  KODE_MINIMAL              = 1;
const uint8_t  KODE_MAKSIMAL             = 25;
const uint8_t  MAKS_DIGIT_KODE           = 2;
const uint8_t  MAKS_DIGIT_HARGA          = 6;

const uint32_t JEDA_PERINGATAN_KARTU_MS  = 5000;
const uint32_t JEDA_KONFIRMASI_BAYAR_MS  = 10000;
const uint32_t JEDA_SERVO_TERBUKA_MS     = 5000;
const uint32_t JEDA_BERJAGA_MS           = 30000;
const uint32_t JEDA_PESAN_SINGKAT_MS     = 2000;

const int      SUDUT_SERVO_TUTUP         = 0;
const int      SUDUT_SERVO_BUKA          = 90;

const uint32_t INTERVAL_GULIR_MS         = 350;
const uint8_t  LEBAR_LCD                 = 16;

const uint8_t  KAPASITAS_BARANG          = 30;

/* ----------------------- PARAMETER DEMO RACE / PIP ------------------------ */
const uint32_t RACE_ITERASI              = 20000;
const uint32_t PIP_KERJA_RENDAH_MS       = 1500;
const uint32_t PIP_GANGGU_MS             = 3000;

/* ============================================================================
 * [UKUR] BLOK INSTRUMENTASI PENGUKURAN METRIK REAL-TIME
 * ----------------------------------------------------------------------------
 *  Cara pakai sudah otomatis. Saat Run:
 *   - tiap task mencatat waktu eksekusi (exec) per iterasi
 *   - disimpan: WCET (max), rata-rata, jumlah deadline miss
 *   - Task_Monitor mencetak ringkasan tiap PERIODE_LAPOR_MS
 *  Salin angka dari Serial ke Tabel 6.2 laporan.
 * ========================================================================== */
const uint32_t PERIODE_LAPOR_MS = 5000;     // [UKUR] tiap 5 detik cetak ringkasan

typedef struct {
  const char*  nama;
  uint32_t     stackTotalB;   // ukuran stack saat xTaskCreate (byte)
  uint32_t     deadlineUs;    // deadline (= periode) dalam mikrodetik
  uint32_t     wcetUs;        // WCET terukur (maksimum exec)
  uint32_t     minUs;         // [UKUR] exec tercepat (untuk jitter = max - min)
  uint64_t     sumUs;         // akumulasi exec utk rata-rata
  uint32_t     iter;          // jumlah iterasi
  uint32_t     miss;          // jumlah deadline miss
} StatTask_t;

// Deadline (us) = periode tiap task: Input 30ms, Logic 20ms, Display 60ms
// Field minUs diisi 0xFFFFFFFF (UINT32_MAX) agar pengukuran pertama langsung menjadi minimum.
StatTask_t stInput   = { "Input",   3072, 30000, 0, 0xFFFFFFFF, 0, 0, 0 };
StatTask_t stLogic   = { "Logic",   6144, 20000, 0, 0xFFFFFFFF, 0, 0, 0 };
StatTask_t stDisplay = { "Display", 3072, 60000, 0, 0xFFFFFFFF, 0, 0, 0 };

static inline uint32_t ukurMulai() { return micros(); }   // [UKUR]

// [UKUR] Kapan pengukuran AKTIF:
//  - ukurAktif=false saat mode admin/debug (task Race/PIP mengganggu timing)
//  - lewati WARMUP_MS pertama agar boot/inisialisasi tidak mengotori data
volatile bool  ukurAktif   = true;
const uint32_t WARMUP_MS    = 2000;   // 2 detik pertama diabaikan

// [UKUR] catat hasil 1 iterasi task (hanya saat operasi normal)
static inline void ukurSelesai(StatTask_t &s, uint32_t t0) {
  if (!ukurAktif)            return;          // sedang demo admin/debug -> abaikan
  if (millis() < WARMUP_MS)  return;          // masih warm-up -> abaikan
  uint32_t exec = micros() - t0;
  s.iter++;
  s.sumUs += exec;
  if (exec > s.wcetUs)     s.wcetUs = exec;
  if (exec < s.minUs)      s.minUs = exec;   // [UKUR] catat exec tercepat utk jitter
  if (exec > s.deadlineUs) s.miss++;
}

/* ------------------------- DAFTAR HARGA MINUMAN --------------------------- */
typedef struct {
  uint8_t  kode;
  uint32_t harga;
} Minuman_t;

Minuman_t daftarMinuman[KAPASITAS_BARANG] = {
  {  1,  5000 }, {  2,  7000 }, {  3,  8000 }, {  4, 10000 }, {  5, 12000 },
  {  6,  6500 }, {  7,  9000 }, {  8, 15000 }, {  9,  5500 }, { 10, 11000 },
};
uint8_t jumlahMinuman = 10;

Minuman_t snapMinuman[KAPASITAS_BARANG];
uint8_t   snapJumlah = 0;

/* --------------------- JENIS KARTU NFC YANG DITERIMA ---------------------- */
const MFRC522::PICC_Type TIPE_KARTU_DITERIMA[] = {
  MFRC522::PICC_TYPE_MIFARE_UL,
};
const uint8_t JUMLAH_TIPE_DITERIMA =
  sizeof(TIPE_KARTU_DITERIMA) / sizeof(TIPE_KARTU_DITERIMA[0]);

/* ------------------------------- PERIPHERAL ------------------------------- */
Keypad keypad = Keypad(makeKeymap(petaTombol), pinBaris, pinKolom,
                       JUMLAH_BARIS, JUMLAH_KOLOM);
LiquidCrystal_I2C lcd(0x27, LEBAR_LCD, 2);
Servo servoPintu;
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);

/* ============================ OBJEK RTOS ================================= */
QueueHandle_t     qEvent;
SemaphoreHandle_t mtxData;
SemaphoreHandle_t mtxLCD;
SemaphoreHandle_t semTombol;

TaskHandle_t hInput, hLogic, hDisplay;
TaskHandle_t hMonitor;                 // [UKUR] handle task monitor

#define KUNCI_DATA()    xSemaphoreTake(mtxData, portMAX_DELAY)
#define BUKA_DATA()     xSemaphoreGive(mtxData)

/* ------------------------------- EVENT INPUT ------------------------------ */
/* (definisi Event_t & State_t sudah dipindah ke atas) */

State_t  stateSekarang   = ST_STANDBY;
uint32_t waktuMasukState = 0;

char     inputKode[MAKS_DIGIT_KODE + 1]   = "";
char     inputHarga[MAKS_DIGIT_HARGA + 1] = "";
uint8_t  jmlDigit        = 0;
uint32_t totalBelanja    = 0;

uint8_t  editKode        = 0;
bool     editBarangBaru  = false;

/* ----------------------- VARIABEL TAMPILAN (LCD) -------------------------- */
char     lcdBaris0[40] = "Selamat Datang";
char     lcdBaris1[40] = "";
bool     baris0Gulir   = false;

/* ============================ FUNGSI BANTU LCD =========================== */
char cacheBaris[2][17] = { "", "" };

void lcdTulis(uint8_t baris, const char* teks) {
  char buf[LEBAR_LCD + 1];
  uint8_t i = 0;
  for (; i < LEBAR_LCD && teks[i] != '\0'; i++) buf[i] = teks[i];
  for (; i < LEBAR_LCD; i++) buf[i] = ' ';
  buf[LEBAR_LCD] = '\0';
  if (strcmp(buf, cacheBaris[baris]) == 0) return;
  strcpy(cacheBaris[baris], buf);
  lcd.setCursor(0, baris);
  lcd.print(buf);
}

void setTampilan(const char* b0, const char* b1, bool gulir0) {
  xSemaphoreTake(mtxLCD, portMAX_DELAY);
  strncpy(lcdBaris0, b0, sizeof(lcdBaris0) - 1); lcdBaris0[sizeof(lcdBaris0)-1] = '\0';
  strncpy(lcdBaris1, b1, sizeof(lcdBaris1) - 1); lcdBaris1[sizeof(lcdBaris1)-1] = '\0';
  baris0Gulir = gulir0;
  xSemaphoreGive(mtxLCD);
}
void tampil(const char* b0, const char* b1, bool gulir = false) {
  setTampilan(b0, b1, gulir);
}

/* ====================== FUNGSI BANTU DATA BARANG ========================= */
int cariIndex(uint8_t kode) {
  for (uint8_t i = 0; i < jumlahMinuman; i++)
    if (daftarMinuman[i].kode == kode) return i;
  return -1;
}

bool cariHarga(uint8_t kode, uint32_t* hargaKeluar) {
  bool ada = false;
  KUNCI_DATA();
  int idx = cariIndex(kode);
  if (idx >= 0) { *hargaKeluar = daftarMinuman[idx].harga; ada = true; }
  BUKA_DATA();
  return ada;
}

bool simpanBarang(uint8_t kode, uint32_t harga) {
  bool ok = false;
  KUNCI_DATA();
  int idx = cariIndex(kode);
  if (idx >= 0) {
    daftarMinuman[idx].harga = harga; ok = true;
  } else if (jumlahMinuman < KAPASITAS_BARANG) {
    daftarMinuman[jumlahMinuman].kode  = kode;
    daftarMinuman[jumlahMinuman].harga = harga;
    jumlahMinuman++; ok = true;
  }
  BUKA_DATA();
  return ok;
}

void dumpDaftarBarang(const char* judul) {
  KUNCI_DATA();
  Serial.printf("\n--- %s (%u barang) ---\n", judul, jumlahMinuman);
  for (uint8_t i = 0; i < jumlahMinuman; i++)
    Serial.printf("  [%2u] kode=%-2u  harga=Rp.%lu\n",
                  i, daftarMinuman[i].kode, (unsigned long)daftarMinuman[i].harga);
  BUKA_DATA();
}

/* ============================ VALIDASI KARTU ============================= */
bool apakahKartuNFC() {
  MFRC522::PICC_Type tipe = rfid.PICC_GetType(rfid.uid.sak);
  Serial.print("Tipe kartu terbaca: ");
  Serial.println(rfid.PICC_GetTypeName(tipe));
  for (uint8_t i = 0; i < JUMLAH_TIPE_DITERIMA; i++)
    if (tipe == TIPE_KARTU_DITERIMA[i]) return true;
  return false;
}

/* ================================= ISR ================================== */
void IRAM_ATTR isrTombol() {
  BaseType_t hpw = pdFALSE;
  xSemaphoreGiveFromISR(semTombol, &hpw);
  portYIELD_FROM_ISR(hpw);
}

/* ======================================================================== *
 *  DEMO RACE CONDITION
 * ======================================================================== */
volatile uint32_t raceCounter = 0;
SemaphoreHandle_t mtxRace;
volatile uint8_t  raceSelesai = 0;
volatile bool     racePakaiMutex = false;

void taskRacePekerja(void* pv) {
  for (uint32_t i = 0; i < RACE_ITERASI; i++) {
    if (racePakaiMutex) xSemaphoreTake(mtxRace, portMAX_DELAY);
    uint32_t tmp = raceCounter;
    tmp = tmp + 1;
    asm volatile("nop");
    raceCounter = tmp;
    if (racePakaiMutex) xSemaphoreGive(mtxRace);
  }
  raceSelesai++;
  vTaskDelete(NULL);
}

void jalankanDemoRace(bool pakaiMutex) {
  racePakaiMutex = pakaiMutex;
  raceCounter = 0; raceSelesai = 0;

  Serial.printf("\n===== DEMO RACE CONDITION (%s mutex) =====\n",
                pakaiMutex ? "DENGAN" : "TANPA");
  Serial.printf("2 task x %lu increment. Nilai benar = %lu\n",
                (unsigned long)RACE_ITERASI, (unsigned long)(2 * RACE_ITERASI));

  xTaskCreatePinnedToCore(taskRacePekerja, "race1", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskRacePekerja, "race2", 2048, NULL, 2, NULL, 1);

  while (raceSelesai < 2) vTaskDelay(pdMS_TO_TICKS(10));

  uint32_t hilang = (2 * RACE_ITERASI) - raceCounter;
  Serial.printf("HASIL  : %lu  (selisih hilang = %lu)\n",
                (unsigned long)raceCounter, (unsigned long)hilang);
  Serial.println(pakaiMutex
    ? ">> Konsisten. Mutex mencegah lost update."
    : ">> MELESET! Inilah race condition (akses tak terlindungi).");
}

/* ======================================================================== *
 *  DEMO PRIORITY INVERSION -> PIP
 * ======================================================================== */
SemaphoreHandle_t lockPIP_sem;
SemaphoreHandle_t lockPIP_mtx;
volatile uint8_t  pipPakaiMutex = 0;
volatile uint8_t  pipFaseRendahPegang = 0;
volatile uint8_t  pipSelesai = 0;
volatile uint32_t pipWaktuTunggu = 0;

void ambilLockPIP() {
  if (pipPakaiMutex) xSemaphoreTake(lockPIP_mtx, portMAX_DELAY);
  else               xSemaphoreTake(lockPIP_sem, portMAX_DELAY);
}
void lepasLockPIP() {
  if (pipPakaiMutex) xSemaphoreGive(lockPIP_mtx);
  else               xSemaphoreGive(lockPIP_sem);
}

void taskPIP_Rendah(void* pv) {
  ambilLockPIP();
  pipFaseRendahPegang = 1;
  uint32_t t0 = millis();
  while (millis() - t0 < PIP_KERJA_RENDAH_MS) { volatile int x=0; x++; }
  lepasLockPIP();
  vTaskDelete(NULL);
}
void taskPIP_Menengah(void* pv) {
  while (!pipFaseRendahPegang) vTaskDelay(1);
  uint32_t t0 = millis();
  while (millis() - t0 < PIP_GANGGU_MS) { volatile int x=0; x++; }
  vTaskDelete(NULL);
}
void taskPIP_Tinggi(void* pv) {
  while (!pipFaseRendahPegang) vTaskDelay(1);
  vTaskDelay(pdMS_TO_TICKS(50));
  uint32_t t0 = millis();
  ambilLockPIP();
  pipWaktuTunggu = millis() - t0;
  lepasLockPIP();
  pipSelesai = 1;
  vTaskDelete(NULL);
}

void jalankanDemoPIP(bool pakaiMutex) {
  pipPakaiMutex = pakaiMutex ? 1 : 0;
  pipFaseRendahPegang = 0; pipSelesai = 0; pipWaktuTunggu = 0;

  Serial.printf("\n===== DEMO PRIORITY INVERSION (%s) =====\n",
                pakaiMutex ? "MUTEX/PIP" : "SEMAPHORE tanpa PIP");

  xTaskCreatePinnedToCore(taskPIP_Rendah,   "pipL", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskPIP_Tinggi,   "pipH", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskPIP_Menengah, "pipM", 2048, NULL, 2, NULL, 1);

  while (!pipSelesai) vTaskDelay(pdMS_TO_TICKS(10));

  Serial.printf("Task prio-TINGGI menunggu lock: %lu ms\n",
                (unsigned long)pipWaktuTunggu);
  Serial.println(pakaiMutex
    ? ">> Singkat. PIP menaikkan prio task-rendah agar cepat melepas lock."
    : ">> Lama! Task menengah menyela -> priority inversion.");
}

/* ============================ GANTI STATE ================================ */
void gantiState(State_t baru) {
  stateSekarang   = baru;
  waktuMasukState = millis();
}

/* ====================== Task_Input  (prioritas 3) ======================= */
void Task_Input(void* pv) {
  for (;;) {
    uint32_t _t0 = ukurMulai();                       // [UKUR] mulai

    char k = keypad.getKey();
    if (k) { Event_t e = { EV_KEY, k }; xQueueSend(qEvent, &e, 0); }

    if (stateSekarang == ST_PEMBAYARAN) {
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        bool ok = apakahKartuNFC();
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
        Event_t e = { ok ? EV_RFID_OK : EV_RFID_SALAH, 0 };
        xQueueSend(qEvent, &e, 0);
      }
    }

    ukurSelesai(stInput, _t0);                         // [UKUR] selesai
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

/* ====================== Task_Display  (prioritas 1) ===================== */
void Task_Display(void* pv) {
  uint16_t pos = 0;
  uint32_t tGulir = 0;
  char b0[40], b1[40];
  bool g0;

  for (;;) {
    uint32_t _t0 = ukurMulai();                        // [UKUR] mulai

    xSemaphoreTake(mtxLCD, portMAX_DELAY);
    strcpy(b0, lcdBaris0); strcpy(b1, lcdBaris1); g0 = baris0Gulir;
    xSemaphoreGive(mtxLCD);

    if (g0 && strlen(b0) > LEBAR_LCD) {
      uint16_t panjang = strlen(b0);
      if (millis() - tGulir >= INTERVAL_GULIR_MS) {
        tGulir = millis();
        pos = (pos + 1) % panjang;
      }
      char tmp[LEBAR_LCD + 1];
      for (uint8_t i = 0; i < LEBAR_LCD; i++) tmp[i] = b0[(pos + i) % panjang];
      tmp[LEBAR_LCD] = '\0';
      lcdTulis(0, tmp);
    } else {
      pos = 0;
      lcdTulis(0, b0);
    }
    lcdTulis(1, b1);

    ukurSelesai(stDisplay, _t0);                       // [UKUR] selesai
    vTaskDelay(pdMS_TO_TICKS(60));
  }
}

/* ======================================================================== *
 *  [UKUR] Task_Monitor (prioritas 1) - cetak ringkasan metrik tiap 5 detik
 * ----------------------------------------------------------------------------
 *  Output siap salin ke TABEL 6.2 laporan.
 * ========================================================================== */
void cetakBarisStat(StatTask_t &s) {
  uint32_t avg   = s.iter ? (uint32_t)(s.sumUs / s.iter) : 0;
  // [UKUR] jitter = exec terlama - exec tercepat (dalam mikrodetik)
  uint32_t jitterUs = (s.minUs == 0xFFFFFFFF) ? 0 : (s.wcetUs - s.minUs);

  TaskHandle_t h = NULL;
  if      (&s == &stInput)   h = hInput;
  else if (&s == &stLogic)   h = hLogic;
  else if (&s == &stDisplay) h = hDisplay;

  // sisa stack minimum (word) -> byte ; pakai HighWaterMark
  uint32_t sisaB    = h ? uxTaskGetStackHighWaterMark(h) * sizeof(StackType_t) : 0;
  uint32_t pakaiB   = (s.stackTotalB > sisaB) ? (s.stackTotalB - sisaB) : 0;
  uint32_t pakaiPct = s.stackTotalB ? (pakaiB * 100UL / s.stackTotalB) : 0;

  // exec dalam ms (2 desimal) via integer: us/1000 ; jitter tampil dalam us
  Serial.printf("%-8s | WCET=%4lu.%02lu ms | avg=%4lu.%02lu ms | jitter=%lu us | miss=%lu/%lu | stack=%lu/%lu B (%lu%%)\n",
    s.nama,
    (unsigned long)(s.wcetUs/1000), (unsigned long)((s.wcetUs%1000)/10),
    (unsigned long)(avg/1000),      (unsigned long)((avg%1000)/10),
    (unsigned long)jitterUs,
    (unsigned long)s.miss, (unsigned long)s.iter,
    (unsigned long)pakaiB, (unsigned long)s.stackTotalB, (unsigned long)pakaiPct);
}

void Task_Monitor(void* pv) {
  // tunggu sistem stabil dulu
  vTaskDelay(pdMS_TO_TICKS(PERIODE_LAPOR_MS));
  for (;;) {
    uint32_t detik = millis() / 1000;
    Serial.printf("\n================ METRIK REAL-TIME @ %lus ================\n",
                  (unsigned long)detik);
    Serial.println("Task     | WCET terukur  | Avg execution | Jitter | Deadline miss | Stack usage");
    Serial.println("---------+---------------+---------------+--------+---------------+------------------");
    cetakBarisStat(stInput);
    cetakBarisStat(stLogic);
    cetakBarisStat(stDisplay);
    Serial.println("==========================================================\n");
    vTaskDelay(pdMS_TO_TICKS(PERIODE_LAPOR_MS));
  }
}

/* ======================================================================== *
 *  Task_Logic  (prioritas 2) - OTAK SISTEM
 * ======================================================================== */
void masukAdminMenu() {
  ukurAktif = false;                 // [UKUR] hentikan ukur saat mode admin/debug
  gantiState(ST_ADMIN_MENU);
  tampil(">> ADMIN MODE <<", "A=Edit  B=Test");
}

void Task_Logic(void* pv) {
  Event_t ev;
  tampil("Selamat Datang", "Tekan tombol...");

  for (;;) {
    /* (1) Tombol admin via ISR -> semaphore (non-blocking) */
    if (xSemaphoreTake(semTombol, 0) == pdTRUE) {
      if (stateSekarang < ST_ADMIN_MENU) {
        Serial.println("\n[TOMBOL] Masuk mode admin");
        masukAdminMenu();
      }
    }

    /* (2) Ambil 1 event input.
     *     [UKUR] PENTING: xQueueReceive bisa TIDUR sampai 20 ms menunggu event.
     *     Waktu tidur itu BUKAN waktu kerja CPU, jadi pengukuran dimulai
     *     SETELAH baris ini agar WCET hanya mencakup pemrosesan FSM. */
    bool adaEvent = (xQueueReceive(qEvent, &ev, pdMS_TO_TICKS(20)) == pdTRUE);

    uint32_t _t0 = ukurMulai();                        // [UKUR] mulai (setelah tunggu queue)

    char k = (adaEvent && ev.jenis == EV_KEY) ? ev.tombol : 0;

    /* (3) FSM */
    switch (stateSekarang) {

      case ST_STANDBY:
        tampil("Selamat Datang", "Tekan tombol...");
        if (k) { totalBelanja = 0; jmlDigit = 0; inputKode[0] = '\0';
                 gantiState(ST_PILIH_KODE); }
        break;

      case ST_PILIH_KODE: {
        char b1[20]; snprintf(b1, sizeof(b1), ": %s", inputKode);
        tampil("Pilih Kode (#=ok *=hps 0#=batal)    ", b1, true);
        if (k >= '0' && k <= '9') {
          if (jmlDigit < MAKS_DIGIT_KODE) { inputKode[jmlDigit++] = k; inputKode[jmlDigit] = '\0'; }
        } else if (k == '*') { jmlDigit = 0; inputKode[0] = '\0'; }
        else if (k == '#') {
          if (jmlDigit == 0) break;
          uint8_t kode = (uint8_t)atoi(inputKode);
          if (kode == 0) { totalBelanja = 0; jmlDigit = 0; inputKode[0]='\0';
                           gantiState(ST_STANDBY); break; }
          uint32_t harga;
          if (kode >= KODE_MINIMAL && kode <= KODE_MAKSIMAL && cariHarga(kode, &harga)) {
            totalBelanja += harga; jmlDigit = 0; inputKode[0] = '\0';
            gantiState(ST_TANYA_LAGI);
          } else gantiState(ST_KODE_SALAH);
        }
        break;
      }

      case ST_KODE_SALAH:
        tampil("Kode Tidak Ada", "Coba Lagi");
        if (millis() - waktuMasukState >= JEDA_PESAN_SINGKAT_MS) {
          jmlDigit = 0; inputKode[0] = '\0'; gantiState(ST_PILIH_KODE);
        }
        break;

      case ST_TANYA_LAGI:
        tampil("Transaksi Lain?    ", "A=Iya  B=Tidak", true);
        if (k == 'A') gantiState(ST_PILIH_KODE);
        else if (k == 'B') gantiState(ST_PEMBAYARAN);
        break;

      case ST_PEMBAYARAN: {
        char b0[24]; snprintf(b0, sizeof(b0), "Total Rp.%lu", (unsigned long)totalBelanja);
        tampil(b0, "Tap Kartu NFC");
        if (ev.jenis == EV_RFID_OK)         gantiState(ST_KONFIRMASI);
        else if (ev.jenis == EV_RFID_SALAH) gantiState(ST_KARTU_SALAH);
        break;
      }

      case ST_KARTU_SALAH:
        tampil("Jenis Kartu Salah", "Coba Lagi");
        if (millis() - waktuMasukState >= JEDA_PERINGATAN_KARTU_MS)
          gantiState(ST_PEMBAYARAN);
        break;

      case ST_KONFIRMASI:
        tampil("Pembayaran OK", "Terima Kasih :)");
        if (millis() - waktuMasukState >= JEDA_KONFIRMASI_BAYAR_MS) {
          servoPintu.write(SUDUT_SERVO_BUKA);
          gantiState(ST_SERVO_BUKA);
        }
        break;

      case ST_SERVO_BUKA:
        tampil("Silahkan Ambil", "Minuman Anda");
        if (millis() - waktuMasukState >= JEDA_SERVO_TERBUKA_MS) {
          servoPintu.write(SUDUT_SERVO_TUTUP);
          gantiState(ST_BERJAGA);
        }
        break;

      case ST_BERJAGA: {
        uint32_t sisa = (JEDA_BERJAGA_MS - (millis() - waktuMasukState)) / 1000;
        char b1[20]; snprintf(b1, sizeof(b1), "Standby %lus", (unsigned long)sisa);
        tampil("Sistem Berjaga", b1);
        if (k) { totalBelanja = 0; jmlDigit = 0; inputKode[0]='\0';
                 gantiState(ST_PILIH_KODE); }
        else if (millis() - waktuMasukState >= JEDA_BERJAGA_MS) {
          totalBelanja = 0; gantiState(ST_STANDBY);
        }
        break;
      }

      case ST_ADMIN_MENU:
        if (k == 'A') {
          editKode = 0; jmlDigit = 0; inputKode[0] = '\0';
          gantiState(ST_EDIT_KODE);
          tampil("Edit Barang", "Kode+#  0#=keluar");
        } else if (k == 'B') {
          KUNCI_DATA();
          memcpy(snapMinuman, daftarMinuman, sizeof(daftarMinuman));
          snapJumlah = jumlahMinuman;
          BUKA_DATA();
          Serial.println("\n[DEBUG] Masuk sandbox. Snapshot data diambil.");
          dumpDaftarBarang("Data sebelum debug");
          gantiState(ST_DEBUG_MENU);
          tampil(">> DEBUG MODE <<", "A=Race B=PIP #=klr");
        }
        break;

      case ST_EDIT_KODE: {
        char b1[20]; snprintf(b1, sizeof(b1), "Kode: %s", inputKode);
        tampil("Edit Barang", b1);
        if (k >= '0' && k <= '9') {
          if (jmlDigit < MAKS_DIGIT_KODE) { inputKode[jmlDigit++]=k; inputKode[jmlDigit]='\0'; }
        } else if (k == '*') { jmlDigit = 0; inputKode[0]='\0'; }
        else if (k == '#') {
          if (jmlDigit == 0) break;
          uint8_t kode = (uint8_t)atoi(inputKode);
          if (kode == 0) { ukurAktif = true; gantiState(ST_STANDBY); tampil("Selamat Datang", ""); break; }
          editKode = kode;
          KUNCI_DATA(); editBarangBaru = (cariIndex(kode) < 0); BUKA_DATA();
          jmlDigit = 0; inputHarga[0] = '\0';
          gantiState(ST_EDIT_HARGA);
          char b0[20]; snprintf(b0, sizeof(b0), editBarangBaru ? "Tambah kode %u" : "Edit kode %u", kode);
          tampil(b0, "Harga+#");
        }
        break;
      }

      case ST_EDIT_HARGA: {
        char b1[20]; snprintf(b1, sizeof(b1), "Rp: %s", inputHarga);
        tampil(editBarangBaru ? "Harga baru" : "Harga edit", b1);
        if (k >= '0' && k <= '9') {
          if (jmlDigit < MAKS_DIGIT_HARGA) { inputHarga[jmlDigit++]=k; inputHarga[jmlDigit]='\0'; }
        } else if (k == '*') { jmlDigit = 0; inputHarga[0]='\0'; }
        else if (k == '#') {
          if (jmlDigit == 0) break;
          uint32_t harga = strtoul(inputHarga, NULL, 10);
          bool ok = simpanBarang(editKode, harga);
          Serial.printf("[EDIT] kode=%u harga=%lu -> %s\n",
                        editKode, (unsigned long)harga, ok ? "OK" : "GAGAL/penuh");
          gantiState(ST_EDIT_SELESAI);
          tampil(ok ? "Tersimpan!" : "Gagal/Penuh", "");
        }
        break;
      }

      case ST_EDIT_SELESAI:
        if (millis() - waktuMasukState >= JEDA_PESAN_SINGKAT_MS) {
          jmlDigit = 0; inputKode[0] = '\0';
          gantiState(ST_EDIT_KODE);
          tampil("Edit Barang", "Kode+#  0#=keluar");
        }
        break;

      case ST_DEBUG_MENU:
        if (k == 'A') {
          gantiState(ST_DEBUG_JALAN);
          tampil("Race Test...", "Lihat Serial");
          jalankanDemoRace(false);
          jalankanDemoRace(true);
          tampil("Race Selesai", "#=kembali");
        } else if (k == 'B') {
          gantiState(ST_DEBUG_JALAN);
          tampil("PIP Test...", "Lihat Serial");
          jalankanDemoPIP(false);
          jalankanDemoPIP(true);
          tampil("PIP Selesai", "#=kembali");
        } else if (k == '#') {
          KUNCI_DATA();
          memcpy(daftarMinuman, snapMinuman, sizeof(daftarMinuman));
          jumlahMinuman = snapJumlah;
          BUKA_DATA();
          Serial.println("\n[DEBUG] Keluar sandbox. Data dipulihkan (revert).");
          dumpDaftarBarang("Data setelah revert");
          ukurAktif = true;                // [UKUR] lanjut ukur lagi (operasi normal)
          gantiState(ST_STANDBY);
          tampil("Selamat Datang", "");
        }
        break;

      case ST_DEBUG_JALAN:
        if (k == '#') {
          gantiState(ST_DEBUG_MENU);
          tampil(">> DEBUG MODE <<", "A=Race B=PIP #=klr");
        }
        break;
    }

    ukurSelesai(stLogic, _t0);                         // [UKUR] selesai
  }
}

/* ================================ SETUP =================================== */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Vending Machine RTOS - boot ===");

  Wire.begin();
  lcd.init();
  lcd.backlight();

  SPI.begin();
  rfid.PCD_Init();

  ESP32PWM::allocateTimer(0);
  servoPintu.setPeriodHertz(50);
  servoPintu.write(SUDUT_SERVO_TUTUP);
  servoPintu.attach(PIN_SERVO, 500, 2400);
  servoPintu.write(SUDUT_SERVO_TUTUP);
  delay(300);

  pinMode(PIN_TOMBOL, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_TOMBOL), isrTombol, RISING);

  qEvent      = xQueueCreate(10, sizeof(Event_t));
  mtxData     = xSemaphoreCreateMutex();
  mtxLCD      = xSemaphoreCreateMutex();
  mtxRace     = xSemaphoreCreateMutex();
  semTombol   = xSemaphoreCreateBinary();
  lockPIP_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(lockPIP_sem);
  lockPIP_mtx = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(Task_Input,   "Input",   3072, NULL, 3, &hInput,   1);
  xTaskCreatePinnedToCore(Task_Logic,   "Logic",   6144, NULL, 2, &hLogic,   1);
  xTaskCreatePinnedToCore(Task_Display, "Display", 3072, NULL, 1, &hDisplay, 1);

  // [UKUR] task monitor: prioritas 1 (sama rendah), 2048 B cukup utk printf
  xTaskCreatePinnedToCore(Task_Monitor, "Monitor", 2048, NULL, 1, &hMonitor, 1);

  Serial.printf("Boot OK. Barang awal: %u. Tekan tombol untuk mode admin.\n",
                jumlahMinuman);
  Serial.println("[UKUR] Ringkasan metrik akan tampil tiap 5 detik di Serial.");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
