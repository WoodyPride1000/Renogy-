// Renogy logger 本体(実装)。renogy_logger.ino から include される。
// 単一翻訳単位として扱うため、ここでは #pragma once のみ使う。
#pragma once

// ---- ピン -----------------------------------------------------------
#define SD_CS_PIN       10
#define LED_PIN         13      // 通常ステータス (SPI SCK と共用)
#define FAULT_LED_PIN   A0      // 故障表示 (旧VIN監視の分圧抵抗は外すこと)
#define BUTTON_PIN      4       // 押すと LOW
#define RENOGY_RX       2
#define RENOGY_TX       3
#define SDA_PIN         A4
#define SCL_PIN         A5

// ---- 調整値 ---------------------------------------------------------
const uint8_t  EEPROM_FAULT_LED_AFTER = 10;   // 連続失敗サイクル(=分)で故障表示
const uint8_t  EEPROM_WDT_RESET_AFTER = 30;   // 連続失敗サイクルで WDT リセット
const uint8_t  RENOGY_FAULT_AFTER     = 5;    // Renogy 通信断の連続回数で故障表示
const uint8_t  META_INTERVAL          = 16;   // 内蔵EEPROMへヒントを保存する間隔(レコード数)
const uint32_t RECOVERY_WINDOW        = 32;   // 高速パスの窓(ヒントの前後)
const uint32_t RECOVERY_SKIP          = 64;   // 走査が中断したときにヒントから飛ばす数
const uint8_t  MAX_BUS_ERRORS_IN_SCAN = 8;    // 走査中の連続バスエラーで打ち切る数

// ---- 故障フラグ (原因ごとに独立して点灯・消灯) -----------------------
// 点滅回数 = 立っている最下位ビットの位置 (1..4)
const uint8_t F_EEPROM = 0x01;   // 1回: EEPROM 書き込みの連続失敗
const uint8_t F_RENOGY = 0x02;   // 2回: Renogy 通信断
const uint8_t F_IC     = 0x04;   // 3回: 24FC1025 が応答しない
const uint8_t F_STATE  = 0x08;   // 4回: 状態復元が不確か、または復旧待ち
uint8_t faultFlags = 0;

// ---- オブジェクト ---------------------------------------------------
SoftwareSerial RenogySerial(RENOGY_RX, RENOGY_TX);
ModbusMaster node;
RTC_DS3231 rtc;
SdFat sd;
SdFile file;

// ---- リセット要因 (.init3 で MCUSR を退避し、WDT を止める) -----------
uint8_t mcusrMirror __attribute__((section(".noinit")));
void earlyInit(void) __attribute__((naked)) __attribute__((section(".init3"))) __attribute__((used));
void earlyInit(void) {
  mcusrMirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

// ---- レコード (32B, 128Bページに整列) -------------------------------
struct __attribute__((packed)) LogRecord {
  uint32_t count;     // 単調増加レコード番号
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;    // bit7: 時刻不確かフラグ, bit0-5: 秒
  int16_t  batV;      // 10mV単位  (0x7FFF = 無効)
  int16_t  batA;      // 10mA単位  (0x7FFF = 無効)
  uint8_t  soc;       // %         (0xFF   = 無効)
  uint16_t pvW;       // W         (0xFFFF = 無効)
  int16_t  pvV;       // 10mV単位  (0x7FFF = 無効)
  int16_t  pvA;       // 10mA単位  (0x7FFF = 無効)
  int16_t  chipTemp;  // DS3231 温度, 0.1℃単位
  uint8_t  reserved[6]; // 32B に揃えるための予備(0固定)。将来 loadA(0x105) や温度(0x103) に使える
  uint16_t crc16;     // 先頭から reserved までの CRC16
};

static_assert(sizeof(LogRecord) == 32, "LogRecord must be exactly 32 bytes");
static_assert(128 % sizeof(LogRecord) == 0, "records must not straddle 128B pages");
#ifdef BUFFER_LENGTH
static_assert(BUFFER_LENGTH >= 32, "Wire buffer must hold one record");
#endif

const int16_t  INVALID_I16 = 0x7FFF;
const uint16_t INVALID_U16 = 0xFFFF;
const uint8_t  INVALID_U8  = 0xFF;

const uint32_t MAX_EEPROM_SIZE  = 393216UL;                           // 3 x 128KB
const uint32_t CAPACITY_RECORDS = MAX_EEPROM_SIZE / sizeof(LogRecord); // 12288

// ---- 状態 -----------------------------------------------------------
bool     rtcPowerLostFlag  = false;
bool     stateValid        = false;   // false の間は記録もダンプもしない
uint8_t  consecutiveErrors = 0;       // EEPROM 書き込みを丸ごと失敗した連続サイクル数
uint8_t  renogyFailCount   = 0;
uint32_t globalCount       = 0;       // 次に書く count
uint32_t lastDumpedCount   = 0;       // SD へ出力済みの位置(次に出力すべき count)

struct {
  int16_t  batV = INVALID_I16;
  int16_t  batA = INVALID_I16;
  uint8_t  soc  = INVALID_U8;
  int16_t  pvV  = INVALID_I16;
  int16_t  pvA  = INVALID_I16;
  uint16_t pvW  = INVALID_U16;
} renogy;

// ---- 内蔵EEPROMのヒント (ローテーション保存) ------------------------
struct InternalMetadata {
  uint32_t globalCount;
  uint32_t lastDumpedCount;
  uint32_t invCheck;          // ~(globalCount ^ lastDumpedCount)
};
const uint16_t INTERNAL_SLOTS = 64;
uint16_t metaSlot = 0;        // 次に書くスロット(復元時に最新の次へ初期化)

enum ReadStatus : uint8_t { READ_OK = 0, READ_CRC_ERROR = 1, READ_BUS_ERROR = 2 };

// ====================================================================
// 故障表示
void setFault(uint8_t flag, bool on) {
  if (on) faultFlags |= flag; else faultFlags &= (uint8_t)~flag;
}

// 100ms ごとに呼ぶ。n回点滅(100ms点灯/200ms消灯)のあと1.5秒休む。
void updateFaultLed() {
  static uint8_t tick = 0;
  if (!faultFlags) {
    digitalWrite(FAULT_LED_PIN, LOW);
    tick = 0;
    return;
  }
  uint8_t n = 1;
  uint8_t f = faultFlags;
  while (!(f & 1)) { f >>= 1; n++; }
  const uint8_t period = n * 3 + 15;
  digitalWrite(FAULT_LED_PIN, (tick < n * 3 && (tick % 3) == 0) ? HIGH : LOW);
  tick++;
  if (tick >= period) tick = 0;
}

// ====================================================================
// I2Cバスクリア: スレーブが SDA を握ったままの状態を、SCL 9クロック + STOP で解放する
void i2cBusClear() {
  Wire.end();
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  for (uint8_t i = 0; i < 9 && digitalRead(SDA_PIN) == LOW; i++) {
    digitalWrite(SCL_PIN, LOW);     // 先に LOW を用意してから出力にする(グリッチ防止)
    pinMode(SCL_PIN, OUTPUT);
    delayMicroseconds(5);
    pinMode(SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(5);
  }

  digitalWrite(SDA_PIN, LOW);
  pinMode(SDA_PIN, OUTPUT);         // SCL=High のまま SDA を落とす (START)
  delayMicroseconds(5);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(SDA_PIN, INPUT_PULLUP);   // SDA を上げる (STOP)
  delayMicroseconds(5);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(25000, true);
}

// 24FC1025 x3 の応答確認 (0x50..0x55)。全て ACK なら true。
bool probeEepromChips(bool verbose) {
  bool allAck = true;
  for (uint8_t addr = 0x50; addr <= 0x55; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) {
      allAck = false;
      if (verbose) {
        Serial.print(F("[WARN] EEPROM at 0x"));
        Serial.print(addr, HEX);
        Serial.println(F(" not responding"));
      }
    }
  }
  return allAck;
}

// ====================================================================
// CRC16 (Modbus, 多項式 0xA001)
uint16_t calculateCRC16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else              crc >>= 1;
    }
  }
  return crc;
}

// 物理アドレス(0..393215) -> I2C デバイスアドレスとワードアドレス
//   chip0: 0x50/0x51, chip1 (A1=1): 0x52/0x53, chip2 (A2=1): 0x54/0x55
static void mapAddress(uint32_t addr, uint8_t &devAddr, uint16_t &wordAddr) {
  uint8_t  chip  = (uint8_t)(addr >> 17);        // 128KB / chip
  uint32_t a     = addr & 0x1FFFFUL;
  uint8_t  block = (uint8_t)((a >> 16) & 0x01);
  devAddr  = 0x50 | (chip << 1) | block;
  wordAddr = (uint16_t)(a & 0xFFFF);
}

// ====================================================================
// 1レコード読み出し (3値)
ReadStatus readRecordWithStatus(uint32_t slot, LogRecord &rec) {
  uint8_t  dev;
  uint16_t word;
  mapAddress(slot * sizeof(LogRecord), dev, word);

  Wire.beginTransmission(dev);
  Wire.write((uint8_t)(word >> 8));
  Wire.write((uint8_t)(word & 0xFF));
  if (Wire.endTransmission() != 0) return READ_BUS_ERROR;

  if (Wire.requestFrom((int)dev, (int)sizeof(LogRecord)) != (int)sizeof(LogRecord)) return READ_BUS_ERROR;
  uint8_t *p = (uint8_t *)&rec;
  for (uint8_t i = 0; i < sizeof(LogRecord); i++) {
    if (!Wire.available()) return READ_BUS_ERROR;
    p[i] = Wire.read();
  }

  if (rec.crc16 != calculateCRC16((const uint8_t *)&rec, sizeof(LogRecord) - 2)) return READ_CRC_ERROR;
  return READ_OK;
}

// ページ境界と Wire バッファ(32B)を超えないよう、最大16Bずつ書く。各チャンクで ACK ポーリング。
bool writeBytesToEEPROM(uint32_t addr, const uint8_t *data, uint8_t len) {
  uint8_t done = 0;
  while (done < len) {
    uint32_t cur      = addr + done;
    uint8_t  pageLeft = (uint8_t)(128 - (cur % 128));
    uint8_t  chunk    = len - done;
    if (chunk > pageLeft) chunk = pageLeft;
    if (chunk > 16)       chunk = 16;

    uint8_t  dev;
    uint16_t word;
    mapAddress(cur, dev, word);

    Wire.beginTransmission(dev);
    Wire.write((uint8_t)(word >> 8));
    Wire.write((uint8_t)(word & 0xFF));
    if (Wire.write(data + done, chunk) != chunk) return false;   // バッファ溢れ検出
    if (Wire.endTransmission() != 0) return false;

    unsigned long t0 = millis();
    bool acked = false;
    while (millis() - t0 < 15) {
      wdt_reset();
      Wire.beginTransmission(dev);
      if (Wire.endTransmission() == 0) { acked = true; break; }
      delayMicroseconds(500);
    }
    if (!acked) return false;

    done += chunk;
  }
  return true;
}

// ====================================================================
// 内蔵EEPROMへヒントを保存
void saveStateToEEPROM(uint32_t gCount, uint32_t dCount) {
  InternalMetadata m;
  m.globalCount     = gCount;
  m.lastDumpedCount = dCount;
  m.invCheck        = ~(gCount ^ dCount);
  EEPROM.put(metaSlot * sizeof(InternalMetadata), m);
  metaSlot = (metaSlot + 1) % INTERNAL_SLOTS;
}

// 起動時の状態復元。
//  戻り値 true : outCount / outDumped が使える (degraded=true なら不確かさあり)
//  戻り値 false: count を決められない (ヒント無し + バス死)。書き込まずに復旧を待つこと。
bool recoverStateFromEEPROM(uint32_t &outCount, uint32_t &outDumped, bool &degraded) {
  degraded = false;

  // 1. 内蔵ヒント
  uint32_t bestG = 0, bestDumped = 0;
  uint16_t bestSlot = 0;
  bool hintFound = false;
  for (uint16_t i = 0; i < INTERNAL_SLOTS; i++) {
    InternalMetadata m;
    EEPROM.get(i * sizeof(InternalMetadata), m);
    if (m.globalCount == 0xFFFFFFFFUL) continue;
    if (m.invCheck != (uint32_t)~(m.globalCount ^ m.lastDumpedCount)) continue;
    if (!hintFound || m.globalCount > bestG) {
      bestG = m.globalCount;
      bestDumped = m.lastDumpedCount;
      bestSlot = i;
      hintFound = true;
    }
  }
  if (hintFound) metaSlot = (bestSlot + 1) % INTERNAL_SLOTS;

  // 2. 高速パス: ヒント前後の窓だけ読む
  if (hintFound && bestG > 0) {
    uint32_t startSeq = (bestG > RECOVERY_WINDOW) ? (bestG - RECOVERY_WINDOW) : 0;
    uint32_t endSeq   = bestG + RECOVERY_WINDOW;
    uint32_t maxCountFound = 0;
    bool foundAny = false;
    bool aborted = false;
    uint8_t busErr = 0;
    LogRecord rec;

    for (uint32_t seq = startSeq; seq <= endSeq; seq++) {
      ReadStatus st = readRecordWithStatus(seq % CAPACITY_RECORDS, rec);
      if (st == READ_BUS_ERROR) {
        if (++busErr >= MAX_BUS_ERRORS_IN_SCAN) { aborted = true; break; }
        continue;
      }
      busErr = 0;
      if (st == READ_OK && rec.count == seq) {
        if (!foundAny || rec.count > maxCountFound) { maxCountFound = rec.count; foundAny = true; }
      }
    }

    // 中断せず、窓の端に達しておらず、ヒントと整合するときだけ採用
    if (!aborted && foundAny && maxCountFound < endSeq && (maxCountFound + 16 >= bestG)) {
      outCount  = (maxCountFound + 1 > bestG) ? (maxCountFound + 1) : bestG;
      outDumped = (bestDumped > outCount) ? outCount : bestDumped;
      return true;
    }
  }

  // 3. 全数走査
  uint32_t maxCount = 0;
  bool found = false;
  bool scanAborted = false;
  uint8_t busErr = 0;
  LogRecord rec;
  for (uint32_t i = 0; i < CAPACITY_RECORDS; i++) {
    if ((i & 0xFF) == 0) wdt_reset();
    ReadStatus st = readRecordWithStatus(i, rec);
    if (st == READ_BUS_ERROR) {
      if (++busErr >= MAX_BUS_ERRORS_IN_SCAN) { scanAborted = true; break; }
      continue;
    }
    busErr = 0;
    if (st == READ_OK && (!found || rec.count > maxCount)) { maxCount = rec.count; found = true; }
  }

  if (!scanAborted) {
    uint32_t scanNext = found ? (maxCount + 1) : 0;
    outCount  = (hintFound && bestG > scanNext) ? bestG : scanNext;   // count を後退させない
    outDumped = hintFound ? bestDumped : 0;
    if (outDumped > outCount) outDumped = outCount;
    return true;
  }

  // 4. 走査が中断した (バス死)
  if (hintFound) {
    // 真の最新はヒントより最大で十数件先。重複 count を避けるため、余裕をもって飛ばす。
    outCount  = bestG + RECOVERY_SKIP;
    outDumped = (bestDumped > outCount) ? outCount : bestDumped;
    degraded  = true;
    return true;
  }
  return false;   // ヒント無し + バス死: count を推測しない
}

// ====================================================================
// レコード書き込み(最大3回試行 + 読み戻し検証)。失敗しても count は進め、穴を残す。
bool writeRecord(LogRecord &rec) {
  rec.crc16 = calculateCRC16((const uint8_t *)&rec, sizeof(LogRecord) - 2);
  uint32_t slot = rec.count % CAPACITY_RECORDS;
  bool ok = false;

  for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++) {
    wdt_reset();
    ReadStatus st = READ_BUS_ERROR;                 // 試行ごとに初期化 (前回の値を引き継がない)
    if (writeBytesToEEPROM(slot * sizeof(LogRecord), (const uint8_t *)&rec, sizeof(LogRecord))) {
      LogRecord v;
      st = readRecordWithStatus(slot, v);
      ok = (st == READ_OK) && (memcmp(&v, &rec, sizeof(LogRecord)) == 0);
    }
    if (!ok) {
      if (st == READ_BUS_ERROR) i2cBusClear();      // バス障害なら、次の試行の前に解除
      delay(10);
    }
  }

  if (ok) {
    consecutiveErrors = 0;
    setFault(F_EEPROM, false);
    setFault(F_STATE, false);                       // 書けた = バスは生きている
  } else {
    if (consecutiveErrors < 255) consecutiveErrors++;
    if (consecutiveErrors >= EEPROM_FAULT_LED_AFTER) setFault(F_EEPROM, true);
    if (consecutiveErrors >= EEPROM_WDT_RESET_AFTER) {
      Serial.println(F("[FATAL] EEPROM write failed for too long. Waiting for WDT reset..."));
      Serial.flush();
      while (true) { /* WDT が再起動する */ }
    }
  }

  globalCount++;
  if (globalCount % META_INTERVAL == 0) saveStateToEEPROM(globalCount, lastDumpedCount);
  return ok;
}

// ====================================================================
// CSV 出力ヘルパ
static void print2(Print &p, uint8_t v) {
  if (v < 10) p.print('0');
  p.print(v);
}

// 固定小数点の整数整形 (float を使わない)。v は 10^-dec 単位の整数、dec は 1 または 2。
static void printFixed(Print &p, int32_t v, uint8_t dec) {
  if (v < 0) { p.print('-'); v = -v; }
  const uint32_t div = (dec == 1) ? 10UL : 100UL;
  p.print((unsigned long)((uint32_t)v / div));
  p.print('.');
  const uint32_t frac = (uint32_t)v % div;
  if (dec == 2 && frac < 10) p.print('0');
  p.print((unsigned long)frac);
}

static void printFixedOrNaN(Print &p, int16_t v, uint8_t dec) {
  if (v == INVALID_I16) p.print(F("NaN"));
  else                  printFixed(p, v, dec);
}

// datetime 列から flag 列まで(改行付き)。先頭の count と , は呼び出し側が出す。
static void writeCsvRow(Print &f, const LogRecord &r) {
  f.print(r.year);  f.print('-');
  print2(f, r.month); f.print('-');
  print2(f, r.day);   f.print(' ');
  print2(f, r.hour);  f.print(':');
  print2(f, r.minute); f.print(':');
  print2(f, r.second & 0x3F);
  f.print(',');
  printFixedOrNaN(f, r.batV, 2); f.print(',');
  printFixedOrNaN(f, r.batA, 2); f.print(',');
  if (r.soc == INVALID_U8) f.print(F("NaN")); else f.print(r.soc);
  f.print(',');
  if (r.pvW == INVALID_U16) f.print(F("NaN")); else f.print(r.pvW);
  f.print(',');
  printFixedOrNaN(f, r.pvV, 2); f.print(',');
  printFixedOrNaN(f, r.pvA, 2); f.print(',');
  printFixedOrNaN(f, r.chipTemp, 1);
  f.print(',');
  f.println((r.second & 0x80) ? F("UNCERTAIN_TIME") : F("OK"));
}

// ====================================================================
// 未ダンプ分を SD へ出力。バス障害で中断したら、進捗位置までだけ lastDumpedCount を進める。
void flushToSD() {
  if (!stateValid) {
    Serial.println(F("State unknown: dump refused."));
    return;
  }
  if (globalCount <= lastDumpedCount) {
    Serial.println(F("No new data to flush."));
    return;
  }

  Serial.println(F("Flushing to SD..."));
  digitalWrite(LED_PIN, HIGH);
  wdt_reset();

  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(4))) {
    Serial.println(F("SD mount FAILED"));
    digitalWrite(LED_PIN, LOW);
    return;
  }
  if (!file.open("datalog.csv", O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println(F("SD file open FAILED"));
    digitalWrite(LED_PIN, LOW);
    return;
  }
  if (file.fileSize() == 0) {
    file.println(F("count,datetime,batV,batA,soc,pvW,pvV,pvA,temp,flag"));
  }

  uint32_t targetEnd   = globalCount;
  uint32_t targetStart = lastDumpedCount;

  // リングが一周して上書きされた範囲は出力できない。欠落を行として残す(10列)。
  if (targetEnd - targetStart > CAPACITY_RECORDS) {
    uint32_t dropped = targetEnd - targetStart - CAPACITY_RECORDS;
    file.print(targetStart);
    file.print(F(",GAP,,,,,,,,LOST_"));
    file.println(dropped);
    targetStart = targetEnd - CAPACITY_RECORDS;
  }

  LogRecord rec;
  uint32_t progress = targetStart;    // ここまで処理済み(次に出すべき count)
  uint32_t good = 0, bad = 0;
  bool busAbort = false;

  for (uint32_t seq = targetStart; seq < targetEnd; seq++) {
    if ((seq & 0x3F) == 0) wdt_reset();
    uint32_t slot = seq % CAPACITY_RECORDS;

    ReadStatus st = readRecordWithStatus(slot, rec);
    if (st == READ_BUS_ERROR) {                 // 1回だけバスクリアして再試行
      i2cBusClear();
      st = readRecordWithStatus(slot, rec);
      if (st == READ_BUS_ERROR) { busAbort = true; break; }   // 行を書く前に中断
    }

    file.print(seq);
    file.print(',');
    if (st == READ_OK && rec.count == seq) {
      writeCsvRow(file, rec);
      good++;
    } else {
      file.println(F("CORRUPT_OR_EMPTY,,,,,,,,"));    // CRC 不正 / 穴 / 旧世代
      bad++;
    }
    progress = seq + 1;
  }

  bool ok = !file.getWriteError();
  ok = file.sync() && ok;
  ok = file.close() && ok;            // 失敗時も必ず close を呼ぶ

  if (!ok) {
    Serial.println(F("SD write/sync/close error. lastDumpedCount not advanced."));
    digitalWrite(LED_PIN, LOW);
    return;
  }

  lastDumpedCount = progress;         // 中断時も、書けた位置まで
  saveStateToEEPROM(globalCount, lastDumpedCount);

  if (busAbort) {
    Serial.print(F("[WARN] Dump aborted by bus error at #"));
    Serial.println(progress);
  } else {
    Serial.print(F("Flushed "));
    Serial.print(good);
    Serial.print(F(" ok, "));
    Serial.print(bad);
    Serial.println(F(" corrupt/empty."));
  }
  digitalWrite(LED_PIN, LOW);
}

// ====================================================================
// シリアルコマンド (行バッファ, ノンブロッキング)
static void printTwoDigits(uint8_t v) {
  if (v < 10) Serial.print('0');
  Serial.print(v);
}

// "123," のような十進フィールドを読む。sep が 0 のときは終端(\0)を要求する。厳密(余りがあれば失敗)。
static bool parseField(const char *&p, int &out, char sep) {
  if (*p < '0' || *p > '9') return false;
  long v = 0;
  while (*p >= '0' && *p <= '9') {
    v = v * 10 + (*p - '0');
    if (v > 9999) return false;
    p++;
  }
  out = (int)v;
  if (sep) { if (*p != sep) return false; p++; }
  else     { if (*p != '\0') return false; }
  return true;
}

static bool parseU32(const char *p, uint32_t &out) {
  if (*p < '0' || *p > '9') return false;
  uint32_t v = 0;
  for (; *p; p++) {
    if (*p < '0' || *p > '9') return false;
    uint8_t d = (uint8_t)(*p - '0');
    if (v > (0xFFFFFFFFUL - d) / 10UL) return false;   // オーバーフロー
    v = v * 10UL + d;
  }
  out = v;
  return true;
}

void handleCommand(char *cmd) {
  if (strncmp(cmd, "SET:", 4) == 0) {
    int y, mo, d, h, mi, s;
    const char *q = cmd + 4;
    bool parsed = parseField(q, y, ',') && parseField(q, mo, ',') && parseField(q, d, ',') &&
                  parseField(q, h, ',') && parseField(q, mi, ',') && parseField(q, s, 0);
    if (parsed && y >= 2000 && y <= 2099 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31 &&
        h >= 0 && h <= 23 && mi >= 0 && mi <= 59 && s >= 0 && s <= 59) {
      rtc.adjust(DateTime(y, mo, d, h, mi, s));
      rtcPowerLostFlag = false;
      Serial.println(F(">>> RTC updated"));
    } else {
      Serial.println(F(">>> Error. Use SET:YYYY,MM,DD,HH,MM,SS"));
    }
  }
  else if (strcasecmp(cmd, "TIME") == 0) {
    DateTime now = rtc.now();
    Serial.print(F(">>> "));
    Serial.print(now.year()); Serial.print('-');
    printTwoDigits(now.month());  Serial.print('-');
    printTwoDigits(now.day());    Serial.print(' ');
    printTwoDigits(now.hour());   Serial.print(':');
    printTwoDigits(now.minute()); Serial.print(':');
    printTwoDigits(now.second());
    if (rtcPowerLostFlag) Serial.print(F(" [UNRELIABLE TIME]"));
    Serial.println();
  }
  else if (strcasecmp(cmd, "DUMP") == 0) {
    flushToSD();
  }
  else if (strcasecmp(cmd, "STATUS") == 0) {
    Serial.print(F("State: "));          Serial.println(stateValid ? F("VALID") : F("WAITING (count unknown)"));
    Serial.print(F("Global Count: "));   Serial.println(globalCount);
    Serial.print(F("Last Dumped: "));    Serial.println(lastDumpedCount);
    Serial.print(F("Pending: "));        Serial.println(globalCount - lastDumpedCount);
    Serial.print(F("EEPROM errors: "));  Serial.println(consecutiveErrors);
    Serial.print(F("Renogy fails: "));   Serial.println(renogyFailCount);
    Serial.print(F("Fault flags: 0x"));  Serial.println(faultFlags, HEX);
  }
  else if (strncmp(cmd, "FORCE:", 6) == 0) {
    // 復旧待ちのときだけ有効。指定した count から手動で開始する。
    if (stateValid) {
      Serial.println(F(">>> FORCE ignored (state is valid)"));
    } else {
      uint32_t forced;
      if (!parseU32(cmd + 6, forced)) { Serial.println(F(">>> Error. Use FORCE:<count>")); return; }
      globalCount = forced;
      if (lastDumpedCount > globalCount) lastDumpedCount = globalCount;
      stateValid = true;
      setFault(F_STATE, true);                    // 最初の書き込み成功まで表示を残す
      saveStateToEEPROM(globalCount, lastDumpedCount);
      Serial.print(F(">>> Forced start at count "));
      Serial.println(globalCount);
    }
  }
}

void checkSerialCommands() {
  static char buf[40];
  static uint8_t n = 0;
  static bool discard = false;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!discard && n > 0) { buf[n] = '\0'; handleCommand(buf); }
      n = 0;
      discard = false;
      continue;
    }
    if (discard) continue;
    if (n < sizeof(buf) - 1) buf[n++] = c;
    else { discard = true; n = 0; }              // 長すぎる行は改行まで捨てる
  }
}

void handleButton() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      flushToSD();
      while (digitalRead(BUTTON_PIN) == LOW) { wdt_reset(); }
    }
  }
}

// ====================================================================
// Renogy 読み出し
static int16_t clampI16(uint32_t v) { return (v > 32766UL) ? 32766 : (int16_t)v; }

// DS3231 温度レジスタ(0x11 上位, 0x12 下位 bit7-6 = 0.25℃)を直接読み、0.1℃単位の整数で返す。
static int16_t readTempTenths() {
  Wire.beginTransmission(0x68);
  Wire.write((uint8_t)0x11);
  if (Wire.endTransmission() != 0) return INVALID_I16;
  if (Wire.requestFrom(0x68, 2) != 2) return INVALID_I16;
  int8_t  msb = (int8_t)Wire.read();
  uint8_t lsb = (uint8_t)Wire.read();
  return (int16_t)(((int16_t)msb * 100 + (lsb >> 6) * 25) / 10);
}

bool readRenogyData() {
  for (uint8_t i = 0; i < 3; i++) {
    wdt_reset();
    if (node.readHoldingRegisters(0x100, 12) == node.ku8MBSuccess) {
      uint16_t s = node.getResponseBuffer(0);
      uint16_t w = node.getResponseBuffer(9);
      renogy.soc  = (s >= 255) ? 254 : (uint8_t)s;
      renogy.batV = clampI16((uint32_t)node.getResponseBuffer(1) * 10UL);   // 0.1V -> 10mV
      renogy.batA = clampI16((uint32_t)node.getResponseBuffer(2));          // 10mA
      renogy.pvV  = clampI16((uint32_t)node.getResponseBuffer(7) * 10UL);   // 0.1V -> 10mV
      renogy.pvA  = clampI16((uint32_t)node.getResponseBuffer(8));          // 10mA
      renogy.pvW  = (w >= 0xFFFF) ? 0xFFFE : w;
      renogyFailCount = 0;
      setFault(F_RENOGY, false);
      return true;
    }
    delay(50);
  }

  if (renogyFailCount < 255) renogyFailCount++;       // 飽和させる
  if (renogyFailCount >= RENOGY_FAULT_AFTER) setFault(F_RENOGY, true);

  renogy.batV = INVALID_I16;
  renogy.batA = INVALID_I16;
  renogy.soc  = INVALID_U8;
  renogy.pvV  = INVALID_I16;
  renogy.pvA  = INVALID_I16;
  renogy.pvW  = INVALID_U16;
  return false;
}

void logToEEPROM() {
  digitalWrite(LED_PIN, HIGH);

  DateTime now = rtc.now();

  LogRecord rec;
  memset(&rec, 0, sizeof(rec));
  rec.count  = globalCount;
  rec.year   = now.year();
  rec.month  = now.month();
  rec.day    = now.day();
  rec.hour   = now.hour();
  rec.minute = now.minute();
  rec.second = (now.second() & 0x3F) | (rtcPowerLostFlag ? 0x80 : 0x00);
  rec.batV   = renogy.batV;
  rec.batA   = renogy.batA;
  rec.soc    = renogy.soc;
  rec.pvW    = renogy.pvW;
  rec.pvV    = renogy.pvV;
  rec.pvA    = renogy.pvA;
  rec.chipTemp = readTempTenths();

  uint32_t cnt = rec.count;
  if (!writeRecord(rec)) {
    Serial.print(F("[ERROR] write failed, hole at #"));
    Serial.println(cnt);
  } else {
    Serial.print(F("[#")); Serial.print(cnt);
    Serial.print(F("] BatV:"));
    printFixedOrNaN(Serial, renogy.batV, 2);
    Serial.print(F(" PV:"));
    if (renogy.pvW == INVALID_U16) Serial.print(F("NaN")); else Serial.print(renogy.pvW);
    Serial.println(F("W"));
  }

  digitalWrite(LED_PIN, LOW);
}

// ====================================================================
// 状態が不確かな間の待機。30秒ごとにバスクリアして再走査する。
void recoveryTick() {
  setFault(F_STATE, true);
  Serial.println(F("[WAIT] count unknown. Retrying recovery..."));
  i2cBusClear();

  bool degraded = false;
  stateValid = recoverStateFromEEPROM(globalCount, lastDumpedCount, degraded);
  if (stateValid) {
    setFault(F_STATE, degraded);
    Serial.print(F("Recovered Global Count: ")); Serial.println(globalCount);
    return;
  }

  unsigned long t0 = millis();
  while (millis() - t0 < 30000UL && !stateValid) {
    wdt_reset();
    checkSerialCommands();                       // FORCE:<count> で手動開始できる
    updateFaultLed();
    delay(100);
  }
}

void maybeProbeChips() {
  static uint8_t cycles = 0;
  if (++cycles >= 60) {                          // 約1時間ごと
    cycles = 0;
    setFault(F_IC, !probeEepromChips(false));
  }
}

// 次の分まで待つ。バス死で rtc.now() が不定でも、30秒未満では抜けない。
void waitForNextMinute() {
  static uint8_t lastMinute = 255;
  if (lastMinute == 255) lastMinute = rtc.now().minute();

  unsigned long start = millis();
  while (millis() - start < 60000UL) {
    wdt_reset();
    checkSerialCommands();
    handleButton();
    updateFaultLed();
    delay(100);

    uint8_t m = rtc.now().minute();
    if (m != lastMinute && millis() - start >= 30000UL) {
      lastMinute = m;
      break;
    }
  }
}

// ====================================================================
void setup() {
  i2cBusClear();                                 // 最優先: スレーブが握った SDA を解放

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(FAULT_LED_PIN, OUTPUT);
  digitalWrite(FAULT_LED_PIN, LOW);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  delay(1000);

  RenogySerial.begin(9600);
  node.begin(1, RenogySerial);

  Serial.println(F("=== Renogy Logger rev.F ==="));
  if (mcusrMirror) {
    Serial.print(F("Reset cause MCUSR=0x"));
    Serial.print(mcusrMirror, HEX);
    if (mcusrMirror & _BV(WDRF)) Serial.print(F(" (WDT)"));
    if (mcusrMirror & _BV(BORF)) Serial.print(F(" (BOD)"));
    Serial.println();
  }

  wdt_enable(WDTO_8S);                           // 復元の走査中も wdt_reset を呼ぶ

  setFault(F_IC, !probeEepromChips(true));

  if (!rtc.begin()) {
    rtcPowerLostFlag = true;
    Serial.println(F("RTC ERROR"));
  } else if (rtc.lostPower()) {
    rtcPowerLostFlag = true;
    Serial.println(F("RTC lost power: time is unreliable"));
  }

  bool degraded = false;
  stateValid = recoverStateFromEEPROM(globalCount, lastDumpedCount, degraded);
  if (!stateValid) {
    setFault(F_STATE, true);
    Serial.println(F("[WAIT] count unknown (no hint and bus dead). Logging suspended."));
  } else {
    if (degraded) {
      setFault(F_STATE, true);
      Serial.println(F("[WARN] Recovery scan aborted: count jumped ahead of hint."));
    }
    Serial.print(F("Recovered Global Count: ")); Serial.println(globalCount);
    Serial.print(F("Recovered Last Dumped: "));  Serial.println(lastDumpedCount);
  }

  for (uint8_t i = 0; i < 3; i++) {
    wdt_reset();
    digitalWrite(LED_PIN, HIGH); delay(200);
    digitalWrite(LED_PIN, LOW);  delay(200);
  }
}

void loop() {
  wdt_reset();
  if (!stateValid) {
    recoveryTick();
    return;
  }
  readRenogyData();
  logToEEPROM();
  maybeProbeChips();
  waitForNextMinute();
}
