/*
  ====================================================================
  Renogy チャージコントローラー バッテリーロガー  rev.G
  ATmega328P (Uno/Nano) + 24FC1025 x3 (384KB リングバッファ) + DS3231 + SD

  設計方針
  - 正本: 外付けEEPROMの走査で得た最大 count。内蔵EEPROMは起動高速化のヒント。
  - count は単調増加。ダンプ後もリセットしない(lastDumpedCount で管理)。
  - 状態が不確か(ヒント無し + バス死)のときは、書き始めずに復旧を待つ。
    (シリアルの FORCE:<count> で手動開始できる)
  - 障害は原因ごとのビットで管理し、点滅回数で原因を示す(A0)。
  - 読み出しは OK / CRC_ERROR / BUS_ERROR の3値。バスエラーを破損扱いしない。
  - rev.G: sscanf / strtoul / float を除去(Flash 節約)。数値整形は整数演算、
    DS3231 温度はレジスタ(0x11,0x12)を直接読む。
  - 推奨ビルドフラグ(Flash/SRAM 節約)。実測(avr-gcc 7.3 -Os, 実ライブラリ):
      フラグなし  Flash 30,064 B / SRAM 1,613 B
      フラグあり  Flash 28,552 B / SRAM 1,501 B (スタック余裕 約 547 B)
      -DUSE_LONG_FILE_NAMES=0 -DSERIAL_TX_BUFFER_SIZE=16
      -DSERIAL_RX_BUFFER_SIZE=32 -D_SS_MAX_RX_BUFF=32
  - 実機未検証。ホスト側テスト(電源断注入)は通過済み。
  ====================================================================

  構成: このファイルは include だけ。実装は renogy_logger.h に置く。
  理由: Arduino IDE は .ino 内の関数に自動でプロトタイプを生成し、スケッチ冒頭
        (型定義より前)へ挿入する。LogRecord / ReadStatus のような自作型を
        引数・戻り値に持つ関数があると 'does not name a type' になる。
        関数を .h 側に置けば、この自動生成の対象外になる。
*/

#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#include "renogy_logger.h"
