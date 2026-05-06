#include <M5Core2.h>
#include <ctype.h>

// for SD-Updater
#define SDU_ENABLE_GZ
#include <M5StackUpdater.h>

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "src/hid_l2cap.h"
#include <Free_Fonts.h>
#include <ArduinoJson.h>

// Define M5TAB_DIAG (e.g. via -DM5TAB_DIAG in arduino-cli build flags) to
// enable the lightweight `[mem]` heap / stack monitor printed every 5 s.
// Off by default so production builds carry zero diagnostic overhead.
// (Flag name kept identical to the Tab5 sketch for consistency.)
#ifdef M5TAB_DIAG
#include <esp_system.h>
#include <esp_heap_caps.h>
#endif

// ---- UIフォントヘルパー（GFXFFベースのきれいな描画に統一） ----
// 旧 setTextSize(1)→Small, (2)→Medium, (3)→Large, (6)→Huge 相当
static inline void uiFontSmall()  { M5.Lcd.setFreeFont(FSS9);   M5.Lcd.setTextSize(1); }
static inline void uiFontMedium() { M5.Lcd.setFreeFont(FSSB12); M5.Lcd.setTextSize(1); }
static inline void uiFontLarge()  { M5.Lcd.setFreeFont(FSSB18); M5.Lcd.setTextSize(1); }
static inline void uiFontHuge()   { M5.Lcd.setFreeFont(FSSB24); M5.Lcd.setTextSize(1); }
// テキスト描画（左上基準 / 矩形中央基準）
static inline void uiDrawL(const char* t, int x, int y) {
  M5.Lcd.setTextDatum(TL_DATUM);
  M5.Lcd.drawString(t, x, y);
}
static inline void uiDrawC(const char* t, int rx, int ry, int rw, int rh) {
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString(t, rx + rw / 2, ry + rh / 2);
}

// CORE2でのピンアサイン（M5 MIDI Module2）
#define RXD2 13
#define TXD2 14

// Bluetooth HID settings for SPT-10 foot pedal
#define TARGET_BT_ADDR  { 0xA1, 0x12, 0x12, 0x92, 0x0E, 0x23 } // SPT-10 (Bluetooth Music Pedal) MAC address
#define BT_CONNECT_TIMEOUT  10000

static const uint8_t PEDAL_LEFT_KEY  = 0x52; // HID keyboard Up Arrow
static const uint8_t PEDAL_RIGHT_KEY = 0x51; // HID keyboard Down Arrow

// SD card bond persistence settings
#define BT_BOND_FILE        "/bt_bond.dat"
#define NVS_BT_NAMESPACE    "bt_config.conf"
#define NVS_BT_KEY_PREFIX   "bt_cfg_key"
#define NVS_BT_MAX_SIZE     4096
#define NVS_BT_CHUNK_SIZE   1984

// 画面サイズ
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define USB_COMMAND_BUFFER_SIZE 128

// モード定義
enum DisplayMode {
  DIRECT_MODE,
  KEY_MODE,
  INSTANT_MODE,
  SEQUENCE_MODE,
  MIDI_MANAGE_MODE,
  CONFIG_EDIT_MODE,    // B long-press: edit /config.json
  BASE_SET_MODE        // A long-press: pick transpose base reference
};

enum MidiManagePage {
  MIDI_PAGE_FILTER,
  MIDI_PAGE_MAPPER
};

enum MidiMapperEditPage {
  MAPPER_PAGE_SOURCE,
  MAPPER_PAGE_DEST
};

enum MidiMessageKind {
  MIDI_KIND_NOTE_OFF,
  MIDI_KIND_NOTE_ON,
  MIDI_KIND_KEY_PRESSURE,
  MIDI_KIND_PROGRAM_CHANGE,
  MIDI_KIND_CONTROL_CHANGE,
  MIDI_KIND_CHANNEL_PRESSURE,
  MIDI_KIND_PITCH_BEND,
  MIDI_KIND_SYSTEM_EXCLUSIVE,
  MIDI_KIND_MIDI_TIME_CODE,
  MIDI_KIND_SONG_POSITION,
  MIDI_KIND_SONG_SELECT,
  MIDI_KIND_TUNE_REQUEST,
  MIDI_KIND_CLOCK,
  MIDI_KIND_START,
  MIDI_KIND_CONTINUE,
  MIDI_KIND_STOP,
  MIDI_KIND_ACTIVE_SENSE,
  MIDI_KIND_SYSTEM_RESET,
  MIDI_KIND_COUNT
};

// 転調範囲モード（3種類に拡張）
enum TransposeRange {
  RANGE_0_TO_12,        // 0から+11
  RANGE_MINUS12_TO_0,   // -11から0
  RANGE_MINUS5_TO_6     // -5から+6（新規追加）
};

// 構造体の定義
struct TouchButton {
  int x, y, w, h;
  int8_t value;
  const char* label;
};

struct KeyButton {
  int x, y, w, h;
  int8_t keyValue;
  const char* keyName;
  bool isBlackKey;
};

// グローバル変数の宣言
TouchButton directButtons[12];
TouchButton instantButtons[8];
KeyButton majorKeys[12];
KeyButton minorKeys[12];

// 転調値の設定
volatile int8_t transposeValue = 0;
bool transposeButtons[12];  // 0から11の12個のボタン（4x3レイアウト）

// KeyMode用の設定
int8_t selectedMajorKey = -1;   // -1は未選択
int8_t selectedMinorKey = -1;   // -1は未選択
bool majorUpperTranspose = false; // false=通常転調, true=上位転調
bool minorUpperTranspose = false; // false=通常転調, true=下位転調

// UI状態管理
DisplayMode currentMode = DIRECT_MODE;
TransposeRange transposeRange = RANGE_MINUS5_TO_6;  // 初期レンジを-5から+6に変更
bool needFullRedraw = true;
bool needPartialUpdate = false;
unsigned long lastButtonCheck = 0;
const unsigned long BUTTON_DEBOUNCE = 200;

// タッチ入力のラッチ（押しっぱなし連続反応を抑止）
bool touchWasActive = false;
char usbCommandBuffer[USB_COMMAND_BUFFER_SIZE];
size_t usbCommandLength = 0;
volatile bool g_usbBinaryTransferActive = false;

// 汎用ナビゲーションボタン構造体
struct NavButton {
  int x, y, w, h;
};

struct MidiFilterRule {
  bool enabled;
  MidiMessageKind kind;
  int8_t channel;  // -1 = ALL, 0-15 = MIDI Ch 1-16
};

struct MidiMapperRule {
  bool enabled;
  MidiMessageKind srcKind;
  int8_t srcChannel;   // -1 = ALL
  int16_t srcData1;    // -1 = ANY
  int16_t srcMin;
  int16_t srcMax;
  MidiMessageKind dstKind;
  int8_t dstChannel;   // -1 = KEEP
  int16_t dstData1;    // -1 = KEEP
  int16_t dstMin;
  int16_t dstMax;
};

struct MidiMessage {
  uint8_t bytes[3];
  uint8_t length;
  MidiMessageKind kind;
  bool hasChannel;
  int8_t channel;
};

// Forward declarations for Arduino's auto-prototype generation.
const char* getMidiKindLabel(MidiMessageKind kind);
bool midiKindHasChannel(MidiMessageKind kind);
uint8_t getMidiStatusForKind(MidiMessageKind kind, uint8_t channel);
MidiMessageKind getMidiKindFromStatus(uint8_t status);
uint8_t getMidiMessageLengthForKind(MidiMessageKind kind);
bool midiKindSupportsData1(MidiMessageKind kind);
int getMidiValueMax(MidiMessageKind kind);
int getMidiPrimaryData(const MidiMessage& msg);
int getMidiValueData(const MidiMessage& msg);
bool midiFilterRuleMatches(const MidiFilterRule& rule, const MidiMessage& msg);
bool shouldAllowMidiMessage(const MidiMessage& msg);
bool midiMapperRuleMatches(const MidiMapperRule& rule, const MidiMessage& msg);
MidiMessage buildMappedMidiMessage(const MidiMapperRule& rule, const MidiMessage& srcMsg);
MidiMessage applyMidiMapper(const MidiMessage& srcMsg);
void formatMidiFilterRuleSummary(const MidiFilterRule& rule, int index, char* out, size_t outSize);
void formatMidiMapperRuleSummary(const MidiMapperRule& rule, int index, char* out, size_t outSize);
void adjustWrappedMidiKind(MidiMessageKind& kind, int delta);
void normalizeMapperRule(MidiMapperRule& rule);
void handleParsedMidiMessage(const MidiMessage& inMsg);
void loadTestRules();
void setDefaultConfig();
bool loadDeviceConfigFromSD();
bool saveDeviceConfigToSD();
void applyDeviceConfig();
void enterConfigEditMode();
void exitConfigEditMode(bool save, bool apply);
void drawConfigEditMode();
void processConfigEditTouch(TouchPoint_t pos);
void enterBaseSetMode();
void exitBaseSetMode();
void drawBaseSetMode();
void processBaseSetTouch(TouchPoint_t pos);
void cycleBaseSetPage();
void setTransposeBase(int newBase);
void processUsbSerialCommands();
void handleUsbSerialCommand(char* line);
void printUsbSerialHelp();
void printUsbSerialStatus();
void sendUsbSerialScreenshot(const char* formatToken);
void dispatchTouchPoint(TouchPoint_t pos);
void injectTouchPoint(int16_t x, int16_t y);
void handleButtonAAction();
void handleButtonBAction();
void handleButtonCShortAction();
void handleButtonCLongAction();
bool setModeFromCommand(const char* modeName);
bool setGroupFromCommand(const char* groupName);
bool parseIntValue(const char* token, int& outValue);
bool tokenEqualsIgnoreCase(const char* lhs, const char* rhs);
const char* getDisplayModeLabel(DisplayMode mode);
const char* getBtStatusLabel(BT_STATUS status);
bool isUsbBinaryTransferActive(void);
int getTrackedNoteStateIndex(uint8_t midiNote, uint8_t channel);
void clearTrackedNoteStates(void);
bool isMidiInputIdle(unsigned long now);
void processDeferredStorageTasks(unsigned long now);
void handleTransposeChange(int8_t newTransposeValue);
void processTouch(void);
void sendAllNotesOff(void);
void processMIDI(void);

// Sequence Mode用の設定
#define SEQ_PATTERN_COUNT 16
#define SEQ_STEP_COUNT    6
#define SEQ_FILE          "/seq_data.dat"

int8_t seqPatterns[SEQ_PATTERN_COUNT][SEQ_STEP_COUNT]; // 16パターン x 6ステップ
int seqCurrentPattern = 0;  // 0-15
int seqCurrentStep = 0;     // 0-5

struct SeqStepSlot {
  int slotX, slotY, slotW, slotH;
  int upBtnX, upBtnY, upBtnW, upBtnH;
  int downBtnX, downBtnY, downBtnW, downBtnH;
};

SeqStepSlot seqSteps[SEQ_STEP_COUNT];
NavButton seqPatLeftBtn, seqPatRightBtn;
NavButton seqStepLeftBtn, seqStepRightBtn;
NavButton seqSaveBtn;

// Instant Mode の 0 ボタン（上段の上、ボタン2個分の幅）
NavButton instantZeroBtn;

// MIDI Manage mode
static const int MAX_FILTER_RULES = 8;
static const int MAX_MAPPER_RULES = 8;
MidiManagePage midiManagePage = MIDI_PAGE_FILTER;
MidiMapperEditPage midiMapperEditPage = MAPPER_PAGE_SOURCE;
bool midiFilterBypass = true;
bool midiMapperBypass = true;
MidiFilterRule midiFilterRules[MAX_FILTER_RULES];
MidiMapperRule midiMapperRules[MAX_MAPPER_RULES];
int midiFilterRuleCount = 1;
int midiMapperRuleCount = 1;
int midiSelectedFilterRule = 0;
int midiSelectedMapperRule = 0;
DisplayMode lastTransposeMode = DIRECT_MODE;
bool btnCLongPressHandled = false;
const unsigned long MODE_LONG_PRESS_MS = 700;

// Bluetooth HID foot pedal state management
static portMUX_TYPE g_keyStateMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_leftPedalPressed = false;
static volatile bool g_rightPedalPressed = false;
static volatile bool g_pedalNeedsProcessing = false;

static uint32_t g_lastReconnectAttempt = 0;
static BT_STATUS g_lastStatusDrawn = BT_UNINITIALIZED;

// All Notes Off機能の有効/無効
bool allNotesOffEnabled = false;

// MIDI統計情報
unsigned long midiInCount = 0;
unsigned long midiOutCount = 0;
unsigned long g_lastMidiInputAt = 0;
bool g_btBondSavePending = false;
bool g_seqSavePending = false;

// アクティブなノート追跡（88鍵盤対応）
// 標準的な88鍵盤: A0(21) から C8(108)
#define PIANO_LOWEST_NOTE 21   // A0
#define PIANO_HIGHEST_NOTE 108 // C8
#define PIANO_KEY_COUNT 88
#define MIDI_CHANNEL_COUNT 16
#define TRACKED_NOTE_STATE_COUNT (PIANO_KEY_COUNT * MIDI_CHANNEL_COUNT)

struct NoteState {
  bool isActive;
  int8_t originalTranspose;  // このノートが押された時の転調値
  uint8_t channel;
  uint8_t velocity;
};

// 現在押されている鍵盤の状態（88鍵盤分）
NoteState currentNoteStates[TRACKED_NOTE_STATE_COUNT];

// 転調変更時の一時保存用
NoteState savedNoteStates[TRACKED_NOTE_STATE_COUNT];
int8_t savedTranspose = 0;

// ユーティリティ：-12〜+12にクランプ
int8_t clampTranspose(int8_t v) {
  if (v < -12) return -12;
  if (v > 12) return 12;
  return v;
}

bool touchInRect(TouchPoint_t pos, int x, int y, int w, int h) {
  return pos.x >= x && pos.x <= x + w && pos.y >= y && pos.y <= y + h;
}

const char* getMidiKindLabel(MidiMessageKind kind) {
  static const char* labels[MIDI_KIND_COUNT] = {
    "NoteOff", "NoteOn", "KeyPrs", "PrgChg", "CtrlChg", "ChPrs", "Bend",
    "SysEx", "MTC", "SongPos", "SongSel", "TuneReq",
    "Clock", "Start", "Cont", "Stop", "ActSn", "Reset"
  };
  if (kind < 0 || kind >= MIDI_KIND_COUNT) return "Unknown";
  return labels[kind];
}

bool midiKindHasChannel(MidiMessageKind kind) {
  return kind <= MIDI_KIND_PITCH_BEND;
}

uint8_t getMidiStatusForKind(MidiMessageKind kind, uint8_t channel) {
  switch (kind) {
    case MIDI_KIND_NOTE_OFF:         return 0x80 | (channel & 0x0F);
    case MIDI_KIND_NOTE_ON:          return 0x90 | (channel & 0x0F);
    case MIDI_KIND_KEY_PRESSURE:     return 0xA0 | (channel & 0x0F);
    case MIDI_KIND_CONTROL_CHANGE:   return 0xB0 | (channel & 0x0F);
    case MIDI_KIND_PROGRAM_CHANGE:   return 0xC0 | (channel & 0x0F);
    case MIDI_KIND_CHANNEL_PRESSURE: return 0xD0 | (channel & 0x0F);
    case MIDI_KIND_PITCH_BEND:       return 0xE0 | (channel & 0x0F);
    case MIDI_KIND_SYSTEM_EXCLUSIVE: return 0xF0;
    case MIDI_KIND_MIDI_TIME_CODE:   return 0xF1;
    case MIDI_KIND_SONG_POSITION:    return 0xF2;
    case MIDI_KIND_SONG_SELECT:      return 0xF3;
    case MIDI_KIND_TUNE_REQUEST:     return 0xF6;
    case MIDI_KIND_CLOCK:            return 0xF8;
    case MIDI_KIND_START:            return 0xFA;
    case MIDI_KIND_CONTINUE:         return 0xFB;
    case MIDI_KIND_STOP:             return 0xFC;
    case MIDI_KIND_ACTIVE_SENSE:     return 0xFE;
    case MIDI_KIND_SYSTEM_RESET:     return 0xFF;
    default:                         return 0x00;
  }
}

MidiMessageKind getMidiKindFromStatus(uint8_t status) {
  if ((status & 0xF0) == 0x80) return MIDI_KIND_NOTE_OFF;
  if ((status & 0xF0) == 0x90) return MIDI_KIND_NOTE_ON;
  if ((status & 0xF0) == 0xA0) return MIDI_KIND_KEY_PRESSURE;
  if ((status & 0xF0) == 0xB0) return MIDI_KIND_CONTROL_CHANGE;
  if ((status & 0xF0) == 0xC0) return MIDI_KIND_PROGRAM_CHANGE;
  if ((status & 0xF0) == 0xD0) return MIDI_KIND_CHANNEL_PRESSURE;
  if ((status & 0xF0) == 0xE0) return MIDI_KIND_PITCH_BEND;

  switch (status) {
    case 0xF0: return MIDI_KIND_SYSTEM_EXCLUSIVE;
    case 0xF1: return MIDI_KIND_MIDI_TIME_CODE;
    case 0xF2: return MIDI_KIND_SONG_POSITION;
    case 0xF3: return MIDI_KIND_SONG_SELECT;
    case 0xF6: return MIDI_KIND_TUNE_REQUEST;
    case 0xF8: return MIDI_KIND_CLOCK;
    case 0xFA: return MIDI_KIND_START;
    case 0xFB: return MIDI_KIND_CONTINUE;
    case 0xFC: return MIDI_KIND_STOP;
    case 0xFE: return MIDI_KIND_ACTIVE_SENSE;
    case 0xFF: return MIDI_KIND_SYSTEM_RESET;
    default:   return MIDI_KIND_SYSTEM_EXCLUSIVE;
  }
}

uint8_t getMidiMessageLengthForKind(MidiMessageKind kind) {
  switch (kind) {
    case MIDI_KIND_NOTE_OFF:
    case MIDI_KIND_NOTE_ON:
    case MIDI_KIND_KEY_PRESSURE:
    case MIDI_KIND_CONTROL_CHANGE:
    case MIDI_KIND_PITCH_BEND:
    case MIDI_KIND_SONG_POSITION:
      return 3;
    case MIDI_KIND_PROGRAM_CHANGE:
    case MIDI_KIND_CHANNEL_PRESSURE:
    case MIDI_KIND_MIDI_TIME_CODE:
    case MIDI_KIND_SONG_SELECT:
      return 2;
    default:
      return 1;
  }
}

bool midiKindSupportsData1(MidiMessageKind kind) {
  switch (kind) {
    case MIDI_KIND_NOTE_OFF:
    case MIDI_KIND_NOTE_ON:
    case MIDI_KIND_KEY_PRESSURE:
    case MIDI_KIND_CONTROL_CHANGE:
    case MIDI_KIND_PROGRAM_CHANGE:
    case MIDI_KIND_MIDI_TIME_CODE:
    case MIDI_KIND_SONG_SELECT:
      return true;
    default:
      return false;
  }
}

int getMidiValueMax(MidiMessageKind kind) {
  switch (kind) {
    case MIDI_KIND_PITCH_BEND:
    case MIDI_KIND_SONG_POSITION:
      return 16383;
    default:
      return 127;
  }
}

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

void initMidiManagementDefaults() {
  midiFilterRuleCount = 1;
  midiMapperRuleCount = 1;
  midiSelectedFilterRule = 0;
  midiSelectedMapperRule = 0;

  midiFilterRules[0].enabled = false;
  midiFilterRules[0].kind = MIDI_KIND_CONTROL_CHANGE;
  midiFilterRules[0].channel = -1;

  midiMapperRules[0].enabled = false;
  midiMapperRules[0].srcKind = MIDI_KIND_CONTROL_CHANGE;
  midiMapperRules[0].srcChannel = -1;
  midiMapperRules[0].srcData1 = -1;
  midiMapperRules[0].srcMin = 0;
  midiMapperRules[0].srcMax = 127;
  midiMapperRules[0].dstKind = MIDI_KIND_CONTROL_CHANGE;
  midiMapperRules[0].dstChannel = -1;
  midiMapperRules[0].dstData1 = -1;
  midiMapperRules[0].dstMin = 0;
  midiMapperRules[0].dstMax = 127;
}

// Populate two filter + two mapper rules with deterministic test content.
// Used by `LOAD TESTRULES` USB-serial command from the regression script.
// Both bypass flags stay at their current values; the orchestrator flips
// them and per-rule .enabled as it walks through the 6 phases.
//
// filter[0] = block all PitchBend (any channel)
// filter[1] = block all ControlChange (any channel)
// mapper[0] = NoteOn Ch 1 -> NoteOn Ch 2 (move channel, keep velocity)
// mapper[1] = NoteOn Ch 3 velocity 0..127 -> 0..63 (halve velocity)
// (Filter 1 / 2 must hit message types midi_stress.py actually emits, hence
// PB and CC rather than e.g. NoteOn-on-Ch10 — the stress fixture uses Ch1..4
// only and so a Ch10 rule would never fire under this fixture.)
//
// Rules 0 and 1 of each chain are disjoint (no incoming message matches
// both), so the mapper's first-match-wins semantics still let the x2 phase
// observe both rules acting independently on different message subsets.
void loadTestRules() {
  midiFilterRuleCount = 2;
  midiSelectedFilterRule = 0;
  midiFilterRules[0].enabled = false;
  midiFilterRules[0].kind = MIDI_KIND_PITCH_BEND;
  midiFilterRules[0].channel = -1;
  midiFilterRules[1].enabled = false;
  midiFilterRules[1].kind = MIDI_KIND_CONTROL_CHANGE;
  midiFilterRules[1].channel = -1;

  midiMapperRuleCount = 2;
  midiSelectedMapperRule = 0;
  midiMapperRules[0].enabled = false;
  midiMapperRules[0].srcKind = MIDI_KIND_NOTE_ON;
  midiMapperRules[0].srcChannel = 0;       // MIDI Ch 1
  midiMapperRules[0].srcData1 = -1;
  midiMapperRules[0].srcMin = 0;
  midiMapperRules[0].srcMax = 127;
  midiMapperRules[0].dstKind = MIDI_KIND_NOTE_ON;
  midiMapperRules[0].dstChannel = 1;       // MIDI Ch 2
  midiMapperRules[0].dstData1 = -1;
  midiMapperRules[0].dstMin = 0;
  midiMapperRules[0].dstMax = 127;

  midiMapperRules[1].enabled = false;
  midiMapperRules[1].srcKind = MIDI_KIND_NOTE_ON;
  midiMapperRules[1].srcChannel = 2;       // MIDI Ch 3
  midiMapperRules[1].srcData1 = -1;
  midiMapperRules[1].srcMin = 0;
  midiMapperRules[1].srcMax = 127;
  midiMapperRules[1].dstKind = MIDI_KIND_NOTE_ON;
  midiMapperRules[1].dstChannel = -1;      // keep
  midiMapperRules[1].dstData1 = -1;
  midiMapperRules[1].dstMin = 0;
  midiMapperRules[1].dstMax = 63;          // halve velocity
}

const char* getChannelLabel(int8_t channel, bool keepLabel) {
  static char label[8];
  if (channel < 0) {
    return keepLabel ? "KEEP" : "ALL";
  }
  snprintf(label, sizeof(label), "Ch%02d", channel + 1);
  return label;
}

const char* getData1Label(int16_t data1, bool keepLabel) {
  static char label[8];
  if (data1 < 0) {
    return keepLabel ? "KEEP" : "ANY";
  }
  snprintf(label, sizeof(label), "%d", data1);
  return label;
}

int getMidiPrimaryData(const MidiMessage& msg) {
  if (msg.length < 2) return 0;
  return msg.bytes[1];
}

int getMidiValueData(const MidiMessage& msg) {
  if (msg.kind == MIDI_KIND_PITCH_BEND || msg.kind == MIDI_KIND_SONG_POSITION) {
    if (msg.length < 3) return 0;
    return (msg.bytes[1] & 0x7F) | ((msg.bytes[2] & 0x7F) << 7);
  }
  if (msg.length >= 3) return msg.bytes[2];
  if (msg.length >= 2) return msg.bytes[1];
  return 0;
}

int mapMidiValueRange(int value, int srcMin, int srcMax, int dstMin, int dstMax) {
  srcMin = clampInt(srcMin, 0, 16383);
  srcMax = clampInt(srcMax, 0, 16383);
  dstMin = clampInt(dstMin, 0, 16383);
  dstMax = clampInt(dstMax, 0, 16383);

  if (srcMax < srcMin) srcMax = srcMin;
  if (dstMax < dstMin) dstMax = dstMin;

  value = clampInt(value, srcMin, srcMax);
  if (srcMax == srcMin) return dstMin;

  long num = (long)(value - srcMin) * (dstMax - dstMin);
  long den = (srcMax - srcMin);
  return dstMin + (int)(num / den);
}

bool midiFilterRuleMatches(const MidiFilterRule& rule, const MidiMessage& msg) {
  if (!rule.enabled) return false;
  if (rule.kind != msg.kind) return false;
  if (rule.channel >= 0) {
    if (!msg.hasChannel) return false;
    if (rule.channel != msg.channel) return false;
  }
  return true;
}

bool shouldAllowMidiMessage(const MidiMessage& msg) {
  if (midiFilterBypass) return true;
  for (int i = 0; i < midiFilterRuleCount; i++) {
    if (midiFilterRuleMatches(midiFilterRules[i], msg)) {
      return false;
    }
  }
  return true;
}

bool midiMapperRuleMatches(const MidiMapperRule& rule, const MidiMessage& msg) {
  if (!rule.enabled) return false;
  if (rule.srcKind != msg.kind) return false;
  if (rule.srcChannel >= 0) {
    if (!msg.hasChannel) return false;
    if (rule.srcChannel != msg.channel) return false;
  }
  if (rule.srcData1 >= 0 && getMidiPrimaryData(msg) != rule.srcData1) return false;

  int value = getMidiValueData(msg);
  if (value < rule.srcMin || value > rule.srcMax) return false;
  return true;
}

MidiMessage buildMappedMidiMessage(const MidiMapperRule& rule, const MidiMessage& srcMsg) {
  MidiMessage dstMsg = srcMsg;
  dstMsg.kind = rule.dstKind;
  dstMsg.hasChannel = midiKindHasChannel(rule.dstKind);
  dstMsg.length = getMidiMessageLengthForKind(rule.dstKind);

  uint8_t outChannel = 0;
  if (dstMsg.hasChannel) {
    if (rule.dstChannel >= 0) outChannel = rule.dstChannel;
    else if (srcMsg.hasChannel) outChannel = srcMsg.channel;
  }
  dstMsg.channel = dstMsg.hasChannel ? outChannel : -1;
  dstMsg.bytes[0] = getMidiStatusForKind(rule.dstKind, outChannel);

  int srcPrimary = getMidiPrimaryData(srcMsg);
  int srcValue = getMidiValueData(srcMsg);
  int mappedValue = mapMidiValueRange(srcValue, rule.srcMin, rule.srcMax, rule.dstMin, rule.dstMax);
  int primaryValue = (rule.dstData1 >= 0) ? rule.dstData1 : srcPrimary;

  if (dstMsg.length == 2) {
    if (midiKindSupportsData1(rule.dstKind)) {
      dstMsg.bytes[1] = clampInt(primaryValue, 0, 127);
    } else {
      dstMsg.bytes[1] = clampInt(mappedValue, 0, 127);
    }
  } else if (dstMsg.length == 3) {
    if (rule.dstKind == MIDI_KIND_PITCH_BEND || rule.dstKind == MIDI_KIND_SONG_POSITION) {
      int v = clampInt(mappedValue, 0, 16383);
      dstMsg.bytes[1] = v & 0x7F;
      dstMsg.bytes[2] = (v >> 7) & 0x7F;
    } else {
      dstMsg.bytes[1] = clampInt(primaryValue, 0, 127);
      dstMsg.bytes[2] = clampInt(mappedValue, 0, 127);
    }
  }

  return dstMsg;
}

MidiMessage applyMidiMapper(const MidiMessage& srcMsg) {
  if (midiMapperBypass) return srcMsg;
  for (int i = 0; i < midiMapperRuleCount; i++) {
    if (midiMapperRuleMatches(midiMapperRules[i], srcMsg)) {
      return buildMappedMidiMessage(midiMapperRules[i], srcMsg);
    }
  }
  return srcMsg;
}

void addDefaultFilterRule() {
  if (midiFilterRuleCount >= MAX_FILTER_RULES) return;
  MidiFilterRule& rule = midiFilterRules[midiFilterRuleCount];
  rule.enabled = false;
  rule.kind = MIDI_KIND_NOTE_OFF;
  rule.channel = -1;
  midiSelectedFilterRule = midiFilterRuleCount;
  midiFilterRuleCount++;
}

void addDefaultMapperRule() {
  if (midiMapperRuleCount >= MAX_MAPPER_RULES) return;
  MidiMapperRule& rule = midiMapperRules[midiMapperRuleCount];
  rule.enabled = false;
  rule.srcKind = MIDI_KIND_CONTROL_CHANGE;
  rule.srcChannel = -1;
  rule.srcData1 = -1;
  rule.srcMin = 0;
  rule.srcMax = 127;
  rule.dstKind = MIDI_KIND_CONTROL_CHANGE;
  rule.dstChannel = -1;
  rule.dstData1 = -1;
  rule.dstMin = 0;
  rule.dstMax = 127;
  midiSelectedMapperRule = midiMapperRuleCount;
  midiMapperRuleCount++;
}

void deleteSelectedFilterRule() {
  if (midiFilterRuleCount <= 1) {
    midiFilterRules[0].enabled = false;
    return;
  }
  for (int i = midiSelectedFilterRule; i < midiFilterRuleCount - 1; i++) {
    midiFilterRules[i] = midiFilterRules[i + 1];
  }
  midiFilterRuleCount--;
  if (midiSelectedFilterRule >= midiFilterRuleCount) {
    midiSelectedFilterRule = midiFilterRuleCount - 1;
  }
}

void deleteSelectedMapperRule() {
  if (midiMapperRuleCount <= 1) {
    midiMapperRules[0].enabled = false;
    return;
  }
  for (int i = midiSelectedMapperRule; i < midiMapperRuleCount - 1; i++) {
    midiMapperRules[i] = midiMapperRules[i + 1];
  }
  midiMapperRuleCount--;
  if (midiSelectedMapperRule >= midiMapperRuleCount) {
    midiSelectedMapperRule = midiMapperRuleCount - 1;
  }
}

bool isTransposeDisplayMode(int mode) {
  return mode == DIRECT_MODE || mode == KEY_MODE || mode == INSTANT_MODE || mode == SEQUENCE_MODE;
}

// --- BT bond persistence (SD card) ---

// BT bond save state
static bool g_btBondSaved = false;
static unsigned long g_btConnectedSince = 0;

// BT status overlay message
static char g_btOverlayMsg[24] = "";
static unsigned long g_btOverlayUntil = 0;

void showBTOverlay(const char* msg, uint16_t color, int durationMs = 2000) {
  strncpy(g_btOverlayMsg, msg, sizeof(g_btOverlayMsg) - 1);
  g_btOverlayMsg[sizeof(g_btOverlayMsg) - 1] = '\0';
  g_btOverlayUntil = millis() + durationMs;

  // 画面中央にオーバーレイ表示
  uiFontMedium();
  int w = M5.Lcd.textWidth(msg) + 24;
  int h = M5.Lcd.fontHeight() + 14;
  int x = (SCREEN_WIDTH - w) / 2;
  int y = (SCREEN_HEIGHT - h) / 2;
  M5.Lcd.fillRect(x, y, w, h, color);
  M5.Lcd.drawRect(x, y, w, h, WHITE);
  M5.Lcd.setTextColor(WHITE);
  uiDrawC(msg, x, y, w, h);
}

// Save BT bonding info from NVS to SD card
bool saveBTBondToSD() {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open(NVS_BT_NAMESPACE, NVS_READONLY, &nvs);
  if (err != ESP_OK) {
    Serial.printf("[BT_BOND] nvs_open failed: err=0x%x\n", err);
    return false;
  }

  uint8_t buffer[NVS_BT_MAX_SIZE];
  size_t totalSize = 0;

  for (int i = 0; i < 10; i++) {
    char key[20];
    snprintf(key, sizeof(key), "%s%d", NVS_BT_KEY_PREFIX, i);

    size_t chunkSize = 0;
    err = nvs_get_blob(nvs, key, NULL, &chunkSize);
    if (err != ESP_OK || chunkSize == 0) {
      Serial.printf("[BT_BOND] NVS key '%s': err=0x%x, size=%d\n", key, err, chunkSize);
      break;
    }
    if (totalSize + chunkSize > sizeof(buffer)) break;

    err = nvs_get_blob(nvs, key, buffer + totalSize, &chunkSize);
    if (err != ESP_OK) {
      Serial.printf("[BT_BOND] NVS read '%s' failed: err=0x%x\n", key, err);
      break;
    }
    Serial.printf("[BT_BOND] NVS key '%s': %d bytes OK\n", key, chunkSize);
    totalSize += chunkSize;
  }
  nvs_close(nvs);

  if (totalSize == 0) {
    Serial.println("[BT_BOND] No bond data in NVS yet");
    return false;
  }

  if (!ensureSDReady()) {
    Serial.println("[BT_BOND] SD card not ready");
    return false;
  }

  File file = SD.open(BT_BOND_FILE, FILE_WRITE);
  if (!file) {
    Serial.println("[BT_BOND] SD file open failed");
    return false;
  }
  size_t written = file.write(buffer, totalSize);
  file.close();

  if (written != totalSize) {
    Serial.printf("[BT_BOND] Write incomplete: %d/%d bytes\n", written, totalSize);
    return false;
  }

  Serial.printf("[BT_BOND] Saved to SD: %d bytes\n", totalSize);
  return true;
}

// Restore BT bonding info from SD card to NVS (call BEFORE Bluedroid init)
bool restoreBTBondFromSD() {
  if (!SD.exists(BT_BOND_FILE)) {
    Serial.println("[BT_BOND] restoreBTBondFromSD: file not found on SD");
    return false;
  }

  File file = SD.open(BT_BOND_FILE, FILE_READ);
  if (!file) {
    Serial.println("[BT_BOND] restoreBTBondFromSD: SD file open failed");
    return false;
  }

  size_t fileSize = file.size();
  if (fileSize == 0 || fileSize > NVS_BT_MAX_SIZE) {
    Serial.printf("[BT_BOND] restoreBTBondFromSD: invalid file size %d\n", fileSize);
    file.close();
    return false;
  }

  uint8_t buffer[NVS_BT_MAX_SIZE];
  file.read(buffer, fileSize);
  file.close();

  // Check if NVS already has valid BT config (don't overwrite if present)
  nvs_handle_t nvs;
  if (nvs_open(NVS_BT_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
    Serial.println("[BT_BOND] restoreBTBondFromSD: nvs_open failed");
    return false;
  }

  size_t existingSize = 0;
  esp_err_t err = nvs_get_blob(nvs, "bt_cfg_key0", NULL, &existingSize);
  if (err == ESP_OK && existingSize > 0) {
    nvs_close(nvs);
    Serial.println("[BT_BOND] NVS already has BT config, skipping SD restore");
    return true;
  }

  // Write data from SD to NVS in chunks
  size_t written = 0;
  int count = 0;
  while (written < fileSize) {
    char key[20];
    snprintf(key, sizeof(key), "%s%d", NVS_BT_KEY_PREFIX, count);

    size_t chunkSize = fileSize - written;
    if (chunkSize > NVS_BT_CHUNK_SIZE) chunkSize = NVS_BT_CHUNK_SIZE;

    if (nvs_set_blob(nvs, key, buffer + written, chunkSize) != ESP_OK) {
      Serial.printf("[BT_BOND] restoreBTBondFromSD: nvs_set_blob failed for %s\n", key);
      nvs_close(nvs);
      return false;
    }

    written += chunkSize;
    count++;
  }

  nvs_commit(nvs);
  nvs_close(nvs);

  Serial.printf("[BT_BOND] Restored from SD to NVS: %d bytes\n", fileSize);
  return true;
}

// --- End BT bond persistence ---

// --- Sequence pattern SD persistence ---

bool ensureSDReady() {
  // Try a simple test to see if SD is accessible
  File testFile = SD.open("/sd_test.tmp", FILE_WRITE);
  if (testFile) {
    testFile.close();
    SD.remove("/sd_test.tmp");
    return true;
  }
  // SD not ready - try re-initializing
  Serial.println("[SD] Re-initializing SD card...");
  SD.end();
  delay(100);
  if (SD.begin(4, SPI, 25000000)) {
    Serial.println("[SD] SD re-init OK");
    return true;
  }
  Serial.println("[SD] SD re-init FAILED");
  return false;
}

struct DeviceConfig {
  char defaultApp[16];
  char defaultTransposeMode[16];
  int  initialTranspose;
  int  transposeBase;
  bool initialAllNotesOff;
  bool initialFilterBypass;
  bool initialMapperBypass;
  char transposeRange[12];
  char midiInputSource[12];
  bool majorUpperTranspose;
  bool btAutoReconnect;
  bool showSplash;
};

DeviceConfig g_config;

void setDefaultConfig() {
  strncpy(g_config.defaultApp, "Transpose", sizeof(g_config.defaultApp));
  strncpy(g_config.defaultTransposeMode, "DIRECT", sizeof(g_config.defaultTransposeMode));
  g_config.initialTranspose = 0;
  g_config.transposeBase = 0;
  g_config.initialAllNotesOff = false;
  g_config.initialFilterBypass = true;
  g_config.initialMapperBypass = true;
  strncpy(g_config.transposeRange, "-5..6", sizeof(g_config.transposeRange));
  strncpy(g_config.midiInputSource, "MIX", sizeof(g_config.midiInputSource));
  g_config.majorUpperTranspose = false;
  g_config.btAutoReconnect = true;
  g_config.showSplash = true;
}

bool loadDeviceConfigFromSD() {
  if (SD.cardType() == CARD_NONE) {
    Serial.println("[CFG] no SD card — using defaults");
    return false;
  }
  if (!SD.exists("/config.json")) {
    Serial.println("[CFG] /config.json not found — using defaults");
    return false;
  }
  File f = SD.open("/config.json", FILE_READ);
  if (!f) {
    Serial.println("[CFG] open failed — using defaults");
    return false;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.printf("[CFG] parse error %s — using defaults\n", err.c_str());
    return false;
  }
  if (doc["DefaultApp"].is<const char*>())           strncpy(g_config.defaultApp, doc["DefaultApp"], sizeof(g_config.defaultApp));
  if (doc["DefaultTransposeMode"].is<const char*>()) strncpy(g_config.defaultTransposeMode, doc["DefaultTransposeMode"], sizeof(g_config.defaultTransposeMode));
  if (doc["InitialTranspose"].is<int>())             g_config.initialTranspose = doc["InitialTranspose"];
  if (doc["TransposeBase"].is<int>())                g_config.transposeBase = doc["TransposeBase"];
  if (doc["InitialAllNotesOff"].is<bool>())          g_config.initialAllNotesOff = doc["InitialAllNotesOff"];
  if (doc["InitialFilterBypass"].is<bool>())         g_config.initialFilterBypass = doc["InitialFilterBypass"];
  if (doc["InitialMapperBypass"].is<bool>())         g_config.initialMapperBypass = doc["InitialMapperBypass"];
  if (doc["TransposeRange"].is<const char*>())       strncpy(g_config.transposeRange, doc["TransposeRange"], sizeof(g_config.transposeRange));
  if (doc["MidiInputSource"].is<const char*>())      strncpy(g_config.midiInputSource, doc["MidiInputSource"], sizeof(g_config.midiInputSource));
  if (doc["MajorUpperTranspose"].is<bool>())         g_config.majorUpperTranspose = doc["MajorUpperTranspose"];
  if (doc["BTAutoReconnect"].is<bool>())             g_config.btAutoReconnect = doc["BTAutoReconnect"];
  if (doc["ShowSplash"].is<bool>())                  g_config.showSplash = doc["ShowSplash"];
  Serial.printf("[CFG] loaded: app=%s base=%d splash=%d\n",
                g_config.defaultApp, g_config.transposeBase, g_config.showSplash ? 1 : 0);
  return true;
}

bool saveDeviceConfigToSD() {
  if (!ensureSDReady()) {
    Serial.println("[CFG] save failed: SD not ready");
    return false;
  }
  JsonDocument doc;
  doc["DefaultApp"]            = g_config.defaultApp;
  doc["DefaultTransposeMode"]  = g_config.defaultTransposeMode;
  doc["InitialTranspose"]      = g_config.initialTranspose;
  doc["TransposeBase"]         = g_config.transposeBase;
  doc["InitialAllNotesOff"]    = g_config.initialAllNotesOff;
  doc["InitialFilterBypass"]   = g_config.initialFilterBypass;
  doc["InitialMapperBypass"]   = g_config.initialMapperBypass;
  doc["TransposeRange"]        = g_config.transposeRange;
  doc["MidiInputSource"]       = g_config.midiInputSource;
  doc["MajorUpperTranspose"]   = g_config.majorUpperTranspose;
  doc["BTAutoReconnect"]       = g_config.btAutoReconnect;
  doc["ShowSplash"]            = g_config.showSplash;
  File f = SD.open("/config.json", FILE_WRITE);
  if (!f) { Serial.println("[CFG] save: open failed"); return false; }
  size_t n = serializeJsonPretty(doc, f);
  f.close();
  Serial.printf("[CFG] saved %u bytes\n", (unsigned)n);
  return n > 0;
}

void applyDeviceConfig() {
  allNotesOffEnabled = g_config.initialAllNotesOff;
  midiFilterBypass = g_config.initialFilterBypass;
  midiMapperBypass = g_config.initialMapperBypass;
  if (strcmp(g_config.transposeRange, "0..11") == 0)        transposeRange = RANGE_0_TO_12;
  else if (strcmp(g_config.transposeRange, "-11..0") == 0)  transposeRange = RANGE_MINUS12_TO_0;
  else                                                       transposeRange = RANGE_MINUS5_TO_6;
  // initial transpose = base + offset (Phase 2 will use base in selection math)
  handleTransposeChange(clampTranspose((int8_t)(g_config.transposeBase + g_config.initialTranspose)));
  // Switch to default app
  if (strcmp(g_config.defaultApp, "Play") == 0
      || strcmp(g_config.defaultApp, "SMF") == 0
      || strcmp(g_config.defaultApp, "MP3") == 0) {
    enterDisplayMode(DIRECT_MODE);
  } else if (strcmp(g_config.defaultApp, "Filter") == 0) {
    midiManagePage = MIDI_PAGE_FILTER;
    enterDisplayMode(MIDI_MANAGE_MODE);
  } else if (strcmp(g_config.defaultApp, "Change") == 0) {
    midiManagePage = MIDI_PAGE_MAPPER;
    enterDisplayMode(MIDI_MANAGE_MODE);
  } else { // Transpose default
    if      (strcmp(g_config.defaultTransposeMode, "KEY") == 0)      enterDisplayMode(KEY_MODE);
    else if (strcmp(g_config.defaultTransposeMode, "INSTANT") == 0)  enterDisplayMode(INSTANT_MODE);
    else if (strcmp(g_config.defaultTransposeMode, "SEQUENCE") == 0) enterDisplayMode(SEQUENCE_MODE);
    else                                                              enterDisplayMode(DIRECT_MODE);
  }
}

bool saveSequencesToSD() {
  if (!ensureSDReady()) {
    Serial.println("[SEQ] Save failed: SD card not ready");
    return false;
  }

  File file = SD.open(SEQ_FILE, FILE_WRITE);
  if (!file) {
    Serial.println("[SEQ] Save failed: cannot open file");
    return false;
  }

  // Header: magic(2) + version(1) + patCount(1) + stepCount(1) + pad(1) = 6 bytes
  file.write('S'); file.write('Q');
  file.write((uint8_t)1);
  file.write((uint8_t)SEQ_PATTERN_COUNT);
  file.write((uint8_t)SEQ_STEP_COUNT);
  file.write((uint8_t)0);

  // Data: 16 patterns x 6 steps = 96 bytes
  for (int p = 0; p < SEQ_PATTERN_COUNT; p++) {
    file.write((const uint8_t*)seqPatterns[p], SEQ_STEP_COUNT);
  }

  file.close();
  Serial.printf("[SEQ] Saved %d patterns to SD\n", SEQ_PATTERN_COUNT);
  return true;
}

bool loadSequencesFromSD() {
  if (!SD.exists(SEQ_FILE)) {
    Serial.println("[SEQ] No sequence file on SD");
    return false;
  }

  File file = SD.open(SEQ_FILE, FILE_READ);
  if (!file) {
    Serial.println("[SEQ] Load failed: cannot open file");
    return false;
  }

  uint8_t header[6];
  if (file.read(header, 6) != 6) {
    Serial.println("[SEQ] Invalid header");
    file.close();
    return false;
  }

  if (header[0] != 'S' || header[1] != 'Q' || header[2] != 1 ||
      header[3] != SEQ_PATTERN_COUNT || header[4] != SEQ_STEP_COUNT) {
    Serial.println("[SEQ] Incompatible file format");
    file.close();
    return false;
  }

  for (int p = 0; p < SEQ_PATTERN_COUNT; p++) {
    file.read((uint8_t*)seqPatterns[p], SEQ_STEP_COUNT);
  }

  file.close();
  Serial.printf("[SEQ] Loaded %d patterns from SD\n", SEQ_PATTERN_COUNT);
  return true;
}

// --- End sequence pattern SD persistence ---

// Bluetooth HID foot pedal callback
void key_callback(uint8_t *p_msg)
{
    bool leftPressed = false;
    bool rightPressed = false;

    for (int i = 0; i < HID_L2CAP_MESSAGE_SIZE - 2; ++i) {
        uint8_t key = p_msg[2 + i];
        if (key == 0) {
            continue;
        }
        if (key == PEDAL_LEFT_KEY) {
            leftPressed = true;
        } else if (key == PEDAL_RIGHT_KEY) {
            rightPressed = true;
        }
    }

    portENTER_CRITICAL(&g_keyStateMux);
    if (leftPressed != g_leftPedalPressed || rightPressed != g_rightPedalPressed) {
        g_leftPedalPressed = leftPressed;
        g_rightPedalPressed = rightPressed;
        g_pedalNeedsProcessing = true;
    }
    portEXIT_CRITICAL(&g_keyStateMux);
}

// ---- 起動スプラッシュ "OWAMIDICON" ----
static void showSplashScreen() {
  // Mirrors the M5Tab-MIDIXposeFil opening: double frame, deco lines with
  // corner accent dots, faded title, subtitle, progress bar, footer.
  // Geometry/font scaled down for the 320×240 panel; animation timing,
  // fade math, and progress-bar logic match Tab5's drawSplash() 1:1.
  const int W = SCREEN_WIDTH;
  const int H = SCREEN_HEIGHT;
  const int cx = W / 2;
  const int cy = H / 2;

  M5.Lcd.fillScreen(TFT_BLACK);

  // Double frame (Tab5 used inset 50/54 on 1280×720 → 12/15 here).
  M5.Lcd.drawRect(12, 12, W - 24, H - 24, 0x2104);
  M5.Lcd.drawRect(15, 15, W - 30, H - 30, 0x18C3);

  // Decorative lines flanking the title with accent dots at the corners.
  // Tab5: cy ± 90, length 640. Scaled here: cy ± 30, length 200.
  const int hlHalf = 100;
  M5.Lcd.drawFastHLine(cx - hlHalf, cy - 30, hlHalf * 2, 0x39E7);
  M5.Lcd.drawFastHLine(cx - hlHalf, cy + 30, hlHalf * 2, 0x39E7);
  M5.Lcd.fillCircle(cx - hlHalf, cy - 30, 2, TFT_CYAN);
  M5.Lcd.fillCircle(cx + hlHalf, cy - 30, 2, TFT_CYAN);
  M5.Lcd.fillCircle(cx - hlHalf, cy + 30, 2, TFT_CYAN);
  M5.Lcd.fillCircle(cx + hlHalf, cy + 30, 2, TFT_CYAN);

  // Subtitle just above the lower deco line. FSS9 (≈ Tab5's FONT_MED at
  // half size) — "少し小さく" per request.
  uiFontSmall();
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.setTextColor(0x8C71, TFT_BLACK);
  M5.Lcd.drawString("MIDI Transposer - Mapper - BT Pedal", cx, cy + 18);

  // Footer (analogue of Tab5's FONT_TINY line).
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(0x6B4D, TFT_BLACK);
  M5.Lcd.drawString("for M5Stack Core2", cx, cy + 66);

  // Progress bar frame (taller than Tab5's 4 px so the 2 px fill is visible).
  const int pbW = 200;
  const int pbH = 6;
  const int pbX = cx - pbW / 2;
  const int pbY = cy + 50;
  M5.Lcd.drawRoundRect(pbX, pbY, pbW, pbH, 2, 0x4208);

  const uint32_t totalMs = 3000;
  const uint32_t startMs = millis();

  uiFontMedium();   // FSSB12 — "少し小さい" analogue of Tab5's FSSB24 title
  M5.Lcd.setTextDatum(MC_DATUM);

  int lastFill = -1;
  uint16_t lastTitleCol = 0xFFFE;  // unlikely value, forces first draw

  while (true) {
    uint32_t e = millis() - startMs;
    if (e >= totalMs) break;

    // Title fades in over the first 700 ms, holds, then fades out the last 300 ms.
    uint8_t lum;
    if (e < 700)                lum = (uint8_t)((uint32_t)255 * e / 700);
    else if (e > totalMs - 300) lum = (uint8_t)((uint32_t)255 * (totalMs - e) / 300);
    else                        lum = 255;

    uint16_t tcol = M5.Lcd.color565(lum, lum, lum);
    if (tcol != lastTitleCol) {
      M5.Lcd.setTextColor(tcol, TFT_BLACK);
      M5.Lcd.drawString("OWAMIDICON-Core2", cx, cy - 10);
      lastTitleCol = tcol;
    }

    int fill = (int)((uint64_t)(pbW - 4) * e / totalMs);
    if (fill != lastFill) {
      if (fill > 0) M5.Lcd.fillRoundRect(pbX + 2, pbY + 2, fill, pbH - 4, 1, TFT_CYAN);
      lastFill = fill;
    }
    delay(16);
  }
}

void setup() {
  M5.begin(true, true, true, true);

  // LCD に直接描画する (PSRAM スプライトの fillRect memcpy バグを回避)。
  // スクリーンショットも公式 M5Stack の TFT_Screen_Capture と同様、
  // LCD GRAM から行ごと readRectRGB で吸い出す。

  // for SD-Updater
  checkSDUpdater( SD, MENU_BIN, 2000, TFCARD_CS_PIN );

  Serial.begin(115200);
  // setRxBufferSize / setTxBufferSize must run BEFORE begin() — the
  // arduino-esp32 implementation early-returns once the UART driver is
  // installed (HardwareSerial.cpp::setTxBufferSize: `if (_uart) return 0;`).
  // The previous order silently no-op'd both calls, so Serial2 ran with
  // the default 256-byte RX ring and NO TX ring at all (just the ~128-byte
  // hardware FIFO). At 31.25 kbaud that overflowed almost immediately when
  // a keyboard player drove sustained MIDI input, making every Serial2.write()
  // block waiting for the FIFO to drain — starving the loop and tripping the
  // task watchdog.
  Serial2.setRxBufferSize(2048);
  Serial2.setTxBufferSize(4096);
  Serial2.begin(31250, SERIAL_8N1, RXD2, TXD2);
  
  // ノート状態の初期化
  for (int i = 0; i < TRACKED_NOTE_STATE_COUNT; i++) {
    currentNoteStates[i].isActive = false;
    currentNoteStates[i].originalTranspose = 0;
    currentNoteStates[i].channel = 0;
    currentNoteStates[i].velocity = 0;
    
    savedNoteStates[i].isActive = false;
    savedNoteStates[i].originalTranspose = 0;
    savedNoteStates[i].channel = 0;
    savedNoteStates[i].velocity = 0;
  }
  
  // UI要素の初期化
  initDirectModeButtons();
  initKeyModeButtons();
  initInstantModeButtons();
  initSequenceModeButtons();
  initMidiManagementDefaults();

  // SDカード状態確認
  if (SD.cardType() == CARD_NONE) {
    Serial.println("[SD] WARNING: No SD card detected!");
  } else {
    Serial.printf("[SD] Card type: %d, Size: %lluMB\n", SD.cardType(), SD.cardSize() / (1024 * 1024));
  }

  // SDカードからシーケンスパターンを読み込み
  Serial.println("Loading sequence patterns from SD...");
  loadSequencesFromSD();
  setDefaultConfig();
  loadDeviceConfigFromSD();

  // 最初の転調値0のボタンを確実に光らせる（-5から+6レンジでは5番目のボタン）
  transposeButtons[5] = true;  // RANGE_MINUS5_TO_6の場合、ボタン5が転調値0
  
  // Restore BT bond info from SD card to NVS (BEFORE Bluedroid init)
  Serial.println("Checking for saved BT bond info on SD card...");
  restoreBTBondFromSD();

  // Initialize Bluetooth HID for foot pedal
  Serial.println("Initializing Bluetooth HID...");
  long ret = hid_l2cap_initialize(key_callback);
  if (ret != 0) {
    Serial.println("hid_l2cap_initialize failed");
  } else {
    BD_ADDR addr = TARGET_BT_ADDR;
    g_lastReconnectAttempt = millis();
    ret = hid_l2cap_connect(addr);
    if (ret < 0) {
      Serial.println("hid_l2cap_connect failed");
    } else {
      Serial.println("Bluetooth HID initialized successfully");
    }
  }

  M5.Lcd.fillScreen(BLACK);
  if (g_config.showSplash) {
    showSplashScreen();
  }
  applyDeviceConfig();
  drawInterface();

  Serial.println("MIDI Transposer Ready!");
  Serial.printf("Initial transpose: %d, Button 5 state: %s\n", transposeValue, transposeButtons[5] ? "ON" : "OFF");
  Serial.println("USB serial command interface ready. Type HELP for commands.");

#ifdef M5TAB_DIAG
  Serial.printf("[boot] Core2 MIDIXposeFilBT ready  panel=%dx%d  reset_reason=%d\n",
                M5.Lcd.width(), M5.Lcd.height(), (int)esp_reset_reason());
  Serial.printf("[boot] heap=%u psram=%u\n",
                (unsigned)esp_get_free_heap_size(),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#endif
}

#ifdef M5TAB_DIAG
// Lightweight memory-leak monitor. Sampled once per loop() iteration so a
// transient drop between reports is still captured. Prints every 5 s. The
// signal that matters most is `all_min`: if it stops dropping after warm-up
// and stays flat, no leak.
static uint32_t g_diagLastReportMs = 0;
static uint32_t g_diagWinMinHeap   = UINT32_MAX;

static inline void diagSample() {
  uint32_t heap = (uint32_t)esp_get_free_heap_size();
  if (heap < g_diagWinMinHeap) g_diagWinMinHeap = heap;
}

static inline void diagMaybeReport(uint32_t nowMs) {
  if (nowMs - g_diagLastReportMs < 5000) return;
  uint32_t heapNow  = (uint32_t)esp_get_free_heap_size();
  uint32_t allMin   = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
  uint32_t psramNow = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  uint32_t psramMin = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
  uint32_t stackHW  = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
  // midiInCount / midiOutCount let us prove the device is actually receiving
  // and emitting MIDI bytes during a stress test — heap stats alone don't
  // tell you whether processMIDI() saw any input. A flat all_min with a flat
  // midi_in is a deaf box, not a leak-free one.
  Serial.printf("[mem] heap=%u win_min=%u all_min=%u psram=%u psram_min=%u stack_hw=%u midi_in=%lu midi_out=%lu uptime_ms=%u\n",
                (unsigned)heapNow, (unsigned)g_diagWinMinHeap,
                (unsigned)allMin, (unsigned)psramNow,
                (unsigned)psramMin, (unsigned)stackHW,
                midiInCount, midiOutCount, (unsigned)nowMs);
  g_diagLastReportMs = nowMs;
  g_diagWinMinHeap   = heapNow;
}
#else
static inline void diagSample() {}
static inline void diagMaybeReport(uint32_t /*nowMs*/) {}
#endif

void loop() {
  diagSample();
  processMIDI();

  static unsigned long lastUICheck = 0;
  unsigned long now = millis();
  diagMaybeReport((uint32_t)now);

  if (now - lastUICheck >= 20) {
    M5.update();
    processUsbSerialCommands();
    processHardwareButtons();
    processTouch();
    processFootPedal();

    // Bluetooth HID connection management
    BT_STATUS status = hid_l2cap_is_connected();
    if (status == BT_CONNECTING) {
      if ((now - g_lastReconnectAttempt) >= BT_CONNECT_TIMEOUT) {
        g_lastReconnectAttempt = now;
        Serial.println("Retry connect (timeout)");
        hid_l2cap_reconnect();
      }
    } else if (status == BT_DISCONNECTED) {
      if ((now - g_lastReconnectAttempt) >= 5000) {
        g_lastReconnectAttempt = now;
        Serial.println("Retry connect (disconnected)");
        hid_l2cap_reconnect();
      }
      g_btConnectedSince = 0;
    }

    // BT接続状態が変わったら画面更新
    if (status != g_lastStatusDrawn) {
      g_lastStatusDrawn = status;
      needFullRedraw = true;
      if (status == BT_CONNECTED) {
        showBTOverlay("BT Connected", 0x03E0, 1500); // dark green
      } else if (status == BT_DISCONNECTED) {
        showBTOverlay("BT Disconnected", RED, 1500);
      }
    }

    // 新しい認証（ペアリング）完了時
    if (hid_l2cap_auth_completed()) {
      Serial.println("[BT_BOND] Auth completed - saving bond to SD...");
      g_btBondSaved = false;
      g_btConnectedSince = 0;
      showBTOverlay("BT Paired!", BLUE, 1500);
    }

    // BT接続安定後（3秒）にボンド情報をSDに保存
    if (status == BT_CONNECTED) {
      if (g_btConnectedSince == 0) {
        g_btConnectedSince = now;
      }
      if (!g_btBondSaved && (now - g_btConnectedSince) >= 3000) {
        g_btBondSaved = true;
        g_btBondSavePending = true;
        Serial.println("[BT_BOND] 3s stable - bond save queued");
      }
    }

    processDeferredStorageTasks(now);

    // オーバーレイ表示期限切れで画面再描画
    if (g_btOverlayUntil > 0 && now >= g_btOverlayUntil) {
      g_btOverlayUntil = 0;
      needFullRedraw = true;
    }

    if (needFullRedraw) {
      drawInterface();
      needFullRedraw = false;
      needPartialUpdate = false;
    } else if (needPartialUpdate) {
      updateStatusArea();
      needPartialUpdate = false;
    }

    lastUICheck = now;
  }
}

void initDirectModeButtons() {
  int buttonWidth = 75;
  int buttonHeight = 60;
  int startX = 10;
  int startY = 46;  // ヘッダ(y=0..40)直下から
  int spacingX = 5;
  int spacingY = 4;
  int buttonsPerRow = 4;  // 4x3レイアウト
  
  for (int i = 0; i < 12; i++) {
    int row = i / buttonsPerRow;
    int col = i % buttonsPerRow;
    
    directButtons[i].x = startX + col * (buttonWidth + spacingX);
    directButtons[i].y = startY + row * (buttonHeight + spacingY);
    directButtons[i].w = buttonWidth;
    directButtons[i].h = buttonHeight;
    
    updateDirectButtonLabels();
    
    transposeButtons[i] = false; // 初期は何も選択しない
  }
}

// 現在の転調値に対応するボタンを選択状態にする
void setCurrentTransposeButton() {
  // 全てのボタンを一旦リセット
  for (int i = 0; i < 12; i++) {
    transposeButtons[i] = false;
  }
  
  // 現在の転調値に対応するボタンを探して選択状態にする
  for (int i = 0; i < 12; i++) {
    if (directButtons[i].value == transposeValue) {
      transposeButtons[i] = true;
      break;
    }
  }
}

void updateDirectButtonLabels() {
  static char labels[12][4];
  
  for (int i = 0; i < 12; i++) {
    if (transposeRange == RANGE_0_TO_12) {
      // 0から+11
      if (i == 0) {
        sprintf(labels[i], "0");
      } else {
        sprintf(labels[i], "+%d", i);
      }
      directButtons[i].value = i;
    } else if (transposeRange == RANGE_MINUS12_TO_0) {
      // -11から0
      if (i == 11) {
        sprintf(labels[i], "0");
      } else {
        sprintf(labels[i], "%d", i - 11);
      }
      directButtons[i].value = i - 11;
    } else { // RANGE_MINUS5_TO_6
      // -5から+6
      int value = i - 5;
      if (value > 0) {
        sprintf(labels[i], "+%d", value);
      } else {
        sprintf(labels[i], "%d", value);
      }
      directButtons[i].value = value;
    }
    directButtons[i].label = labels[i];
  }
}

void initKeyModeButtons() {
  const char* keyNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

  int whiteKeyWidth = 44;
  int blackKeyWidth = 28;
  int whiteKeyHeight = 78;
  int blackKeyHeight = 52;
  int startX = 10;

  // メジャーキー（上段）: ラベル y=44 直下、鍵盤 y=58
  int majorY = 58;
  int whiteKeyIndex = 0;
  
  for (int i = 0; i < 12; i++) {
    majorKeys[i].keyValue = i;
    majorKeys[i].keyName = keyNames[i];
    majorKeys[i].isBlackKey = (i == 1 || i == 3 || i == 6 || i == 8 || i == 10);
    
    if (majorKeys[i].isBlackKey) {
      majorKeys[i].x = startX + (whiteKeyIndex * whiteKeyWidth) - (blackKeyWidth / 2);
      majorKeys[i].y = majorY;
      majorKeys[i].w = blackKeyWidth;
      majorKeys[i].h = blackKeyHeight;
    } else {
      majorKeys[i].x = startX + (whiteKeyIndex * whiteKeyWidth);
      majorKeys[i].y = majorY;
      majorKeys[i].w = whiteKeyWidth;
      majorKeys[i].h = whiteKeyHeight;
      whiteKeyIndex++;
    }
  }
  
  // マイナーキー（下段）: ラベル y=140、鍵盤 y=156、画面下端 y=234 まで
  int minorY = 156;
  whiteKeyIndex = 0;
  
  for (int i = 0; i < 12; i++) {
    minorKeys[i].keyValue = i;
    minorKeys[i].keyName = keyNames[i];
    minorKeys[i].isBlackKey = (i == 1 || i == 3 || i == 6 || i == 8 || i == 10);
    
    if (minorKeys[i].isBlackKey) {
      minorKeys[i].x = startX + (whiteKeyIndex * whiteKeyWidth) - (blackKeyWidth / 2);
      minorKeys[i].y = minorY;
      minorKeys[i].w = blackKeyWidth;
      minorKeys[i].h = blackKeyHeight;
    } else {
      minorKeys[i].x = startX + (whiteKeyIndex * whiteKeyWidth);
      minorKeys[i].y = minorY;
      minorKeys[i].w = whiteKeyWidth;
      minorKeys[i].h = whiteKeyHeight;
      whiteKeyIndex++;
    }
  }
}

// 相対モードのボタン配置（絶対指定モード：押した値に転調）
// レイアウト:
//   上段の上: 0ボタン（ボタン2個分の幅で中央）
//   上段: +1, +2, +3, +5
//   下段: -1, -2, -3, -5
void initInstantModeButtons() {
  static const int8_t values[8] = {+1, +2, +3, +5, -1, -2, -3, -5};
  static char labels[8][4];

  int buttonWidth = 75;
  int buttonHeight = 58;
  int startX = 10;
  int startY = 114;
  int spacingX = 5;
  int spacingY = 6;
  int buttonsPerRow = 4;

  for (int i = 0; i < 8; i++) {
    int row = i / buttonsPerRow;
    int col = i % buttonsPerRow;

    instantButtons[i].x = startX + col * (buttonWidth + spacingX);
    instantButtons[i].y = startY + row * (buttonHeight + spacingY);
    instantButtons[i].w = buttonWidth;
    instantButtons[i].h = buttonHeight;
    instantButtons[i].value = values[i];

    if (values[i] > 0) sprintf(labels[i], "+%d", values[i]);
    else sprintf(labels[i], "%d", values[i]);
    instantButtons[i].label = labels[i];
  }

  // 0 ボタン（ボタン2個分の幅、上段の上に中央配置）
  int zeroW = buttonWidth * 2 + spacingX;  // 155
  int zeroH = 44;
  instantZeroBtn.x = (SCREEN_WIDTH - zeroW) / 2;
  instantZeroBtn.y = startY - zeroH - 4;
  instantZeroBtn.w = zeroW;
  instantZeroBtn.h = zeroH;
}

// シーケンスモードのボタン配置
void initSequenceModeButtons() {
  memset(seqPatterns, 0, sizeof(seqPatterns));

  int slotWidth = 48;
  int slotSpacing = 5;
  int startX = 5;

  // レイアウト (ヘッダ y=0..40 の下から):
  //   パターン選択(h=30) → 上ボタン(h=26) → 値(h=36) → 下ボタン(h=26) → ステップ移動(h=42)
  int patRowY = 46;   int patRowH = 30;
  int upBtnY  = 80;   int upBtnH  = 26;
  int slotY   = 110;  int slotH   = 36;
  int downBtnY = 150;  int downBtnH = 26;
  int navY    = 182;  int navH    = 42;

  for (int i = 0; i < SEQ_STEP_COUNT; i++) {
    int x = startX + i * (slotWidth + slotSpacing);

    seqSteps[i].upBtnX = x;    seqSteps[i].upBtnY = upBtnY;
    seqSteps[i].upBtnW = slotWidth; seqSteps[i].upBtnH = upBtnH;

    seqSteps[i].slotX = x;     seqSteps[i].slotY = slotY;
    seqSteps[i].slotW = slotWidth;  seqSteps[i].slotH = slotH;

    seqSteps[i].downBtnX = x;  seqSteps[i].downBtnY = downBtnY;
    seqSteps[i].downBtnW = slotWidth; seqSteps[i].downBtnH = downBtnH;
  }

  // パターン選択ボタン（大きめ）
  seqPatLeftBtn.x = 5;    seqPatLeftBtn.y = patRowY;
  seqPatLeftBtn.w = 50;   seqPatLeftBtn.h = patRowH;

  seqPatRightBtn.x = 165; seqPatRightBtn.y = patRowY;
  seqPatRightBtn.w = 50;  seqPatRightBtn.h = patRowH;

  // SAVEボタン（大きめ）
  seqSaveBtn.x = 225; seqSaveBtn.y = patRowY;
  seqSaveBtn.w = 90;  seqSaveBtn.h = patRowH;

  // ステップ移動ボタン
  seqStepLeftBtn.x = 10;   seqStepLeftBtn.y = navY;
  seqStepLeftBtn.w = 80;   seqStepLeftBtn.h = navH;

  seqStepRightBtn.x = 230; seqStepRightBtn.y = navY;
  seqStepRightBtn.w = 80;  seqStepRightBtn.h = navH;
}

void drawInterface() {
  M5.Lcd.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);

  uiFontSmall();

  // ── Row 1 (y=2) : タイトル + 転調値 ──
  M5.Lcd.setTextColor(WHITE);
  uiDrawL(currentMode == MIDI_MANAGE_MODE ? "MIDI Manager" : "MIDI Transposer", 10, 2);

  // ── Row 2 (y=22) : AllOff | BT | I/O | ボタン補助 ──
  M5.Lcd.setTextColor(allNotesOffEnabled ? GREEN : RED);
  {
    char buf[16];
    snprintf(buf, sizeof(buf), "AllOff:%s", allNotesOffEnabled ? "ON " : "OFF");
    uiDrawL(buf, 10, 22);
  }
  {
    BT_STATUS btSt = hid_l2cap_is_connected();
    uint16_t btColor;
    const char* btLabel;
    if (btSt == BT_CONNECTED)    { btColor = GREEN;  btLabel = "BT:ON"; }
    else if (btSt == BT_CONNECTING) { btColor = YELLOW; btLabel = "BT:.."; }
    else                            { btColor = RED;     btLabel = "BT:--"; }
    M5.Lcd.setTextColor(btColor);
    uiDrawL(btLabel, 96, 22);
  }
  M5.Lcd.setTextColor(DARKGREY);
  {
    const char* hint;
    if (currentMode == MIDI_MANAGE_MODE) {
      hint = (midiManagePage == MIDI_PAGE_FILTER) ? "B:Type  Hold:Grp" : "B:PG1/2  Hold:Grp";
    } else {
      hint = "B:Action  Hold:Grp";
    }
    uiDrawL(hint, 200, 22);
  }

  // ── ヘッダ下の区切り線 (y=40) ──
  M5.Lcd.drawFastHLine(0, 40, SCREEN_WIDTH, DARKGREY);

  if (currentMode == DIRECT_MODE) {
    drawDirectMode();
  } else if (currentMode == KEY_MODE) {
    drawKeyMode();
  } else if (currentMode == INSTANT_MODE) {
    drawInstantMode();
  } else if (currentMode == SEQUENCE_MODE) {
    drawSequenceMode();
  } else if (currentMode == CONFIG_EDIT_MODE) {
    drawConfigEditMode();
  } else if (currentMode == BASE_SET_MODE) {
    drawBaseSetMode();
  } else {
    drawMidiManageMode();
  }

  updateStatusArea();
}

void drawDirectMode() {
  uiFontLarge();
  for (int i = 0; i < 12; i++) {
    uint16_t color = transposeButtons[i] ? GREEN : DARKGREY;
    uint16_t textColor = transposeButtons[i] ? BLACK : WHITE;

    M5.Lcd.fillRect(directButtons[i].x, directButtons[i].y,
                    directButtons[i].w, directButtons[i].h, color);
    M5.Lcd.drawRect(directButtons[i].x, directButtons[i].y,
                    directButtons[i].w, directButtons[i].h, WHITE);

    M5.Lcd.setTextColor(textColor);
    uiDrawC(directButtons[i].label,
            directButtons[i].x, directButtons[i].y,
            directButtons[i].w, directButtons[i].h);
  }
}

void drawKeyMode() {
  uiFontSmall();
  M5.Lcd.setTextColor(CYAN);
  // ラベルは各鍵盤の直上 (鍵盤の y - 14) に置く
  uiDrawL(majorUpperTranspose ? "Major Keys (Upper +):" : "Major Keys (to C):", 10, 44);
  uiDrawL(minorUpperTranspose ? "Minor Keys (Lower -):" : "Minor Keys (to Am):", 10, 142);
  
  int correspondingMajorKey = -1;
  int correspondingMinorKey = -1;
  
  if (majorUpperTranspose) {
    if (transposeValue >= 1 && transposeValue <= 12) {
      correspondingMajorKey = 12 - transposeValue;
    }
  } else {
    if (transposeValue <= 0 && transposeValue >= -11) {
      correspondingMajorKey = -transposeValue;
    }
  }
  
  if (minorUpperTranspose) {
    if (transposeValue <= -1 && transposeValue >= -9) {
      correspondingMinorKey = 9 + transposeValue;
    }
  } else {
    if (transposeValue >= -2 && transposeValue <= 9) {
      correspondingMinorKey = 9 - transposeValue;
    }
  }
  
  uiFontSmall();
  M5.Lcd.setTextDatum(BC_DATUM); // bottom-center: ラベルを鍵盤下端に揃える
  for (int i = 0; i < 12; i++) {
    if (!majorKeys[i].isBlackKey) {
      uint16_t color;
      uint16_t textColor;
      if (selectedMajorKey == i) { color = GREEN; textColor = BLACK; }
      else if (correspondingMajorKey == i) { color = 0x07E0; textColor = BLACK; }
      else { color = WHITE; textColor = BLACK; }
      M5.Lcd.fillRect(majorKeys[i].x, majorKeys[i].y, majorKeys[i].w, majorKeys[i].h, color);
      M5.Lcd.drawRect(majorKeys[i].x, majorKeys[i].y, majorKeys[i].w, majorKeys[i].h, DARKGREY);
      M5.Lcd.setTextColor(textColor);
      M5.Lcd.drawString(majorKeys[i].keyName,
                        majorKeys[i].x + majorKeys[i].w / 2,
                        majorKeys[i].y + majorKeys[i].h - 4);
    }
  }
  for (int i = 0; i < 12; i++) {
    if (majorKeys[i].isBlackKey) {
      uint16_t color;
      uint16_t textColor = WHITE;
      if (selectedMajorKey == i) { color = GREEN; textColor = BLACK; }
      else if (correspondingMajorKey == i) { color = 0x07E0; textColor = BLACK; }
      else { color = DARKGREY; textColor = WHITE; }
      M5.Lcd.fillRect(majorKeys[i].x, majorKeys[i].y, majorKeys[i].w, majorKeys[i].h, color);
      M5.Lcd.drawRect(majorKeys[i].x, majorKeys[i].y, majorKeys[i].w, majorKeys[i].h, WHITE);
      M5.Lcd.setTextColor(textColor);
      M5.Lcd.drawString(majorKeys[i].keyName,
                        majorKeys[i].x + majorKeys[i].w / 2,
                        majorKeys[i].y + majorKeys[i].h - 4);
    }
  }
  for (int i = 0; i < 12; i++) {
    if (!minorKeys[i].isBlackKey) {
      uint16_t color;
      uint16_t textColor;
      if (selectedMinorKey == i) { color = ORANGE; textColor = BLACK; }
      else if (correspondingMinorKey == i) { color = 0xFD20; textColor = BLACK; }
      else { color = WHITE; textColor = BLACK; }
      M5.Lcd.fillRect(minorKeys[i].x, minorKeys[i].y, minorKeys[i].w, minorKeys[i].h, color);
      M5.Lcd.drawRect(minorKeys[i].x, minorKeys[i].y, minorKeys[i].w, minorKeys[i].h, DARKGREY);
      M5.Lcd.setTextColor(textColor);
      M5.Lcd.drawString(minorKeys[i].keyName,
                        minorKeys[i].x + minorKeys[i].w / 2,
                        minorKeys[i].y + minorKeys[i].h - 4);
    }
  }
  for (int i = 0; i < 12; i++) {
    if (minorKeys[i].isBlackKey) {
      uint16_t color;
      uint16_t textColor = WHITE;
      if (selectedMinorKey == i) { color = ORANGE; textColor = BLACK; }
      else if (correspondingMinorKey == i) { color = 0xFD20; textColor = BLACK; }
      else { color = DARKGREY; textColor = WHITE; }
      M5.Lcd.fillRect(minorKeys[i].x, minorKeys[i].y, minorKeys[i].w, minorKeys[i].h, color);
      M5.Lcd.drawRect(minorKeys[i].x, minorKeys[i].y, minorKeys[i].w, minorKeys[i].h, WHITE);
      M5.Lcd.setTextColor(textColor);
      M5.Lcd.drawString(minorKeys[i].keyName,
                        minorKeys[i].x + minorKeys[i].w / 2,
                        minorKeys[i].y + minorKeys[i].h - 4);
    }
  }
}

// 相対モード描画（絶対指定モード：押した値に直接転調）
void drawInstantMode() {
  // 説明
  uiFontSmall();
  M5.Lcd.setTextColor(CYAN);
  uiDrawL("Instant Mode: tap to set value", 10, 46);

  // 0 ボタン（上段の上、ボタン2個分幅）
  bool zeroActive = (transposeValue == 0);
  uint16_t zeroColor = zeroActive ? GREEN : DARKGREY;
  uint16_t zeroTxt   = zeroActive ? BLACK : WHITE;
  M5.Lcd.fillRect(instantZeroBtn.x, instantZeroBtn.y,
                  instantZeroBtn.w, instantZeroBtn.h, zeroColor);
  M5.Lcd.drawRect(instantZeroBtn.x, instantZeroBtn.y,
                  instantZeroBtn.w, instantZeroBtn.h, WHITE);
  uiFontHuge();
  M5.Lcd.setTextColor(zeroTxt);
  uiDrawC("0", instantZeroBtn.x, instantZeroBtn.y,
          instantZeroBtn.w, instantZeroBtn.h);

  // +1,+2,+3,+5 / -1,-2,-3,-5 ボタン群
  uiFontLarge();
  for (int i = 0; i < 8; i++) {
    bool active = (instantButtons[i].value == transposeValue);
    uint16_t color = active ? GREEN : DARKGREY;
    uint16_t txt   = active ? BLACK : WHITE;
    M5.Lcd.fillRect(instantButtons[i].x, instantButtons[i].y,
                    instantButtons[i].w, instantButtons[i].h, color);
    M5.Lcd.drawRect(instantButtons[i].x, instantButtons[i].y,
                    instantButtons[i].w, instantButtons[i].h, WHITE);
    M5.Lcd.setTextColor(txt);
    uiDrawC(instantButtons[i].label,
            instantButtons[i].x, instantButtons[i].y,
            instantButtons[i].w, instantButtons[i].h);
  }
}

// シーケンスモード描画
void drawSequenceMode() {
  // パターン選択行（大きめボタン）
  // 左矢印ボタン
  M5.Lcd.fillRect(seqPatLeftBtn.x, seqPatLeftBtn.y,
                  seqPatLeftBtn.w, seqPatLeftBtn.h, BLUE);
  M5.Lcd.drawRect(seqPatLeftBtn.x, seqPatLeftBtn.y,
                  seqPatLeftBtn.w, seqPatLeftBtn.h, WHITE);
  uiFontLarge();
  M5.Lcd.setTextColor(WHITE);
  uiDrawC("<", seqPatLeftBtn.x, seqPatLeftBtn.y, seqPatLeftBtn.w, seqPatLeftBtn.h);

  // パターン番号表示
  int patDispX = seqPatLeftBtn.x + seqPatLeftBtn.w + 3;
  int patDispW = seqPatRightBtn.x - patDispX - 3;
  M5.Lcd.fillRect(patDispX, seqPatLeftBtn.y, patDispW, seqPatLeftBtn.h, NAVY);
  M5.Lcd.drawRect(patDispX, seqPatLeftBtn.y, patDispW, seqPatLeftBtn.h, WHITE);
  char patStr[20];
  snprintf(patStr, sizeof(patStr), "Pat %02d/%02d", seqCurrentPattern + 1, SEQ_PATTERN_COUNT);
  uiFontMedium();
  M5.Lcd.setTextColor(YELLOW);
  uiDrawC(patStr, patDispX, seqPatLeftBtn.y, patDispW, seqPatLeftBtn.h);

  // 右矢印ボタン
  M5.Lcd.fillRect(seqPatRightBtn.x, seqPatRightBtn.y,
                  seqPatRightBtn.w, seqPatRightBtn.h, BLUE);
  M5.Lcd.drawRect(seqPatRightBtn.x, seqPatRightBtn.y,
                  seqPatRightBtn.w, seqPatRightBtn.h, WHITE);
  uiFontLarge();
  M5.Lcd.setTextColor(WHITE);
  uiDrawC(">", seqPatRightBtn.x, seqPatRightBtn.y, seqPatRightBtn.w, seqPatRightBtn.h);

  // SAVEボタン
  M5.Lcd.fillRect(seqSaveBtn.x, seqSaveBtn.y,
                  seqSaveBtn.w, seqSaveBtn.h, RED);
  M5.Lcd.drawRect(seqSaveBtn.x, seqSaveBtn.y,
                  seqSaveBtn.w, seqSaveBtn.h, WHITE);
  uiFontMedium();
  M5.Lcd.setTextColor(WHITE);
  uiDrawC("SAVE", seqSaveBtn.x, seqSaveBtn.y, seqSaveBtn.w, seqSaveBtn.h);

  // 6つのステップスロット（プリセットモードと同様のレイアウト）
  for (int i = 0; i < SEQ_STEP_COUNT; i++) {
    bool isCurrent = (i == seqCurrentStep);
    int8_t value = seqPatterns[seqCurrentPattern][i];

    // 上ボタン（▲）
    M5.Lcd.fillRect(seqSteps[i].upBtnX, seqSteps[i].upBtnY,
                    seqSteps[i].upBtnW, seqSteps[i].upBtnH, DARKGREY);
    M5.Lcd.drawRect(seqSteps[i].upBtnX, seqSteps[i].upBtnY,
                    seqSteps[i].upBtnW, seqSteps[i].upBtnH, WHITE);
    int cx = seqSteps[i].upBtnX + seqSteps[i].upBtnW / 2;
    int cy = seqSteps[i].upBtnY + seqSteps[i].upBtnH / 2;
    int ts = 10;
    M5.Lcd.fillTriangle(cx, cy - ts, cx - ts, cy + ts, cx + ts, cy + ts, WHITE);

    // 転調値スロット
    uint16_t slotColor = isCurrent ? GREEN : DARKGREY;
    uint16_t slotTextColor = isCurrent ? BLACK : WHITE;
    M5.Lcd.fillRect(seqSteps[i].slotX, seqSteps[i].slotY,
                    seqSteps[i].slotW, seqSteps[i].slotH, slotColor);
    M5.Lcd.drawRect(seqSteps[i].slotX, seqSteps[i].slotY,
                    seqSteps[i].slotW, seqSteps[i].slotH, WHITE);

    char valueStr[5];
    if (value > 0) sprintf(valueStr, "+%d", value);
    else sprintf(valueStr, "%d", value);

    uiFontMedium();
    M5.Lcd.setTextColor(slotTextColor);
    uiDrawC(valueStr,
            seqSteps[i].slotX, seqSteps[i].slotY,
            seqSteps[i].slotW, seqSteps[i].slotH);

    // 下ボタン（▼）
    M5.Lcd.fillRect(seqSteps[i].downBtnX, seqSteps[i].downBtnY,
                    seqSteps[i].downBtnW, seqSteps[i].downBtnH, DARKGREY);
    M5.Lcd.drawRect(seqSteps[i].downBtnX, seqSteps[i].downBtnY,
                    seqSteps[i].downBtnW, seqSteps[i].downBtnH, WHITE);
    cx = seqSteps[i].downBtnX + seqSteps[i].downBtnW / 2;
    cy = seqSteps[i].downBtnY + seqSteps[i].downBtnH / 2;
    M5.Lcd.fillTriangle(cx, cy + ts, cx - ts, cy - ts, cx + ts, cy - ts, WHITE);
  }

  // ステップ移動ボタン
  M5.Lcd.fillRect(seqStepLeftBtn.x, seqStepLeftBtn.y,
                  seqStepLeftBtn.w, seqStepLeftBtn.h, BLUE);
  M5.Lcd.drawRect(seqStepLeftBtn.x, seqStepLeftBtn.y,
                  seqStepLeftBtn.w, seqStepLeftBtn.h, WHITE);
  uiFontLarge();
  M5.Lcd.setTextColor(WHITE);
  uiDrawC("<", seqStepLeftBtn.x, seqStepLeftBtn.y, seqStepLeftBtn.w, seqStepLeftBtn.h);

  M5.Lcd.fillRect(seqStepRightBtn.x, seqStepRightBtn.y,
                  seqStepRightBtn.w, seqStepRightBtn.h, BLUE);
  M5.Lcd.drawRect(seqStepRightBtn.x, seqStepRightBtn.y,
                  seqStepRightBtn.w, seqStepRightBtn.h, WHITE);
  M5.Lcd.setTextColor(WHITE);
  uiDrawC(">", seqStepRightBtn.x, seqStepRightBtn.y, seqStepRightBtn.w, seqStepRightBtn.h);
}

void drawMidiRuleListBox(int x, int y, int w, int h, bool selected) {
  M5.Lcd.fillRect(x, y, w, h, selected ? DARKGREY : BLACK);
  M5.Lcd.drawRect(x, y, w, h, selected ? GREEN : DARKGREY);
}

void drawMidiActionButton(int x, int y, int w, int h, const char* label, uint16_t fillColor) {
  M5.Lcd.fillRect(x, y, w, h, fillColor);
  M5.Lcd.drawRect(x, y, w, h, WHITE);
  uiFontSmall();
  M5.Lcd.setTextColor(WHITE);
  uiDrawC(label, x, y, w, h);
}

int midiManageVisibleRuleRows() { return 2; }
int midiManageTopButtonY() { return 46; }
int midiManageTopButtonH() { return 22; }
int midiManageRuleRowY(int row) { return 74 + row * 22; }
int midiManageRuleRowH() { return 18; }
int midiManageActionRowY(int row) { return 122 + row * 26; }
int midiManageActionRowH() { return 22; }
int midiManageActionGap() { return 6; }
int midiManageEditFullRowY(int row) { return 178 + row * 26; }
int midiManageEditGridRowY(int row) { return 174 + row * 21; }
int midiManageEditRowH() { return 18; }

void drawMidiInlineEditBox(const char* label, const char* value, int x, int y, int w, int h) {
  const int arrowW = 24;
  const int valueX = x + arrowW + 2;
  const int valueW = w - (arrowW * 2) - 4;

  M5.Lcd.fillRect(x, y, w, h, BLACK);
  M5.Lcd.drawRect(x, y, w, h, DARKGREY);

  M5.Lcd.fillRect(x, y, arrowW, h, NAVY);
  M5.Lcd.drawRect(x, y, arrowW, h, WHITE);
  M5.Lcd.fillRect(x + w - arrowW, y, arrowW, h, NAVY);
  M5.Lcd.drawRect(x + w - arrowW, y, arrowW, h, WHITE);

  uiFontSmall();
  M5.Lcd.setTextColor(WHITE);
  uiDrawC("<", x, y, arrowW, h);
  uiDrawC(">", x + w - arrowW, y, arrowW, h);

  char text[40];
  snprintf(text, sizeof(text), "%s %s", label, value);
  M5.Lcd.setTextColor(CYAN);
  uiDrawC(text, valueX, y, valueW, h);
}

void drawMidiHalfEditPair(const char* leftLabel, const char* leftValue,
                          const char* rightLabel, const char* rightValue,
                          int y) {
  drawMidiInlineEditBox(leftLabel, leftValue, 10, y, 146, midiManageEditRowH());
  drawMidiInlineEditBox(rightLabel, rightValue, 164, y, 146, midiManageEditRowH());
}

void drawMidiCycleButton(const char* label, const char* value, int x, int y, int w, int h) {
  M5.Lcd.fillRect(x, y, w, h, NAVY);
  M5.Lcd.drawRect(x, y, w, h, WHITE);

  char text[40];
  snprintf(text, sizeof(text), "%s %s", label, value);

  uiFontSmall();
  M5.Lcd.setTextColor(WHITE);
  uiDrawC(text, x, y, w, h);
}

void getMidiActionButtonRect(bool mapperPage, int index, int& x, int& y, int& w, int& h) {
  const int gap = midiManageActionGap();
  const int rowH = midiManageActionRowH();
  h = rowH;

  if (!mapperPage) {
    if (index <= 2) {
      y = midiManageActionRowY(0);
      w = 96;
      x = 10 + index * (w + gap);
    } else {
      y = midiManageActionRowY(1);
      w = 147;
      x = (index == 3) ? 10 : 163;
    }
    return;
  }

  y = midiManageActionRowY(index / 3);
  w = 96;
  x = 10 + (index % 3) * (w + gap);
}

int getMidiInlineEditDelta(TouchPoint_t pos, int x, int y, int w, int h) {
  const int arrowW = 24;
  if (touchInRect(pos, x, y, arrowW, h)) return -1;
  if (touchInRect(pos, x + w - arrowW, y, arrowW, h)) return 1;
  return 0;
}

void formatMidiFilterRuleSummary(const MidiFilterRule& rule, int index, char* out, size_t outSize) {
  snprintf(out, outSize, "%d %c %s %s",
           index + 1,
           rule.enabled ? '*' : '-',
           getChannelLabel(rule.channel, false),
           getMidiKindLabel(rule.kind));
}

void formatMidiMapperRuleSummary(const MidiMapperRule& rule, int index, char* out, size_t outSize) {
  snprintf(out, outSize, "%d %c %s %s %s %d>%s %s %s %d",
           index + 1,
           rule.enabled ? '*' : '-',
           getMidiKindLabel(rule.srcKind),
           getChannelLabel(rule.srcChannel, false),
           getData1Label(rule.srcData1, false),
           rule.srcMin,
           getMidiKindLabel(rule.dstKind),
           getChannelLabel(rule.dstChannel, true),
           getData1Label(rule.dstData1, true),
           rule.dstMin);
}

void drawMidiManageMode() {
  const int topY = midiManageTopButtonY();
  const int topH = midiManageTopButtonH();

  drawMidiActionButton(10, topY, 92, topH, "FILTER", midiManagePage == MIDI_PAGE_FILTER ? GREEN : NAVY);
  drawMidiActionButton(108, topY, 92, topH, "MAPPER", midiManagePage == MIDI_PAGE_MAPPER ? GREEN : NAVY);

  bool bypass = (midiManagePage == MIDI_PAGE_FILTER) ? midiFilterBypass : midiMapperBypass;
  drawMidiActionButton(206, topY, 104, topH, bypass ? "BYPASS" : "ACTIVE", bypass ? ORANGE : BLUE);

  int selectedIndex = (midiManagePage == MIDI_PAGE_FILTER) ? midiSelectedFilterRule : midiSelectedMapperRule;
  int ruleCount = (midiManagePage == MIDI_PAGE_FILTER) ? midiFilterRuleCount : midiMapperRuleCount;
  int visibleStart = selectedIndex - 1;
  if (visibleStart < 0) visibleStart = 0;
  if (visibleStart > ruleCount - midiManageVisibleRuleRows()) visibleStart = ruleCount - midiManageVisibleRuleRows();
  if (visibleStart < 0) visibleStart = 0;

  uiFontSmall();
  for (int i = 0; i < midiManageVisibleRuleRows(); i++) {
    int ruleIndex = visibleStart + i;
    if (ruleIndex >= ruleCount) break;

    int rowY = midiManageRuleRowY(i);
    drawMidiRuleListBox(10, rowY, 300, midiManageRuleRowH(), ruleIndex == selectedIndex);

    char line[80];
    if (midiManagePage == MIDI_PAGE_FILTER) {
      formatMidiFilterRuleSummary(midiFilterRules[ruleIndex], ruleIndex, line, sizeof(line));
    } else {
      formatMidiMapperRuleSummary(midiMapperRules[ruleIndex], ruleIndex, line, sizeof(line));
    }
    M5.Lcd.setTextColor(ruleIndex == selectedIndex ? GREEN : WHITE);
    uiDrawL(line, 14, rowY + 3);
  }

  if (midiManagePage == MIDI_PAGE_FILTER) {
    int x, y, w, h;
    for (int i = 0; i < 5; i++) {
      getMidiActionButtonRect(false, i, x, y, w, h);
      const char* label = "";
      uint16_t color = NAVY;
      if (i == 0) { label = midiFilterRules[midiSelectedFilterRule].enabled ? "EN" : "DIS"; color = BLUE; }
      if (i == 1) { label = "ADD"; color = BLUE; }
      if (i == 2) { label = "DEL"; color = RED; }
      if (i == 3) { label = "UP"; color = NAVY; }
      if (i == 4) { label = "DOWN"; color = NAVY; }
      drawMidiActionButton(x, y, w, h, label, color);
    }

    drawMidiCycleButton("Type", getMidiKindLabel(midiFilterRules[midiSelectedFilterRule].kind),
                        10, midiManageEditFullRowY(0), 300, 20);
    drawMidiInlineEditBox("Ch", getChannelLabel(midiFilterRules[midiSelectedFilterRule].channel, false),
                          10, midiManageEditFullRowY(1), 300, 20);
  } else {
    char pageLabel[8];
    snprintf(pageLabel, sizeof(pageLabel), "PG%d", midiMapperEditPage == MAPPER_PAGE_SOURCE ? 1 : 2);
    int x, y, w, h;
    for (int i = 0; i < 6; i++) {
      getMidiActionButtonRect(true, i, x, y, w, h);
      const char* label = "";
      uint16_t color = NAVY;
      if (i == 0) { label = midiMapperRules[midiSelectedMapperRule].enabled ? "EN" : "DIS"; color = BLUE; }
      if (i == 1) { label = "ADD"; color = BLUE; }
      if (i == 2) { label = "DEL"; color = RED; }
      if (i == 3) { label = "UP"; color = NAVY; }
      if (i == 4) { label = "DOWN"; color = NAVY; }
      if (i == 5) { label = pageLabel; color = NAVY; }
      drawMidiActionButton(x, y, w, h, label, color);
    }

    MidiMapperRule& rule = midiMapperRules[midiSelectedMapperRule];
    if (midiMapperEditPage == MAPPER_PAGE_SOURCE) {
      char minStr[12], maxStr[12];
      drawMidiHalfEditPair("Type", getMidiKindLabel(rule.srcKind),
                           "Ch", getChannelLabel(rule.srcChannel, false),
                           midiManageEditGridRowY(0));
      drawMidiInlineEditBox("Data1", getData1Label(rule.srcData1, false),
                            10, midiManageEditGridRowY(1), 300, midiManageEditRowH());
      snprintf(minStr, sizeof(minStr), "%d", rule.srcMin);
      snprintf(maxStr, sizeof(maxStr), "%d", rule.srcMax);
      drawMidiHalfEditPair("Min", minStr, "Max", maxStr, midiManageEditGridRowY(2));
    } else {
      char minStr[12], maxStr[12];
      drawMidiHalfEditPair("Type", getMidiKindLabel(rule.dstKind),
                           "Ch", getChannelLabel(rule.dstChannel, true),
                           midiManageEditGridRowY(0));
      drawMidiInlineEditBox("Data1", getData1Label(rule.dstData1, true),
                            10, midiManageEditGridRowY(1), 300, midiManageEditRowH());
      snprintf(minStr, sizeof(minStr), "%d", rule.dstMin);
      snprintf(maxStr, sizeof(maxStr), "%d", rule.dstMax);
      drawMidiHalfEditPair("Min", minStr, "Max", maxStr, midiManageEditGridRowY(2));
    }
  }
}

void updateStatusArea() {
  // 右上の動的ステータスエリア (Row1 右側のみ)
  // Row2 は AllOff/BT/Hint が居るので触らない
  M5.Lcd.fillRect(155, 0, SCREEN_WIDTH - 155, 18, BLACK);

  uiFontSmall();

  char buf[32];
  if (transposeValue > 0) {
    snprintf(buf, sizeof(buf), "Trans:+%d", transposeValue);
  } else if (transposeValue < 0) {
    snprintf(buf, sizeof(buf), "Trans:%d", transposeValue);
  } else {
    snprintf(buf, sizeof(buf), "Trans: 0");
  }
  M5.Lcd.setTextColor(YELLOW);
  uiDrawL(buf, 240, 2);

  // I/O カウンタは Row1 中央寄り
  M5.Lcd.setTextColor(DARKGREY);
  snprintf(buf, sizeof(buf), "I:%lu O:%lu", midiInCount, midiOutCount);
  uiDrawL(buf, 165, 2);
}

// フットスイッチによる転調値選択処理
void processFootPedal() {
  bool needsProcessing = false;
  bool leftPressed = false;
  bool rightPressed = false;

  // 安全にペダル状態を読み取り
  portENTER_CRITICAL(&g_keyStateMux);
  if (g_pedalNeedsProcessing) {
    g_pedalNeedsProcessing = false;
    leftPressed = g_leftPedalPressed;
    rightPressed = g_rightPedalPressed;
    needsProcessing = true;
  }
  portEXIT_CRITICAL(&g_keyStateMux);

  if (!needsProcessing) return;

  // ペダルが押された瞬間の処理（押しっぱなし防止）
  static bool lastLeftPressed = false;
  static bool lastRightPressed = false;

  bool leftJustPressed = leftPressed && !lastLeftPressed;
  bool rightJustPressed = rightPressed && !lastRightPressed;

  lastLeftPressed = leftPressed;
  lastRightPressed = rightPressed;

  if (leftJustPressed || rightJustPressed) {
    Serial.printf("Foot pedal: Left=%s Right=%s\n",
                  leftJustPressed ? "PRESSED" : "released",
                  rightJustPressed ? "PRESSED" : "released");

    if (currentMode == SEQUENCE_MODE) {
      // SEQUENCE_MODE: ペダルでステップを移動
      if (leftJustPressed) {
        seqCurrentStep = (seqCurrentStep > 0) ? seqCurrentStep - 1 : SEQ_STEP_COUNT - 1;
        handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
        needFullRedraw = true;
        Serial.printf("Sequence Mode (Pedal): Step %d, Pat %d\n", seqCurrentStep, seqCurrentPattern);
      } else if (rightJustPressed) {
        seqCurrentStep = (seqCurrentStep < SEQ_STEP_COUNT - 1) ? seqCurrentStep + 1 : 0;
        handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
        needFullRedraw = true;
        Serial.printf("Sequence Mode (Pedal): Step %d, Pat %d\n", seqCurrentStep, seqCurrentPattern);
      }
    } else {
      // 他のモード: 従来の転調値選択処理
      // 各モードとレンジに応じた転調値配列を取得
      int8_t* transposeValues = nullptr;
      int numValues = 0;
      int8_t values[12];

      if (currentMode == DIRECT_MODE) {
        // DirectModeの転調値配列を作成
        for (int i = 0; i < 12; i++) {
          values[i] = directButtons[i].value;
        }
        transposeValues = values;
        numValues = 12;
      } else if (currentMode == KEY_MODE) {
        // KeyModeでは現在の設定に応じた転調値範囲を使用
        if (majorUpperTranspose || minorUpperTranspose) {
          // 上位/下位転調の場合は広い範囲
          for (int i = 0; i < 12; i++) {
            values[i] = i - 11; // -11 to 0 or similar range
          }
        } else {
          // 通常転調の場合
          for (int i = 0; i < 12; i++) {
            values[i] = i - 5; // -5 to +6
          }
        }
        transposeValues = values;
        numValues = 12;
      } else { // INSTANT_MODE
        // 相対モードでは-5から+6の範囲を使用
        for (int i = 0; i < 12; i++) {
          values[i] = i - 5; // -5 to +6
        }
        transposeValues = values;
        numValues = 12;
      }

      // 現在の転調値のインデックスを見つける
      int currentIndex = -1;
      for (int i = 0; i < numValues; i++) {
        if (transposeValues[i] == transposeValue) {
          currentIndex = i;
          break;
        }
      }

      if (currentIndex == -1) {
        // 現在の転調値が配列にない場合は、最も近い値を選択
        currentIndex = 5; // デフォルト（通常は0に近い値）
      }

      // フットスイッチに基づいて転調値をシフト
      int newIndex = currentIndex;
      if (leftJustPressed) {
        // 上ボタン：減る方向（インデックスを減らす）
        newIndex = (currentIndex > 0) ? currentIndex - 1 : numValues - 1;
      } else if (rightJustPressed) {
        // 下ボタン：増える方向（インデックスを増やす）
        newIndex = (currentIndex < numValues - 1) ? currentIndex + 1 : 0;
      }

      int8_t newTransposeValue = transposeValues[newIndex];

      Serial.printf("Transpose change: %d -> %d (index %d -> %d)\n",
                    transposeValue, newTransposeValue, currentIndex, newIndex);

      // 転調値を更新
      handleTransposeChange(newTransposeValue);

      // 対応するUIの更新
      if (currentMode == DIRECT_MODE) {
        // DirectModeの場合、対応するボタンを光らせる
        for (int j = 0; j < 12; j++) transposeButtons[j] = false;
        transposeButtons[newIndex] = true;
        needFullRedraw = true;
      } else if (currentMode == KEY_MODE) {
        // KeyModeの場合、選択状態をリセット（新しい転調値に基づいて自動選択される）
        selectedMajorKey = -1;
        selectedMinorKey = -1;
        needFullRedraw = true;
      } else {
        // InstantModeの場合、画面を更新
        needFullRedraw = true;
      }
    }
  }
}

void enterDisplayMode(int newMode) {
  currentMode = (DisplayMode)newMode;

  if (currentMode == DIRECT_MODE) {
    setCurrentTransposeButton();
  } else if (currentMode == KEY_MODE) {
    selectedMajorKey = -1;
    selectedMinorKey = -1;
  } else if (currentMode == SEQUENCE_MODE) {
    handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
  }

  if (isTransposeDisplayMode(currentMode)) {
    lastTransposeMode = currentMode;
  }

  needFullRedraw = true;
}

void advanceDisplayMode() {
  sendAllNotesOff();

  if (currentMode == MIDI_MANAGE_MODE) {
    DisplayMode restoreMode = isTransposeDisplayMode(lastTransposeMode) ? lastTransposeMode : DIRECT_MODE;
    enterDisplayMode(restoreMode);
    Serial.printf("Group: TRANSPOSE (%d)\n", restoreMode);
  } else {
    if (isTransposeDisplayMode(currentMode)) {
      lastTransposeMode = currentMode;
    }
    enterDisplayMode(MIDI_MANAGE_MODE);
    Serial.println("Group: MIDI_MANAGE");
  }
}

void advanceSubMode() {
  if (currentMode == MIDI_MANAGE_MODE) {
    midiManagePage = (midiManagePage == MIDI_PAGE_FILTER) ? MIDI_PAGE_MAPPER : MIDI_PAGE_FILTER;
    needFullRedraw = true;
    Serial.printf("MIDI page: %s\n", midiManagePage == MIDI_PAGE_FILTER ? "FILTER" : "MAPPER");
    return;
  }

  sendAllNotesOff();

  if (currentMode == DIRECT_MODE) enterDisplayMode(KEY_MODE);
  else if (currentMode == KEY_MODE) enterDisplayMode(INSTANT_MODE);
  else if (currentMode == INSTANT_MODE) enterDisplayMode(SEQUENCE_MODE);
  else enterDisplayMode(DIRECT_MODE);

  // Use the lookup helper so we don't OOB-read past the array when
  // currentMode is SEQUENCE_MODE (=4) or MIDI_MANAGE_MODE (=5). The bare
  // `modeNames[currentMode]` indexed past a 4-entry array on every entry
  // into SEQUENCE mode and depended on whatever memory followed being a
  // valid C string — a latent crash that returns intermittently.
  Serial.printf("Transpose mode: %s\n", getDisplayModeLabel(currentMode));
}

void processHardwareButtons() {
  unsigned long now = millis();
  static bool btnALongPressHandled = false;
  static bool btnBLongPressHandled = false;

  // ── 長押し検出 (押下中に MODE_LONG_PRESS_MS 経過したら一度だけ発火) ──
  // 重要: ここではフラグをリセットしない。リリース時の短押し判定が
  // フラグを参照してから、その後でリセットする (ブロックの末尾で)。
  if (M5.BtnC.isPressed() && !btnCLongPressHandled
      && M5.BtnC.pressedFor(MODE_LONG_PRESS_MS)) {
    handleButtonCLongAction();
    btnCLongPressHandled = true;
    lastButtonCheck = now;
  }
  if (M5.BtnB.isPressed() && !btnBLongPressHandled
      && M5.BtnB.pressedFor(MODE_LONG_PRESS_MS)) {
    enterConfigEditMode();
    btnBLongPressHandled = true;
    lastButtonCheck = now;
  }
  if (M5.BtnA.isPressed() && !btnALongPressHandled
      && M5.BtnA.pressedFor(MODE_LONG_PRESS_MS)) {
    if (currentMode == BASE_SET_MODE) exitBaseSetMode();
    else                              enterBaseSetMode();
    btnALongPressHandled = true;
    lastButtonCheck = now;
  }

  if (now - lastButtonCheck < BUTTON_DEBOUNCE) return;

  // ── 短押しはリリース時判定。長押しフラグが立っていればスキップ ──
  // 各ボタンとも「リリースを観測 → 短押し処理 (長押し未発火時のみ) →
  // 長押しフラグをリセット」の順。これにより長押し中の release 判定で
  // 短押しが誤発火しない。

  if (M5.BtnA.wasReleased()) {
    if (!btnALongPressHandled) handleButtonAAction();
    btnALongPressHandled = false;
    lastButtonCheck = now;
    return;
  }
  if (M5.BtnB.wasReleased()) {
    if (!btnBLongPressHandled) handleButtonBAction();
    btnBLongPressHandled = false;
    lastButtonCheck = now;
    return;
  }
  if (M5.BtnC.wasReleased()) {
    if (!btnCLongPressHandled) {
      if (currentMode == BASE_SET_MODE) cycleBaseSetPage();
      else                              handleButtonCShortAction();
    }
    btnCLongPressHandled = false;
    lastButtonCheck = now;
  }

  if (false && M5.BtnC.wasPressed()) {
    sendAllNotesOff();
    delay(10);

    if (currentMode == DIRECT_MODE) currentMode = KEY_MODE;
    else if (currentMode == KEY_MODE) currentMode = INSTANT_MODE;
    else if (currentMode == INSTANT_MODE) currentMode = SEQUENCE_MODE;
    else currentMode = DIRECT_MODE;

    if (currentMode == DIRECT_MODE) {
      setCurrentTransposeButton();  // 現在の転調値に対応するボタンを光らせる
      Serial.println("DirectMode: Current transpose button selected");
    } else if (currentMode == KEY_MODE) {
      selectedMajorKey = -1;
      selectedMinorKey = -1;
      Serial.println("KeyMode: All keys deselected");
    } else if (currentMode == INSTANT_MODE) {
      // 相対モード：特に選択状態は持たない
      Serial.println("InstantMode entered");
    } else {
      // シーケンスモード：現在ステップの転調値を適用
      handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
      Serial.println("SequenceMode entered");
    }

    needFullRedraw = true;
    lastButtonCheck = now;
    // Same OOB hazard as advanceSubMode — see comment there. Use the
    // safe lookup so SEQUENCE_MODE (=4) doesn't read past the array.
    Serial.printf("Mode: %s, Transpose: %d (maintained)\n",
                  getDisplayModeLabel(currentMode), transposeValue);
  }
}

void recalculateKeyModeTranspose() {
  if (selectedMajorKey != -1) {
    if (majorUpperTranspose) {
      transposeValue = 12 - selectedMajorKey; // 上位転調
    } else {
      transposeValue = -selectedMajorKey; // 通常転調
    }
  } else if (selectedMinorKey != -1) {
    if (minorUpperTranspose) {
      transposeValue = -(9 - selectedMinorKey); // 下位転調（負の値）
    } else {
      transposeValue = 9 - selectedMinorKey; // 通常転調
    }
  }
}

// ピアノキーのインデックスを取得（A0=21からC8=108の88鍵盤）
int getPianoKeyIndex(uint8_t midiNote) {
  if (midiNote < PIANO_LOWEST_NOTE || midiNote > PIANO_HIGHEST_NOTE) {
    return -1;
  }
  return midiNote - PIANO_LOWEST_NOTE;
}

int getTrackedNoteStateIndex(uint8_t midiNote, uint8_t channel) {
  int pianoKeyIndex = getPianoKeyIndex(midiNote);
  if (pianoKeyIndex < 0 || channel >= MIDI_CHANNEL_COUNT) {
    return -1;
  }
  return pianoKeyIndex * MIDI_CHANNEL_COUNT + channel;
}

void clearTrackedNoteStates(void) {
  for (int i = 0; i < TRACKED_NOTE_STATE_COUNT; i++) {
    currentNoteStates[i].isActive = false;
    savedNoteStates[i].isActive = false;
  }
}

bool isMidiInputIdle(unsigned long now) {
  return Serial2.available() == 0 && (now - g_lastMidiInputAt) >= 250;
}

void processDeferredStorageTasks(unsigned long now) {
  if (!isMidiInputIdle(now)) {
    return;
  }

  if (g_btBondSavePending) {
    g_btBondSavePending = false;
    Serial.println("[BT_BOND] Idle window detected - saving bond to SD...");
    if (saveBTBondToSD()) {
      showBTOverlay("Bond Saved!", 0x03E0, 1500);
    } else {
      showBTOverlay("Bond Save Err", RED, 2000);
    }
    return;
  }

  if (g_seqSavePending) {
    g_seqSavePending = false;
    Serial.println("[SEQ] Idle window detected - saving sequence to SD...");
    if (saveSequencesToSD()) {
      showBTOverlay("SEQ Saved!", 0x03E0, 1200);
    } else {
      showBTOverlay("SEQ Save Err", RED, 1600);
    }
  }
}

// 転調値変更時の処理（スムーズな転調）
// MIDI ホットパスを止めないよう、ここでは待ちを入れない。
void handleTransposeChange(int8_t newTransposeValue) {
  newTransposeValue = clampTranspose(newTransposeValue);
  if (newTransposeValue == transposeValue) return; // 変更なし
  
  if (allNotesOffEnabled) {
    sendAllNotesOff();
    transposeValue = newTransposeValue;
    needPartialUpdate = true;
    return;
  }
  
  savedTranspose = transposeValue;
  for (int i = 0; i < TRACKED_NOTE_STATE_COUNT; i++) {
    savedNoteStates[i] = currentNoteStates[i];
  }
  
  transposeValue = newTransposeValue;
  needPartialUpdate = true;
  
  Serial.printf("Smooth transpose: %d -> %d\n", savedTranspose, transposeValue);
}

void processTouch() {
  TouchPoint_t pos = M5.Touch.getPressPoint();

  // ラッチ：押下開始の瞬間だけ処理
  bool nowActive = (pos.x != -1 && pos.y != -1);
  if (!nowActive) {
    touchWasActive = false;
    return;
  }
  if (touchWasActive) {
    // 既に押下中は無視（連続反応防止）
    return;
  }
  touchWasActive = true;

  // ここから「押した瞬間」のみ処理
  dispatchTouchPoint(pos);
}

void processDirectModeTouch(TouchPoint_t pos) {
  for (int i = 0; i < 12; i++) {
    if (pos.x >= directButtons[i].x && pos.x <= directButtons[i].x + directButtons[i].w &&
        pos.y >= directButtons[i].y && pos.y <= directButtons[i].y + directButtons[i].h) {
      
      handleTransposeChange(directButtons[i].value);
      
      for (int j = 0; j < 12; j++) transposeButtons[j] = false;
      transposeButtons[i] = true;
      
      needFullRedraw = true;
      break;
    }
  }
}

void processKeyModeTouch(TouchPoint_t pos) {
  int8_t newTransposeValue = transposeValue;
  
  // メジャー（黒鍵優先）
  for (int i = 0; i < 12; i++) {
    if (majorKeys[i].isBlackKey &&
        pos.x >= majorKeys[i].x && pos.x <= majorKeys[i].x + majorKeys[i].w &&
        pos.y >= majorKeys[i].y && pos.y <= majorKeys[i].y + majorKeys[i].h) {
      
      selectedMajorKey = i;
      selectedMinorKey = -1;
      newTransposeValue = majorUpperTranspose ? (12 - i) : (-i);
      handleTransposeChange(newTransposeValue);
      needFullRedraw = true;
      return;
    }
  }
  for (int i = 0; i < 12; i++) {
    if (!majorKeys[i].isBlackKey &&
        pos.x >= majorKeys[i].x && pos.x <= majorKeys[i].x + majorKeys[i].w &&
        pos.y >= majorKeys[i].y && pos.y <= majorKeys[i].y + majorKeys[i].h) {
      
      selectedMajorKey = i;
      selectedMinorKey = -1;
      newTransposeValue = majorUpperTranspose ? (12 - i) : (-i);
      handleTransposeChange(newTransposeValue);
      needFullRedraw = true;
      return;
    }
  }
  
  // マイナー（黒鍵優先）
  for (int i = 0; i < 12; i++) {
    if (minorKeys[i].isBlackKey &&
        pos.x >= minorKeys[i].x && pos.x <= minorKeys[i].x + minorKeys[i].w &&
        pos.y >= minorKeys[i].y && pos.y <= minorKeys[i].y + minorKeys[i].h) {
      
      selectedMinorKey = i;
      selectedMajorKey = -1;
      newTransposeValue = minorUpperTranspose ? (-(9 - i)) : (9 - i);
      handleTransposeChange(newTransposeValue);
      needFullRedraw = true;
      return;
    }
  }
  for (int i = 0; i < 12; i++) {
    if (!minorKeys[i].isBlackKey &&
        pos.x >= minorKeys[i].x && pos.x <= minorKeys[i].x + minorKeys[i].w &&
        pos.y >= minorKeys[i].y && pos.y <= minorKeys[i].y + minorKeys[i].h) {
      
      selectedMinorKey = i;
      selectedMajorKey = -1;
      newTransposeValue = minorUpperTranspose ? (-(9 - i)) : (9 - i);
      handleTransposeChange(newTransposeValue);
      needFullRedraw = true;
      return;
    }
  }
}

// 相対モードのタッチ処理（絶対指定：押した値に転調）
void processInstantModeTouch(TouchPoint_t pos) {
  // 0 ボタン
  if (pos.x >= instantZeroBtn.x && pos.x <= instantZeroBtn.x + instantZeroBtn.w &&
      pos.y >= instantZeroBtn.y && pos.y <= instantZeroBtn.y + instantZeroBtn.h) {
    handleTransposeChange(0);
    needFullRedraw = true;
    return;
  }

  // ±1,2,3,5 ボタン
  for (int i = 0; i < 8; i++) {
    if (pos.x >= instantButtons[i].x && pos.x <= instantButtons[i].x + instantButtons[i].w &&
        pos.y >= instantButtons[i].y && pos.y <= instantButtons[i].y + instantButtons[i].h) {
      handleTransposeChange(instantButtons[i].value);
      needFullRedraw = true;
      return;
    }
  }
}

// ============================================================
// CONFIG_EDIT_MODE — entered via B long-press, edits /config.json
// ============================================================
// Single-page list view: 12 rows, tap row to cycle field's value.
// Bottom bar: SAVE / CANCEL / APPLY.
//   SAVE   = write to SD, exit to mode that was active before entering editor
//   CANCEL = restore in-memory copy from snapshot, exit
//   APPLY  = SAVE + applyDeviceConfig() (jump to DefaultApp's mode now)
//
// Operates on g_config in place. snapshot is taken on enter so CANCEL
// can revert. SAVE / APPLY commit to SD.
static DeviceConfig g_configSnapshot;
static DisplayMode g_modeBeforeOverlay = DIRECT_MODE;

static const int CFG_ROWS = 12;
static const int CFG_ROWS_PER_PAGE = 4;
static const int CFG_PAGES = 3;        // 4 × 3 = 12
static int g_configPage = 0;
static const char* CFG_LABELS[CFG_ROWS] = {
  // Page 1: Boot defaults
  "DefaultApp",
  "DefTransMode",
  "InitTranspose",
  "TransposeBase",
  // Page 2: Initial runtime state
  "AllNotesOff",
  "FilterBypass",
  "MapperBypass",
  "TransposeRange",
  // Page 3: Misc
  "MidiInputSrc",
  "MajorUpperTr",
  "BTAutoReconn",
  "ShowSplash",
};

static void cycleString(char* dst, size_t cap, const char* const* options, int n) {
  int idx = 0;
  for (int i = 0; i < n; i++) if (strcmp(dst, options[i]) == 0) { idx = i; break; }
  idx = (idx + 1) % n;
  strncpy(dst, options[idx], cap);
  dst[cap - 1] = '\0';
}

static void cfgFormatValue(int row, char* out, size_t outSize) {
  switch (row) {
    case 0: snprintf(out, outSize, "%s", g_config.defaultApp); break;
    case 1: snprintf(out, outSize, "%s", g_config.defaultTransposeMode); break;
    case 2: snprintf(out, outSize, "%+d", g_config.initialTranspose); break;
    case 3: snprintf(out, outSize, "%+d", g_config.transposeBase); break;
    case 4: snprintf(out, outSize, "%s", g_config.initialAllNotesOff ? "ON" : "OFF"); break;
    case 5: snprintf(out, outSize, "%s", g_config.initialFilterBypass ? "ON" : "OFF"); break;
    case 6: snprintf(out, outSize, "%s", g_config.initialMapperBypass ? "ON" : "OFF"); break;
    case 7: snprintf(out, outSize, "%s", g_config.transposeRange); break;
    case 8: snprintf(out, outSize, "%s", g_config.midiInputSource); break;
    case 9: snprintf(out, outSize, "%s", g_config.majorUpperTranspose ? "ON" : "OFF"); break;
    case 10: snprintf(out, outSize, "%s", g_config.btAutoReconnect ? "ON" : "OFF"); break;
    case 11: snprintf(out, outSize, "%s", g_config.showSplash ? "ON" : "OFF"); break;
    default: out[0] = '\0';
  }
}

static void cfgCycleValue(int row) {
  static const char* APPS[]   = {"Transpose","Play","SMF","MP3","Filter","Change"};
  static const char* TMODES[] = {"DIRECT","KEY","INSTANT","SEQUENCE"};
  static const char* RANGES[] = {"-5..6","0..11","-11..0"};
  static const char* INPUTS[] = {"MIX","MIDIIN","USB"};
  switch (row) {
    case 0: cycleString(g_config.defaultApp, sizeof(g_config.defaultApp), APPS, 6); break;
    case 1: cycleString(g_config.defaultTransposeMode, sizeof(g_config.defaultTransposeMode), TMODES, 4); break;
    case 2: g_config.initialTranspose = (g_config.initialTranspose >= 11) ? -11 : g_config.initialTranspose + 1; break;
    case 3: g_config.transposeBase    = (g_config.transposeBase    >= 11) ? -11 : g_config.transposeBase + 1; break;
    case 4: g_config.initialAllNotesOff = !g_config.initialAllNotesOff; break;
    case 5: g_config.initialFilterBypass = !g_config.initialFilterBypass; break;
    case 6: g_config.initialMapperBypass = !g_config.initialMapperBypass; break;
    case 7: cycleString(g_config.transposeRange, sizeof(g_config.transposeRange), RANGES, 3); break;
    case 8: cycleString(g_config.midiInputSource, sizeof(g_config.midiInputSource), INPUTS, 3); break;
    case 9: g_config.majorUpperTranspose = !g_config.majorUpperTranspose; break;
    case 10: g_config.btAutoReconnect = !g_config.btAutoReconnect; break;
    case 11: g_config.showSplash = !g_config.showSplash; break;
  }
}

void enterConfigEditMode() {
  g_modeBeforeOverlay = currentMode;
  g_configSnapshot = g_config;
  g_configPage = 0;
  enterDisplayMode(CONFIG_EDIT_MODE);
}

void exitConfigEditMode(bool save, bool apply) {
  if (!save && !apply) {
    // CANCEL — revert
    g_config = g_configSnapshot;
  } else {
    saveDeviceConfigToSD();
    if (apply) {
      applyDeviceConfig();
      needFullRedraw = true;
      return;  // applyDeviceConfig sets a fresh mode
    }
  }
  enterDisplayMode(g_modeBeforeOverlay);
}

// Layout: header y=0..40 (existing). Below the header divider:
//   Page nav strip   y=44..72  (28px) — "Page n/3" + [<] [>]
//   Field rows       y=76..196 (4 rows × 30px) — tap row to cycle value
//   Footer buttons   y=204..234 (30px) — SAVE / CANCEL / APPLY
// 12 fields split into 3 pages of 4 each so rows stay tappable on a 320×240
// screen without overlapping the AllOff/BT/I-O header strip.
void drawConfigEditMode() {
  M5.Lcd.fillRect(0, 41, SCREEN_WIDTH, SCREEN_HEIGHT - 41, BLACK);

  // Page navigation strip
  const int navY = 44, navH = 28;
  M5.Lcd.fillRect(0, navY, SCREEN_WIDTH, navH, NAVY);
  M5.Lcd.drawRect(50, navY, 220, navH, WHITE);  // central title box
  uiFontMedium();
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.setTextDatum(MC_DATUM);
  char pgStr[24];
  snprintf(pgStr, sizeof(pgStr), "Config %d/%d (B-long)", g_configPage + 1, CFG_PAGES);
  M5.Lcd.drawString(pgStr, SCREEN_WIDTH / 2, navY + navH / 2);
  // < page-prev (left)
  M5.Lcd.fillRect(5, navY, 40, navH, BLUE);
  M5.Lcd.drawRect(5, navY, 40, navH, WHITE);
  M5.Lcd.setTextColor(WHITE);
  uiDrawC("<", 5, navY, 40, navH);
  // > page-next (right)
  M5.Lcd.fillRect(SCREEN_WIDTH - 45, navY, 40, navH, BLUE);
  M5.Lcd.drawRect(SCREEN_WIDTH - 45, navY, 40, navH, WHITE);
  uiDrawC(">", SCREEN_WIDTH - 45, navY, 40, navH);

  // Field rows for current page
  uiFontSmall();
  M5.Lcd.setTextDatum(ML_DATUM);
  const int rowY0 = 76;
  const int rowH  = 30;
  for (int r = 0; r < CFG_ROWS_PER_PAGE; r++) {
    int field = g_configPage * CFG_ROWS_PER_PAGE + r;
    if (field >= CFG_ROWS) break;
    int y = rowY0 + r * rowH;
    M5.Lcd.fillRect(0, y, SCREEN_WIDTH, rowH - 2, 0x18C3);
    M5.Lcd.drawRect(0, y, SCREEN_WIDTH, rowH - 2, 0x4208);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.drawString(CFG_LABELS[field], 8, y + (rowH - 2) / 2);
    char val[24];
    cfgFormatValue(field, val, sizeof(val));
    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.setTextDatum(MR_DATUM);
    M5.Lcd.drawString(val, SCREEN_WIDTH - 8, y + (rowH - 2) / 2);
    M5.Lcd.setTextDatum(ML_DATUM);
  }

  // Footer buttons (y=204..234)
  M5.Lcd.setTextDatum(MC_DATUM);
  const int btnY = 204, btnH = 30;
  M5.Lcd.fillRect(8, btnY, 96, btnH, 0x03E0);
  M5.Lcd.drawRect(8, btnY, 96, btnH, WHITE);
  M5.Lcd.setTextColor(WHITE); uiFontMedium();
  uiDrawC("SAVE", 8, btnY, 96, btnH);
  M5.Lcd.fillRect(112, btnY, 96, btnH, RED);
  M5.Lcd.drawRect(112, btnY, 96, btnH, WHITE);
  uiDrawC("CANCEL", 112, btnY, 96, btnH);
  M5.Lcd.fillRect(216, btnY, 96, btnH, BLUE);
  M5.Lcd.drawRect(216, btnY, 96, btnH, WHITE);
  uiDrawC("APPLY", 216, btnY, 96, btnH);
}

void processConfigEditTouch(TouchPoint_t pos) {
  // Page-nav strip
  const int navY = 44, navH = 28;
  if (pos.y >= navY && pos.y <= navY + navH) {
    if (pos.x >= 5 && pos.x <= 45) {
      g_configPage = (g_configPage + CFG_PAGES - 1) % CFG_PAGES;
      needFullRedraw = true;
      return;
    }
    if (pos.x >= SCREEN_WIDTH - 45 && pos.x <= SCREEN_WIDTH - 5) {
      g_configPage = (g_configPage + 1) % CFG_PAGES;
      needFullRedraw = true;
      return;
    }
  }
  // Footer buttons
  if (pos.y >= 204 && pos.y <= 234) {
    if (pos.x >= 8   && pos.x <= 104) { exitConfigEditMode(true,  false); return; }
    if (pos.x >= 112 && pos.x <= 208) { exitConfigEditMode(false, false); return; }
    if (pos.x >= 216 && pos.x <= 312) { exitConfigEditMode(true,  true);  return; }
  }
  // Field rows for current page
  const int rowY0 = 76, rowH = 30;
  for (int r = 0; r < CFG_ROWS_PER_PAGE; r++) {
    int field = g_configPage * CFG_ROWS_PER_PAGE + r;
    if (field >= CFG_ROWS) break;
    int y = rowY0 + r * rowH;
    if (pos.y >= y && pos.y < y + rowH - 2) {
      cfgCycleValue(field);
      needFullRedraw = true;
      return;
    }
  }
}

// ============================================================
// BASE_SET_MODE — A long-press: pick the transpose base reference
// ============================================================
// All transpose values selected in DIRECT/KEY/INSTANT/SEQUENCE become
// (base + offset). Changing the base re-applies transpose immediately
// (we shift the current effective transpose by the base delta — keeps
// the user's last selection in the new base frame).
//
// The base picker is laid out like the DIRECT 4×3 grid but spans three
// pages so the full -12..+12 range is reachable:
//   page L: -12..-1   (entered by tapping -5 on page M, or via C button)
//   page M: -5..+6    (default entry; mirrors DIRECT mode's range)
//   page R: +1..+12   (entered by tapping +6 on page M)
// Tapping -1 on page L or +1 on page R returns to page M (per spec).
// C button cycles L → M → R → L. A long-press exits the mode entirely.
static int g_basePage = 0;            // -1=L, 0=M, +1=R
static DisplayMode g_modeBeforeBase = DIRECT_MODE;

static void getBasePageRange(int page, int* lo, int* hi) {
  if (page == -1)     { *lo = -12; *hi = -1; }
  else if (page == 1) { *lo =   1; *hi = 12; }
  else                { *lo =  -5; *hi =  6; }
}

static int basePageValueAt(int page, int idx) {
  int lo, hi; getBasePageRange(page, &lo, &hi);
  return lo + idx;
}

static int pageContainingBase(int base) {
  if (base < -5)     return -1;
  else if (base > 6) return 1;
  else                return 0;
}

void setTransposeBase(int newBase) {
  newBase = clampTranspose((int8_t)newBase);
  int delta = newBase - g_config.transposeBase;
  g_config.transposeBase = newBase;
  if (delta != 0) {
    // Apply immediately — shift the current effective transpose by delta so
    // any user-selected offset in DIRECT/KEY/INSTANT/SEQUENCE follows the
    // new base.
    handleTransposeChange(clampTranspose((int8_t)(transposeValue + delta)));
  }
  needFullRedraw = true;
}

void enterBaseSetMode() {
  g_modeBeforeBase = currentMode;
  g_basePage = pageContainingBase(g_config.transposeBase);
  enterDisplayMode(BASE_SET_MODE);
}

void exitBaseSetMode() {
  enterDisplayMode(g_modeBeforeBase);
}

void cycleBaseSetPage() {
  // L (-1) → M (0) → R (+1) → L
  if (g_basePage == -1)      g_basePage = 0;
  else if (g_basePage == 0)  g_basePage = 1;
  else                       g_basePage = -1;
  needFullRedraw = true;
}

void drawBaseSetMode() {
  M5.Lcd.fillRect(0, 41, SCREEN_WIDTH, SCREEN_HEIGHT - 41, BLACK);

  // Title strip
  M5.Lcd.fillRect(0, 41, SCREEN_WIDTH, 22, NAVY);
  M5.Lcd.setTextDatum(MC_DATUM);
  uiFontSmall();
  M5.Lcd.setTextColor(YELLOW);
  char title[40];
  const char* pgLabel = (g_basePage == -1) ? "L (-12..-1)"
                       : (g_basePage == 1) ? "R (+1..+12)"
                                            : "M (-5..+6)";
  snprintf(title, sizeof(title), "Base = %+d   Page %s   (A-long: exit  C: page)",
           g_config.transposeBase, pgLabel);
  M5.Lcd.drawString(title, SCREEN_WIDTH / 2, 52);

  // 4×3 grid mirroring DIRECT mode (75×60 buttons, 5×4 spacing, start (10, 66))
  const int btnW = 75, btnH = 56, gapX = 5, gapY = 4;
  const int startX = 10, startY = 68;
  const int colsPerRow = 4;
  uiFontLarge();
  for (int i = 0; i < 12; i++) {
    int col = i % colsPerRow;
    int row = i / colsPerRow;
    int x = startX + col * (btnW + gapX);
    int y = startY + row * (btnH + gapY);
    int val = basePageValueAt(g_basePage, i);
    bool selected = (val == g_config.transposeBase);
    uint16_t bg = selected ? GREEN : DARKGREY;
    uint16_t txtCol = selected ? BLACK : WHITE;
    M5.Lcd.fillRect(x, y, btnW, btnH, bg);
    M5.Lcd.drawRect(x, y, btnW, btnH, WHITE);
    M5.Lcd.setTextColor(txtCol);
    char vs[6];
    if (val > 0) snprintf(vs, sizeof(vs), "+%d", val);
    else         snprintf(vs, sizeof(vs), "%d", val);
    uiDrawC(vs, x, y, btnW, btnH);
  }
}

void processBaseSetTouch(TouchPoint_t pos) {
  const int btnW = 75, btnH = 56, gapX = 5, gapY = 4;
  const int startX = 10, startY = 68;
  const int colsPerRow = 4;
  for (int i = 0; i < 12; i++) {
    int col = i % colsPerRow;
    int row = i / colsPerRow;
    int x = startX + col * (btnW + gapX);
    int y = startY + row * (btnH + gapY);
    if (pos.x >= x && pos.x < x + btnW && pos.y >= y && pos.y < y + btnH) {
      int val = basePageValueAt(g_basePage, i);
      setTransposeBase(val);
      // Page transitions per spec.
      if (g_basePage == 0) {
        if (val == 6)       g_basePage = 1;   // M → R
        else if (val == -5) g_basePage = -1;  // M → L
      } else if (g_basePage == 1) {
        if (val == 1) g_basePage = 0;          // R → M
      } else { // L
        if (val == -1) g_basePage = 0;         // L → M
      }
      needFullRedraw = true;
      return;
    }
  }
}

// シーケンスモードのタッチ処理
void processSequenceModeTouch(TouchPoint_t pos) {
  // パターン左ボタン
  if (pos.x >= seqPatLeftBtn.x && pos.x <= seqPatLeftBtn.x + seqPatLeftBtn.w &&
      pos.y >= seqPatLeftBtn.y && pos.y <= seqPatLeftBtn.y + seqPatLeftBtn.h) {
    seqCurrentPattern = (seqCurrentPattern > 0) ? seqCurrentPattern - 1 : SEQ_PATTERN_COUNT - 1;
    seqCurrentStep = 0;
    handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
    needFullRedraw = true;
    Serial.printf("Seq: Pattern %d selected\n", seqCurrentPattern + 1);
    return;
  }

  // パターン右ボタン
  if (pos.x >= seqPatRightBtn.x && pos.x <= seqPatRightBtn.x + seqPatRightBtn.w &&
      pos.y >= seqPatRightBtn.y && pos.y <= seqPatRightBtn.y + seqPatRightBtn.h) {
    seqCurrentPattern = (seqCurrentPattern < SEQ_PATTERN_COUNT - 1) ? seqCurrentPattern + 1 : 0;
    seqCurrentStep = 0;
    handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
    needFullRedraw = true;
    Serial.printf("Seq: Pattern %d selected\n", seqCurrentPattern + 1);
    return;
  }

  // SAVEボタン
  if (pos.x >= seqSaveBtn.x && pos.x <= seqSaveBtn.x + seqSaveBtn.w &&
      pos.y >= seqSaveBtn.y && pos.y <= seqSaveBtn.y + seqSaveBtn.h) {
    g_seqSavePending = true;
    showBTOverlay("SEQ Save Q", BLUE, 800);
    return;
  }

  // ステップ左ボタン
  if (pos.x >= seqStepLeftBtn.x && pos.x <= seqStepLeftBtn.x + seqStepLeftBtn.w &&
      pos.y >= seqStepLeftBtn.y && pos.y <= seqStepLeftBtn.y + seqStepLeftBtn.h) {
    seqCurrentStep = (seqCurrentStep > 0) ? seqCurrentStep - 1 : SEQ_STEP_COUNT - 1;
    handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
    needFullRedraw = true;
    return;
  }

  // ステップ右ボタン
  if (pos.x >= seqStepRightBtn.x && pos.x <= seqStepRightBtn.x + seqStepRightBtn.w &&
      pos.y >= seqStepRightBtn.y && pos.y <= seqStepRightBtn.y + seqStepRightBtn.h) {
    seqCurrentStep = (seqCurrentStep < SEQ_STEP_COUNT - 1) ? seqCurrentStep + 1 : 0;
    handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
    needFullRedraw = true;
    return;
  }

  // ステップの上下ボタンとスロットタッチ
  for (int i = 0; i < SEQ_STEP_COUNT; i++) {
    // 上ボタン（値を+1）
    if (pos.x >= seqSteps[i].upBtnX && pos.x <= seqSteps[i].upBtnX + seqSteps[i].upBtnW &&
        pos.y >= seqSteps[i].upBtnY && pos.y <= seqSteps[i].upBtnY + seqSteps[i].upBtnH) {
      seqPatterns[seqCurrentPattern][i] = clampTranspose(seqPatterns[seqCurrentPattern][i] + 1);
      seqCurrentStep = i;
      handleTransposeChange(seqPatterns[seqCurrentPattern][i]);
      needFullRedraw = true;
      return;
    }

    // 下ボタン（値を-1）
    if (pos.x >= seqSteps[i].downBtnX && pos.x <= seqSteps[i].downBtnX + seqSteps[i].downBtnW &&
        pos.y >= seqSteps[i].downBtnY && pos.y <= seqSteps[i].downBtnY + seqSteps[i].downBtnH) {
      seqPatterns[seqCurrentPattern][i] = clampTranspose(seqPatterns[seqCurrentPattern][i] - 1);
      seqCurrentStep = i;
      handleTransposeChange(seqPatterns[seqCurrentPattern][i]);
      needFullRedraw = true;
      return;
    }

    // スロットタッチ（ステップ選択）
    if (pos.x >= seqSteps[i].slotX && pos.x <= seqSteps[i].slotX + seqSteps[i].slotW &&
        pos.y >= seqSteps[i].slotY && pos.y <= seqSteps[i].slotY + seqSteps[i].slotH) {
      seqCurrentStep = i;
      handleTransposeChange(seqPatterns[seqCurrentPattern][i]);
      needFullRedraw = true;
      return;
    }
  }
}

void adjustWrappedChannel(int8_t& channel, int delta) {
  if (delta > 0) channel = (channel >= 15) ? -1 : channel + 1;
  else channel = (channel < 0) ? 15 : channel - 1;
}

void adjustWrappedData1(int16_t& data1, int delta) {
  if (delta > 0) data1 = (data1 >= 127) ? -1 : data1 + 1;
  else data1 = (data1 < 0) ? 127 : data1 - 1;
}

void adjustWrappedMidiKind(MidiMessageKind& kind, int delta) {
  int value = (int)kind + delta;
  if (value < 0) value = MIDI_KIND_COUNT - 1;
  if (value >= MIDI_KIND_COUNT) value = 0;
  kind = (MidiMessageKind)value;
}

void normalizeMapperRule(MidiMapperRule& rule) {
  int srcMaxValue = getMidiValueMax(rule.srcKind);
  int dstMaxValue = getMidiValueMax(rule.dstKind);

  if (!midiKindSupportsData1(rule.srcKind)) rule.srcData1 = -1;
  if (!midiKindSupportsData1(rule.dstKind)) rule.dstData1 = -1;

  rule.srcMin = clampInt(rule.srcMin, 0, srcMaxValue);
  rule.srcMax = clampInt(rule.srcMax, 0, srcMaxValue);
  rule.dstMin = clampInt(rule.dstMin, 0, dstMaxValue);
  rule.dstMax = clampInt(rule.dstMax, 0, dstMaxValue);

  if (rule.srcMin > rule.srcMax) rule.srcMin = rule.srcMax;
  if (rule.dstMin > rule.dstMax) rule.dstMin = rule.dstMax;
}

void processMidiManageTouch(TouchPoint_t pos) {
  const int topY = midiManageTopButtonY();
  const int topH = midiManageTopButtonH();

  if (touchInRect(pos, 10, topY, 92, topH)) {
    midiManagePage = MIDI_PAGE_FILTER;
    needFullRedraw = true;
    return;
  }
  if (touchInRect(pos, 108, topY, 92, topH)) {
    midiManagePage = MIDI_PAGE_MAPPER;
    needFullRedraw = true;
    return;
  }
  if (touchInRect(pos, 206, topY, 104, topH)) {
    if (midiManagePage == MIDI_PAGE_FILTER) midiFilterBypass = !midiFilterBypass;
    else midiMapperBypass = !midiMapperBypass;
    needFullRedraw = true;
    return;
  }

  int selectedIndex = (midiManagePage == MIDI_PAGE_FILTER) ? midiSelectedFilterRule : midiSelectedMapperRule;
  int ruleCount = (midiManagePage == MIDI_PAGE_FILTER) ? midiFilterRuleCount : midiMapperRuleCount;
  int visibleStart = selectedIndex - 1;
  if (visibleStart < 0) visibleStart = 0;
  if (visibleStart > ruleCount - midiManageVisibleRuleRows()) visibleStart = ruleCount - midiManageVisibleRuleRows();
  if (visibleStart < 0) visibleStart = 0;

  for (int i = 0; i < midiManageVisibleRuleRows(); i++) {
    int ruleIndex = visibleStart + i;
    if (ruleIndex >= ruleCount) break;
    int rowY = midiManageRuleRowY(i);
    if (touchInRect(pos, 10, rowY, 300, midiManageRuleRowH())) {
      if (midiManagePage == MIDI_PAGE_FILTER) midiSelectedFilterRule = ruleIndex;
      else midiSelectedMapperRule = ruleIndex;
      needFullRedraw = true;
      return;
    }
  }

  if (midiManagePage == MIDI_PAGE_FILTER) {
    for (int i = 0; i < 5; i++) {
      int x, y, w, h;
      getMidiActionButtonRect(false, i, x, y, w, h);
      if (!touchInRect(pos, x, y, w, h)) continue;
      if (i == 0) midiFilterRules[midiSelectedFilterRule].enabled = !midiFilterRules[midiSelectedFilterRule].enabled;
      if (i == 1) addDefaultFilterRule();
      if (i == 2) deleteSelectedFilterRule();
      if (i == 3 && midiSelectedFilterRule > 0) {
        MidiFilterRule tmp = midiFilterRules[midiSelectedFilterRule - 1];
        midiFilterRules[midiSelectedFilterRule - 1] = midiFilterRules[midiSelectedFilterRule];
        midiFilterRules[midiSelectedFilterRule] = tmp;
        midiSelectedFilterRule--;
      }
      if (i == 4 && midiSelectedFilterRule < midiFilterRuleCount - 1) {
        MidiFilterRule tmp = midiFilterRules[midiSelectedFilterRule + 1];
        midiFilterRules[midiSelectedFilterRule + 1] = midiFilterRules[midiSelectedFilterRule];
        midiFilterRules[midiSelectedFilterRule] = tmp;
        midiSelectedFilterRule++;
      }
      needFullRedraw = true;
      return;
    }

    if (touchInRect(pos, 10, midiManageEditFullRowY(0), 300, 20)) {
      adjustWrappedMidiKind(midiFilterRules[midiSelectedFilterRule].kind, 1);
      if (!midiKindHasChannel(midiFilterRules[midiSelectedFilterRule].kind)) {
        midiFilterRules[midiSelectedFilterRule].channel = -1;
      }
      needFullRedraw = true;
      return;
    }

    int delta = getMidiInlineEditDelta(pos, 10, midiManageEditFullRowY(1), 300, 20);
    if (delta != 0) {
      adjustWrappedChannel(midiFilterRules[midiSelectedFilterRule].channel, delta);
      needFullRedraw = true;
      return;
    }
  } else {
    MidiMapperRule& rule = midiMapperRules[midiSelectedMapperRule];

    for (int i = 0; i < 6; i++) {
      int x, y, w, h;
      getMidiActionButtonRect(true, i, x, y, w, h);
      if (!touchInRect(pos, x, y, w, h)) continue;
      if (i == 0) rule.enabled = !rule.enabled;
      if (i == 1) addDefaultMapperRule();
      if (i == 2) deleteSelectedMapperRule();
      if (i == 3 && midiSelectedMapperRule > 0) {
        MidiMapperRule tmp = midiMapperRules[midiSelectedMapperRule - 1];
        midiMapperRules[midiSelectedMapperRule - 1] = rule;
        midiMapperRules[midiSelectedMapperRule] = tmp;
        midiSelectedMapperRule--;
      }
      if (i == 4 && midiSelectedMapperRule < midiMapperRuleCount - 1) {
        MidiMapperRule tmp = midiMapperRules[midiSelectedMapperRule + 1];
        midiMapperRules[midiSelectedMapperRule + 1] = rule;
        midiMapperRules[midiSelectedMapperRule] = tmp;
        midiSelectedMapperRule++;
      }
      if (i == 5) midiMapperEditPage = (midiMapperEditPage == MAPPER_PAGE_SOURCE) ? MAPPER_PAGE_DEST : MAPPER_PAGE_SOURCE;
      needFullRedraw = true;
      return;
    }

    if (midiMapperEditPage == MAPPER_PAGE_SOURCE) {
      int delta = getMidiInlineEditDelta(pos, 10, midiManageEditGridRowY(0), 146, midiManageEditRowH());
      if (delta != 0) {
        adjustWrappedMidiKind(rule.srcKind, delta);
        normalizeMapperRule(rule);
        needFullRedraw = true;
        return;
      }
      delta = getMidiInlineEditDelta(pos, 164, midiManageEditGridRowY(0), 146, midiManageEditRowH());
      if (delta != 0) {
        adjustWrappedChannel(rule.srcChannel, delta);
        needFullRedraw = true;
        return;
      }
      delta = getMidiInlineEditDelta(pos, 10, midiManageEditGridRowY(1), 300, midiManageEditRowH());
      if (delta != 0 && midiKindSupportsData1(rule.srcKind)) {
        adjustWrappedData1(rule.srcData1, delta);
        needFullRedraw = true;
        return;
      }
      delta = getMidiInlineEditDelta(pos, 10, midiManageEditGridRowY(2), 146, midiManageEditRowH());
      if (delta != 0) {
        rule.srcMin = clampInt(rule.srcMin + delta, 0, getMidiValueMax(rule.srcKind));
        if (rule.srcMin > rule.srcMax) rule.srcMax = rule.srcMin;
        needFullRedraw = true;
        return;
      }
      delta = getMidiInlineEditDelta(pos, 164, midiManageEditGridRowY(2), 146, midiManageEditRowH());
      if (delta != 0) {
        rule.srcMax = clampInt(rule.srcMax + delta, 0, getMidiValueMax(rule.srcKind));
        if (rule.srcMax < rule.srcMin) rule.srcMin = rule.srcMax;
        needFullRedraw = true;
        return;
      }
    } else {
      int delta = getMidiInlineEditDelta(pos, 10, midiManageEditGridRowY(0), 146, midiManageEditRowH());
      if (delta != 0) {
        adjustWrappedMidiKind(rule.dstKind, delta);
        normalizeMapperRule(rule);
        needFullRedraw = true;
        return;
      }
      delta = getMidiInlineEditDelta(pos, 164, midiManageEditGridRowY(0), 146, midiManageEditRowH());
      if (delta != 0) {
        adjustWrappedChannel(rule.dstChannel, delta);
        needFullRedraw = true;
        return;
      }
      delta = getMidiInlineEditDelta(pos, 10, midiManageEditGridRowY(1), 300, midiManageEditRowH());
      if (delta != 0 && midiKindSupportsData1(rule.dstKind)) {
        adjustWrappedData1(rule.dstData1, delta);
        needFullRedraw = true;
        return;
      }
      delta = getMidiInlineEditDelta(pos, 10, midiManageEditGridRowY(2), 146, midiManageEditRowH());
      if (delta != 0) {
        rule.dstMin = clampInt(rule.dstMin + delta, 0, getMidiValueMax(rule.dstKind));
        if (rule.dstMin > rule.dstMax) rule.dstMax = rule.dstMin;
        needFullRedraw = true;
        return;
      }
      delta = getMidiInlineEditDelta(pos, 164, midiManageEditGridRowY(2), 146, midiManageEditRowH());
      if (delta != 0) {
        rule.dstMax = clampInt(rule.dstMax + delta, 0, getMidiValueMax(rule.dstKind));
        if (rule.dstMax < rule.dstMin) rule.dstMin = rule.dstMax;
        needFullRedraw = true;
        return;
      }
    }
  }
}

const char* getDisplayModeLabel(DisplayMode mode) {
  switch (mode) {
    case DIRECT_MODE: return "DIRECT";
    case KEY_MODE: return "KEY";
    case INSTANT_MODE: return "INSTANT";
    case SEQUENCE_MODE: return "SEQUENCE";
    case MIDI_MANAGE_MODE: return "MIDI_MANAGER";
    case CONFIG_EDIT_MODE: return "CONFIG_EDIT";
    case BASE_SET_MODE: return "BASE_SET";
    default: return "UNKNOWN";
  }
}

const char* getBtStatusLabel(BT_STATUS status) {
  switch (status) {
    case BT_UNINITIALIZED: return "UNINITIALIZED";
    case BT_DISCONNECTED: return "DISCONNECTED";
    case BT_CONNECTING: return "CONNECTING";
    case BT_CONNECTED: return "CONNECTED";
    default: return "UNKNOWN";
  }
}

bool isUsbBinaryTransferActive(void) {
  return g_usbBinaryTransferActive;
}

bool tokenEqualsIgnoreCase(const char* lhs, const char* rhs) {
  if (lhs == nullptr || rhs == nullptr) return false;
  while (*lhs != '\0' && *rhs != '\0') {
    if (toupper((unsigned char)*lhs) != toupper((unsigned char)*rhs)) return false;
    lhs++;
    rhs++;
  }
  return *lhs == '\0' && *rhs == '\0';
}

bool parseIntValue(const char* token, int& outValue) {
  if (token == nullptr || *token == '\0') return false;
  char* endPtr = nullptr;
  long value = strtol(token, &endPtr, 10);
  if (endPtr == token || *endPtr != '\0') return false;
  outValue = (int)value;
  return true;
}

void handleButtonAAction() {
  allNotesOffEnabled = !allNotesOffEnabled;
  needFullRedraw = true;
  Serial.printf("All Notes Off: %s\n", allNotesOffEnabled ? "ON" : "OFF");
}

void handleButtonBAction() {
  if (currentMode == DIRECT_MODE) {
    if (transposeRange == RANGE_0_TO_12) {
      transposeRange = RANGE_MINUS12_TO_0;
    } else if (transposeRange == RANGE_MINUS12_TO_0) {
      transposeRange = RANGE_MINUS5_TO_6;
    } else {
      transposeRange = RANGE_0_TO_12;
    }
    updateDirectButtonLabels();
    setCurrentTransposeButton();
    needFullRedraw = true;
  } else if (currentMode == KEY_MODE) {
    majorUpperTranspose = !majorUpperTranspose;
    minorUpperTranspose = !minorUpperTranspose;
    selectedMajorKey = -1;
    selectedMinorKey = -1;
    needFullRedraw = true;
  } else if (currentMode == MIDI_MANAGE_MODE) {
    if (midiManagePage == MIDI_PAGE_FILTER) {
      adjustWrappedMidiKind(midiFilterRules[midiSelectedFilterRule].kind, 1);
      if (!midiKindHasChannel(midiFilterRules[midiSelectedFilterRule].kind)) {
        midiFilterRules[midiSelectedFilterRule].channel = -1;
      }
      Serial.printf("Filter type: %s\n", getMidiKindLabel(midiFilterRules[midiSelectedFilterRule].kind));
    } else {
      midiMapperEditPage = (midiMapperEditPage == MAPPER_PAGE_SOURCE) ? MAPPER_PAGE_DEST : MAPPER_PAGE_SOURCE;
      Serial.printf("Mapper edit page: PG%d\n", midiMapperEditPage == MAPPER_PAGE_SOURCE ? 1 : 2);
    }
    needFullRedraw = true;
  }

  Serial.println("Mode action (B)");
}

void handleButtonCShortAction() {
  advanceSubMode();
}

void handleButtonCLongAction() {
  advanceDisplayMode();
}

void dispatchTouchPoint(TouchPoint_t pos) {
  if (currentMode == DIRECT_MODE) {
    processDirectModeTouch(pos);
  } else if (currentMode == KEY_MODE) {
    processKeyModeTouch(pos);
  } else if (currentMode == INSTANT_MODE) {
    processInstantModeTouch(pos);
  } else if (currentMode == SEQUENCE_MODE) {
    processSequenceModeTouch(pos);
  } else if (currentMode == CONFIG_EDIT_MODE) {
    processConfigEditTouch(pos);
  } else if (currentMode == BASE_SET_MODE) {
    processBaseSetTouch(pos);
  } else {
    processMidiManageTouch(pos);
  }
}

void injectTouchPoint(int16_t x, int16_t y) {
  TouchPoint_t pos;
  pos.x = x;
  pos.y = y;
  dispatchTouchPoint(pos);
}

bool setGroupFromCommand(const char* groupName) {
  if (tokenEqualsIgnoreCase(groupName, "TRANSPOSE")) {
    DisplayMode restoreMode = isTransposeDisplayMode(lastTransposeMode) ? lastTransposeMode : DIRECT_MODE;
    enterDisplayMode(restoreMode);
    return true;
  }
  if (tokenEqualsIgnoreCase(groupName, "MIDI") ||
      tokenEqualsIgnoreCase(groupName, "MANAGER") ||
      tokenEqualsIgnoreCase(groupName, "MIDI_MANAGER")) {
    enterDisplayMode(MIDI_MANAGE_MODE);
    return true;
  }
  return false;
}

bool setModeFromCommand(const char* modeName) {
  if (tokenEqualsIgnoreCase(modeName, "DIRECT")) {
    enterDisplayMode(DIRECT_MODE);
    return true;
  }
  if (tokenEqualsIgnoreCase(modeName, "KEY")) {
    enterDisplayMode(KEY_MODE);
    return true;
  }
  if (tokenEqualsIgnoreCase(modeName, "INSTANT")) {
    enterDisplayMode(INSTANT_MODE);
    return true;
  }
  if (tokenEqualsIgnoreCase(modeName, "SEQUENCE") || tokenEqualsIgnoreCase(modeName, "SEQ")) {
    enterDisplayMode(SEQUENCE_MODE);
    return true;
  }
  if (tokenEqualsIgnoreCase(modeName, "FILTER")) {
    enterDisplayMode(MIDI_MANAGE_MODE);
    midiManagePage = MIDI_PAGE_FILTER;
    needFullRedraw = true;
    return true;
  }
  if (tokenEqualsIgnoreCase(modeName, "MAPPER")) {
    enterDisplayMode(MIDI_MANAGE_MODE);
    midiManagePage = MIDI_PAGE_MAPPER;
    needFullRedraw = true;
    return true;
  }
  if (tokenEqualsIgnoreCase(modeName, "MIDI") ||
      tokenEqualsIgnoreCase(modeName, "MANAGER") ||
      tokenEqualsIgnoreCase(modeName, "MIDI_MANAGER")) {
    enterDisplayMode(MIDI_MANAGE_MODE);
    return true;
  }
  return false;
}

void printUsbSerialStatus() {
  const char* pageLabel = (midiManagePage == MIDI_PAGE_FILTER) ? "FILTER" : "MAPPER";
  const char* mapperPageLabel = (midiMapperEditPage == MAPPER_PAGE_SOURCE) ? "PG1" : "PG2";
  const char* btLabel = getBtStatusLabel(hid_l2cap_is_connected());
  Serial.printf(
    "OK STATUS mode=%s group=%s transpose=%d range=%d filter_bypass=%d mapper_bypass=%d "
    "filter_rule=%d/%d mapper_rule=%d/%d page=%s mapper_page=%s midi_in=%lu midi_out=%lu bt=%s\n",
    getDisplayModeLabel(currentMode),
    (currentMode == MIDI_MANAGE_MODE) ? "MIDI" : "TRANSPOSE",
    transposeValue,
    (int)transposeRange,
    midiFilterBypass ? 1 : 0,
    midiMapperBypass ? 1 : 0,
    midiSelectedFilterRule + 1, midiFilterRuleCount,
    midiSelectedMapperRule + 1, midiMapperRuleCount,
    pageLabel,
    mapperPageLabel,
    midiInCount,
    midiOutCount,
    btLabel
  );
}

void printUsbSerialHelp() {
  Serial.println("OK HELP BEGIN");
  Serial.println("HELP");
  Serial.println("STATUS");
  Serial.println("REDRAW");
  Serial.println("BUTTON A|B|C [LONG]");
  Serial.println("TOUCH <x> <y>");
  Serial.println("MODE DIRECT|KEY|INSTANT|SEQUENCE|FILTER|MAPPER|MIDI");
  Serial.println("GROUP TRANSPOSE|MIDI");
  Serial.println("SET TRANSPOSE <-11..11>");
  Serial.println("SET FILTER BYPASS 0|1");
  Serial.println("SET FILTER ENABLED <n> 0|1");
  Serial.println("SET MAPPER BYPASS 0|1");
  Serial.println("SET MAPPER ENABLED <n> 0|1");
  Serial.println("LOAD TESTRULES");
  Serial.println("SCREENSHOT [PPM|RGB888]");
  Serial.println("INFO SCREEN");
  Serial.println("OK HELP END");
}

void sendUsbSerialScreenshot(const char* formatToken) {
  bool asPpm = (formatToken == nullptr || tokenEqualsIgnoreCase(formatToken, "PPM"));
  bool asRgb888 = tokenEqualsIgnoreCase(formatToken, "RGB888");

  if (!asPpm && !asRgb888) {
    Serial.println("ERR SCREENSHOT format must be PPM or RGB888");
    return;
  }

  // LCD に直接描画したものを GRAM から読み戻す。
  // PSRAM スプライトは使わない (公式 M5Stack screenServer.ino と同方針)。
  needFullRedraw = false;
  needPartialUpdate = false;
  drawInterface();
  delay(150);

  const size_t totalBytes = (size_t)SCREEN_WIDTH * SCREEN_HEIGHT * 3;

  char header[32];
  size_t headerBytes = 0;
  size_t payloadBytes = totalBytes;
  g_usbBinaryTransferActive = true;
  if (asPpm) {
    headerBytes = (size_t)snprintf(header, sizeof(header), "P6\n%d %d\n255\n", SCREEN_WIDTH, SCREEN_HEIGHT);
    payloadBytes += headerBytes;
    Serial.printf("OK SCREENSHOT format=PPM width=%d height=%d bytes=%u\n",
                  SCREEN_WIDTH, SCREEN_HEIGHT, (unsigned int)payloadBytes);
    Serial.write((const uint8_t*)header, headerBytes);
  } else {
    Serial.printf("OK SCREENSHOT format=RGB888 width=%d height=%d bytes=%u\n",
                  SCREEN_WIDTH, SCREEN_HEIGHT, (unsigned int)payloadBytes);
  }
  Serial.flush();

  // ★最終手段★ 1 ピクセルずつ readPixel で取得する。
  // 各 readPixel は単独で setAddrWindow → READRAM を走らせるので、
  // どんな位置でも横方向 / 縦方向のドリフトが累積しない。
  // 速度は遅い (~10 秒) が、確実にクリーンに取れる。
  static uint8_t rowBuffer[SCREEN_WIDTH * 3];
  for (int y = 0; y < SCREEN_HEIGHT; y++) {
    uint8_t* dst = rowBuffer;
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      uint16_t c = M5.Lcd.readPixel(x, y);  // RGB565
      uint8_t r5 = (c >> 11) & 0x1F;
      uint8_t g6 = (c >> 5)  & 0x3F;
      uint8_t b5 = c & 0x1F;
      *dst++ = (uint8_t)((r5 << 3) | (r5 >> 2));
      *dst++ = (uint8_t)((g6 << 2) | (g6 >> 4));
      *dst++ = (uint8_t)((b5 << 3) | (b5 >> 2));
    }
    Serial.write(rowBuffer, sizeof(rowBuffer));
  }

  Serial.flush();
  g_usbBinaryTransferActive = false;
  Serial.println();
  Serial.println("OK SCREENSHOT_DONE");
}

void handleUsbSerialCommand(char* line) {
  while (*line != '\0' && isspace((unsigned char)*line)) line++;
  if (*line == '\0') return;

  char* end = line + strlen(line) - 1;
  while (end >= line && isspace((unsigned char)*end)) {
    *end = '\0';
    end--;
  }
  if (*line == '\0') return;

  char* savePtr = nullptr;
  char* command = strtok_r(line, " \t", &savePtr);
  if (command == nullptr) return;

  if (tokenEqualsIgnoreCase(command, "HELP")) {
    printUsbSerialHelp();
    return;
  }

  if (tokenEqualsIgnoreCase(command, "STATUS")) {
    printUsbSerialStatus();
    return;
  }

  if (tokenEqualsIgnoreCase(command, "REDRAW")) {
    needFullRedraw = true;
    Serial.println("OK REDRAW");
    return;
  }

  if (tokenEqualsIgnoreCase(command, "INFO")) {
    char* subject = strtok_r(nullptr, " \t", &savePtr);
    if (subject != nullptr && tokenEqualsIgnoreCase(subject, "SCREEN")) {
      Serial.printf("OK SCREEN width=%d height=%d\n", SCREEN_WIDTH, SCREEN_HEIGHT);
      return;
    }
    Serial.println("ERR INFO requires SCREEN");
    return;
  }

  if (tokenEqualsIgnoreCase(command, "BUTTON")) {
    char* button = strtok_r(nullptr, " \t", &savePtr);
    char* modifier = strtok_r(nullptr, " \t", &savePtr);
    if (button == nullptr) {
      Serial.println("ERR BUTTON requires A, B, or C");
      return;
    }

    if (tokenEqualsIgnoreCase(button, "A")) {
      handleButtonAAction();
      Serial.println("OK BUTTON A");
      return;
    }
    if (tokenEqualsIgnoreCase(button, "B")) {
      handleButtonBAction();
      Serial.println("OK BUTTON B");
      return;
    }
    if (tokenEqualsIgnoreCase(button, "C")) {
      if (modifier != nullptr && tokenEqualsIgnoreCase(modifier, "LONG")) {
        handleButtonCLongAction();
        Serial.println("OK BUTTON C LONG");
      } else {
        handleButtonCShortAction();
        Serial.println("OK BUTTON C");
      }
      return;
    }

    Serial.println("ERR BUTTON requires A, B, or C");
    return;
  }

  if (tokenEqualsIgnoreCase(command, "TOUCH")) {
    char* xToken = strtok_r(nullptr, " \t", &savePtr);
    char* yToken = strtok_r(nullptr, " \t", &savePtr);
    int x = 0;
    int y = 0;
    if (!parseIntValue(xToken, x) || !parseIntValue(yToken, y)) {
      Serial.println("ERR TOUCH requires integer x and y");
      return;
    }
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) {
      Serial.println("ERR TOUCH out of range");
      return;
    }
    injectTouchPoint((int16_t)x, (int16_t)y);
    Serial.printf("OK TOUCH %d %d\n", x, y);
    return;
  }

  if (tokenEqualsIgnoreCase(command, "MODE")) {
    char* modeName = strtok_r(nullptr, " \t", &savePtr);
    if (modeName == nullptr) {
      Serial.println("ERR MODE requires a target mode");
      return;
    }
    if (!setModeFromCommand(modeName)) {
      Serial.println("ERR MODE unknown target");
      return;
    }
    Serial.printf("OK MODE %s\n", modeName);
    return;
  }

  if (tokenEqualsIgnoreCase(command, "GROUP")) {
    char* groupName = strtok_r(nullptr, " \t", &savePtr);
    if (groupName == nullptr) {
      Serial.println("ERR GROUP requires TRANSPOSE or MIDI");
      return;
    }
    if (!setGroupFromCommand(groupName)) {
      Serial.println("ERR GROUP unknown target");
      return;
    }
    Serial.printf("OK GROUP %s\n", groupName);
    return;
  }

  if (tokenEqualsIgnoreCase(command, "SET")) {
    char* target = strtok_r(nullptr, " \t", &savePtr);
    if (target != nullptr && tokenEqualsIgnoreCase(target, "TRANSPOSE")) {
      char* valueToken = strtok_r(nullptr, " \t", &savePtr);
      int newValue = 0;
      if (!parseIntValue(valueToken, newValue)) {
        Serial.println("ERR SET TRANSPOSE requires an integer");
        return;
      }
      handleTransposeChange(clampTranspose((int8_t)newValue));
      if (currentMode == DIRECT_MODE) setCurrentTransposeButton();
      needFullRedraw = true;
      Serial.printf("OK SET TRANSPOSE %d\n", transposeValue);
      return;
    }
    if (target != nullptr && (tokenEqualsIgnoreCase(target, "FILTER") || tokenEqualsIgnoreCase(target, "MAPPER"))) {
      const bool isFilter = tokenEqualsIgnoreCase(target, "FILTER");
      char* sub = strtok_r(nullptr, " \t", &savePtr);
      if (sub == nullptr) {
        Serial.printf("ERR SET %s requires BYPASS or ENABLED\n", isFilter ? "FILTER" : "MAPPER");
        return;
      }
      if (tokenEqualsIgnoreCase(sub, "BYPASS")) {
        char* val = strtok_r(nullptr, " \t", &savePtr);
        int v;
        if (!parseIntValue(val, v) || (v != 0 && v != 1)) {
          Serial.printf("ERR SET %s BYPASS requires 0 or 1\n", isFilter ? "FILTER" : "MAPPER");
          return;
        }
        if (isFilter) midiFilterBypass = (v != 0);
        else          midiMapperBypass = (v != 0);
        needFullRedraw = true;
        Serial.printf("OK SET %s BYPASS %d\n", isFilter ? "FILTER" : "MAPPER", v);
        return;
      }
      if (tokenEqualsIgnoreCase(sub, "ENABLED")) {
        char* idxToken = strtok_r(nullptr, " \t", &savePtr);
        char* valToken = strtok_r(nullptr, " \t", &savePtr);
        int idx, v;
        const int ruleCount = isFilter ? midiFilterRuleCount : midiMapperRuleCount;
        if (!parseIntValue(idxToken, idx) || !parseIntValue(valToken, v)
            || idx < 1 || idx > ruleCount || (v != 0 && v != 1)) {
          Serial.printf("ERR SET %s ENABLED <1..%d> 0|1\n",
                        isFilter ? "FILTER" : "MAPPER", ruleCount);
          return;
        }
        if (isFilter) midiFilterRules[idx - 1].enabled = (v != 0);
        else          midiMapperRules[idx - 1].enabled = (v != 0);
        needFullRedraw = true;
        Serial.printf("OK SET %s ENABLED %d %d\n",
                      isFilter ? "FILTER" : "MAPPER", idx, v);
        return;
      }
      Serial.printf("ERR SET %s supports BYPASS or ENABLED\n", isFilter ? "FILTER" : "MAPPER");
      return;
    }
    Serial.println("ERR SET supports TRANSPOSE, FILTER, MAPPER");
    return;
  }

  if (tokenEqualsIgnoreCase(command, "LOAD")) {
    char* target = strtok_r(nullptr, " \t", &savePtr);
    if (target != nullptr && tokenEqualsIgnoreCase(target, "TESTRULES")) {
      loadTestRules();
      needFullRedraw = true;
      Serial.printf("OK LOAD TESTRULES filter=%d mapper=%d\n",
                    midiFilterRuleCount, midiMapperRuleCount);
      return;
    }
    Serial.println("ERR LOAD supports only TESTRULES");
    return;
  }

  if (tokenEqualsIgnoreCase(command, "SCREENSHOT")) {
    char* format = strtok_r(nullptr, " \t", &savePtr);
    sendUsbSerialScreenshot(format);
    return;
  }

  Serial.println("ERR Unknown command");
}

void processUsbSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();
    if (incoming == '\r') continue;
    if (incoming == '\n') {
      usbCommandBuffer[usbCommandLength] = '\0';
      handleUsbSerialCommand(usbCommandBuffer);
      usbCommandLength = 0;
      continue;
    }
    if (usbCommandLength < (USB_COMMAND_BUFFER_SIZE - 1)) {
      usbCommandBuffer[usbCommandLength++] = incoming;
    }
  }
}

void sendAllNotesOff() {
  for (int channel = 0; channel < 16; channel++) {
    Serial2.write(0xB0 | channel);
    Serial2.write(123);
    Serial2.write(0);
    
    Serial2.write(0xB0 | channel);
    Serial2.write(120);
    Serial2.write(0);
    
    midiOutCount += 6;
  }

  clearTrackedNoteStates();
  
  Serial.println("All Notes Off sent");
}

void processMIDI() {
  // Cap the work done per loop iteration. A keyboard player driving heavy
  // running-status + clock + chord traffic can sustain bursts that outpace
  // the 31.25 kbaud MIDI OUT, and Serial2.write() blocks once the TX ring
  // is full. Without a budget, draining-to-empty inside a single loop()
  // iteration prevents M5.update() / LCD refresh / Bluetooth servicing /
  // the IDLE task from running and the task watchdog resets the device.
  // Unconsumed bytes stay in the Serial2 RX ring and are picked up on the
  // next loop iteration. 3 ms keeps response latency below one MIDI clock
  // tick at 240 BPM.
  const uint32_t startUs = micros();
  const uint32_t kBudgetUs = 3000UL;
  bool sawInput = false;
  while (Serial2.available()) {
    if ((micros() - startUs) >= kBudgetUs) break;
    uint8_t incomingByte = Serial2.read();
    sawInput = true;
    midiInCount++;
    processMIDIByte(incomingByte);
  }
  if (sawInput) {
    g_lastMidiInputAt = millis();
  }
}

void handleParsedMidiMessage(const MidiMessage& inMsg) {
  if (!shouldAllowMidiMessage(inMsg)) return;
  MidiMessage mappedMsg = applyMidiMapper(inMsg);
  sendMIDIMessage(mappedMsg.bytes, mappedMsg.length);
}

void processMIDIByte(uint8_t midiData) {
  static uint8_t midiBuffer[3];
  static int bufferIndex = 0;
  static uint8_t runningStatus = 0;
  static uint8_t currentStatus = 0;
  static bool inSysEx = false;
  static bool allowCurrentSysEx = true;

  if (midiData >= 0xF8) {
    MidiMessage msg;
    msg.bytes[0] = midiData;
    msg.length = 1;
    msg.kind = getMidiKindFromStatus(midiData);
    msg.hasChannel = false;
    msg.channel = -1;
    handleParsedMidiMessage(msg);
    return;
  }

  if (inSysEx) {
    // Defensive: a non-realtime status byte during SysEx is illegal but
    // some devices emit it. Without this escape we would stay in SysEx
    // forever and every subsequent message bypasses transpose / mapper,
    // which manifests as stuck notes after a glitchy device sends bad SysEx.
    if (midiData >= 0x80 && midiData != 0xF7) {
      if (allowCurrentSysEx) {
        Serial2.write((uint8_t)0xF7);  // close SysEx synthetically
        midiOutCount++;
      }
      inSysEx = false;
      // fall through so the new status byte is parsed normally
    } else {
      if (allowCurrentSysEx) {
        Serial2.write(midiData);
        midiOutCount++;
      }
      if (midiData == 0xF7) {
        inSysEx = false;
      }
      return;
    }
  }

  if (midiData == 0xF0) {
    MidiMessage sysExMsg;
    sysExMsg.bytes[0] = 0xF0;
    sysExMsg.length = 1;
    sysExMsg.kind = MIDI_KIND_SYSTEM_EXCLUSIVE;
    sysExMsg.hasChannel = false;
    sysExMsg.channel = -1;

    inSysEx = true;
    allowCurrentSysEx = shouldAllowMidiMessage(sysExMsg);
    if (allowCurrentSysEx) {
      Serial2.write(midiData);
      midiOutCount++;
    }
    return;
  }

  if (midiData & 0x80) {
    currentStatus = midiData;
    midiBuffer[0] = midiData;
    bufferIndex = 1;

    if (midiData < 0xF0) runningStatus = midiData;
    else runningStatus = 0;

    int messageLength = getMIDIMessageLength(currentStatus);
    if (messageLength == 1) {
      MidiMessage msg;
      msg.bytes[0] = currentStatus;
      msg.length = 1;
      msg.kind = getMidiKindFromStatus(currentStatus);
      msg.hasChannel = false;
      msg.channel = -1;
      handleParsedMidiMessage(msg);
      bufferIndex = 0;
      return;
    }
  } else if (bufferIndex > 0) {
    midiBuffer[bufferIndex++] = midiData;
  } else if (runningStatus != 0) {
    currentStatus = runningStatus;
    midiBuffer[0] = currentStatus;
    midiBuffer[1] = midiData;
    bufferIndex = 2;
  } else {
    return;
  }

  int messageLength = getMIDIMessageLength(currentStatus);
  if (bufferIndex >= messageLength) {
    MidiMessage msg;
    msg.length = messageLength;
    msg.kind = getMidiKindFromStatus(currentStatus);
    msg.hasChannel = midiKindHasChannel(msg.kind);
    msg.channel = msg.hasChannel ? (currentStatus & 0x0F) : -1;
    for (int i = 0; i < messageLength; i++) {
      msg.bytes[i] = midiBuffer[i];
    }
    handleParsedMidiMessage(msg);
    bufferIndex = 0;
  }
}

int getMIDIMessageLength(uint8_t status) {
  if ((status & 0xF0) == 0x80 || (status & 0xF0) == 0x90 ||
      (status & 0xF0) == 0xA0 || (status & 0xF0) == 0xB0 ||
      (status & 0xF0) == 0xE0) {
    return 3;
  }
  if ((status & 0xF0) == 0xC0 || (status & 0xF0) == 0xD0) {
    return 2;
  }

  switch (status) {
    case 0xF1:
    case 0xF3:
      return 2;
    case 0xF2:
      return 3;
    default:
      return 1;
  }
}

void sendMIDIMessage(uint8_t* buffer, int length) {
  uint8_t status = buffer[0];
  uint8_t messageType = status & 0xF0;
  uint8_t channel = status & 0x0F;
  
  if ((messageType == 0x90 || messageType == 0x80) && length == 3) {
    uint8_t note = buffer[1];
    uint8_t velocity = buffer[2];
    
    bool isNoteOn = (messageType == 0x90) && (velocity > 0);
    int noteStateIndex = getTrackedNoteStateIndex(note, channel);
    
    if (isNoteOn) {
      int16_t transposedNote = note + transposeValue;
      if (transposedNote >= 0 && transposedNote <= 127) {
        Serial2.write(status);
        Serial2.write((uint8_t)transposedNote);
        Serial2.write(velocity);
        midiOutCount += 3;
        
        if (noteStateIndex >= 0) {
          currentNoteStates[noteStateIndex].isActive = true;
          currentNoteStates[noteStateIndex].originalTranspose = transposeValue;
          currentNoteStates[noteStateIndex].channel = channel;
          currentNoteStates[noteStateIndex].velocity = velocity;
        }
      }
    } else {
      int16_t transposedNote;
      bool shouldSendNoteOff = true;
      
      if (noteStateIndex >= 0) {
        if (currentNoteStates[noteStateIndex].isActive) {
          transposedNote = note + currentNoteStates[noteStateIndex].originalTranspose;
          currentNoteStates[noteStateIndex].isActive = false;
        } else if (savedNoteStates[noteStateIndex].isActive) {
          transposedNote = note + savedNoteStates[noteStateIndex].originalTranspose;
          savedNoteStates[noteStateIndex].isActive = false;
        } else {
          transposedNote = note + transposeValue;
        }
      } else {
        transposedNote = note + transposeValue;
      }
      
      if (transposedNote >= 0 && transposedNote <= 127 && shouldSendNoteOff) {
        Serial2.write(status);
        Serial2.write((uint8_t)transposedNote);
        Serial2.write(velocity);
        midiOutCount += 3;
      }
    }
  } else {
    for (int i = 0; i < length; i++) {
      Serial2.write(buffer[i]);
    }
    midiOutCount += length;
  }
}
