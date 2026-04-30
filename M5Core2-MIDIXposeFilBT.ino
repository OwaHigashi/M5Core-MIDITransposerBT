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

// M5 Unit MIDI (SAM2695) is connected to M5Core2 Port A (Grove).
// Port A pins: G32 (SDA) / G33 (SCL). They are repurposed here as UART2.
// Per M5Unit-Synth example for Core2: synth.begin(&Serial2, 31250, 33, 32);
//   -> Serial2 RX = G33 (host receives from Unit MIDI's TX)
//   -> Serial2 TX = G32 (host transmits to Unit MIDI's RX)
static constexpr uint32_t MIDI_UART_BAUD = 31250;
static constexpr int MIDI_UART_RX_PIN = 33;
static constexpr int MIDI_UART_TX_PIN = 32;

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
  PLAY_MODE,
  DIRECT_MODE,
  KEY_MODE,
  INSTANT_MODE,
  SEQUENCE_MODE,
  MIDI_MANAGE_MODE
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

typedef enum {
    GrandPiano_1 = 0,
    BrightPiano_2,
    ElGrdPiano_3,
    HonkyTonkPiano,
    ElPiano1,
    ElPiano2,
    Harpsichord,
    Clavi,
    Celesta,
    Glockenspiel,
    MusicBox,
    Vibraphone,
    Marimba,
    Xylophone,
    TubularBells,
    Santur,
    DrawbarOrgan,
    PercussiveOrgan,
    RockOrgan,
    ChurchOrgan,
    ReedOrgan,
    AccordionFrench,
    Harmonica,
    TangoAccordion,
    AcGuitarNylon,
    AcGuitarSteel,
    AcGuitarJazz,
    AcGuitarClean,
    AcGuitarMuted,
    OverdrivenGuitar,
    DistortionGuitar,
    GuitarHarmonics,
    AcousticBass,
    FingerBass,
    PickedBass,
    FretlessBass,
    SlapBass1,
    SlapBass2,
    SynthBass1,
    SynthBass2,
    Violin,
    Viola,
    Cello,
    Contrabass,
    TremoloStrings,
    PizzicatoStrings,
    OrchestralHarp,
    Timpani,
    StringEnsemble1,
    StringEnsemble2,
    SynthStrings1,
    SynthStrings2,
    ChoirAahs,
    VoiceOohs,
    SynthVoice,
    OrchestraHit,
    Trumpet,
    Trombone,
    Tuba,
    MutedTrumpet,
    FrenchHorn,
    BrassSection,
    SynthBrass1,
    SynthBrass2,
    SopranoSax,
    AltoSax,
    TenorSax,
    BaritoneSax,
    Oboe,
    EnglishHorn,
    Bassoon,
    Clarinet,
    Piccolo,
    Flute,
    Recorder,
    PanFlute,
    BlownBottle,
    Shakuhachi,
    Whistle,
    Ocarina,
    Lead1Square,
    Lead2Sawtooth,
    Lead3Calliope,
    Lead4Chiff,
    Lead5Charang,
    Lead6Voice,
    Lead7Fifths,
    Lead8BassLead,
    Pad1Fantasia,
    Pad2Warm,
    Pad3PolySynth,
    Pad4Choir,
    Pad5Bowed,
    Pad6Metallic,
    Pad7Halo,
    Pad8Sweep,
    FX1Rain,
    FX2Soundtrack,
    FX3Crystal,
    FX4Atmosphere,
    FX5Brightness,
    FX6Goblins,
    FX7Echoes,
    FX8SciFi,
    Sitar,
    Banjo,
    Shamisen,
    Koto,
    Kalimba,
    BagPipe,
    Fiddle,
    Shanai,
    TinkleBell,
    Agogo,
    SteelDrums,
    Woodblock,
    TaikoDrum,
    MelodicTom,
    SynthDrum,
    ReverseCymbal,
    GtFretNoise,
    BreathNoise,
    Seashore,
    BirdTweet,
    TelephRing,
    Helicopter,
    Applause,
    Gunshot,
} unit_synth_instrument_t;

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
DisplayMode currentMode = PLAY_MODE;
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

enum PlayControlButtonIndex {
  PLAY_BTN_VOL_DOWN,
  PLAY_BTN_VOL_UP,
  PLAY_BTN_PROG_DOWN,
  PLAY_BTN_PROG_UP,
  PLAY_BTN_BEND_DOWN,
  PLAY_BTN_BEND_UP,
  PLAY_BTN_SUSTAIN,
  PLAY_BTN_INIT,
  PLAY_BTN_COUNT
};

enum PlayProgramPickerButtonIndex {
  PLAY_PICKER_PREV,
  PLAY_PICKER_NEXT,
  PLAY_PICKER_CLOSE,
  PLAY_PICKER_COUNT
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
void drawPlayMode(void);
void drawPlayProgramPicker(void);
void processPlayModeTouch(TouchPoint_t pos);
void processPlayProgramPickerTouch(TouchPoint_t pos);
void drawMidiActionButton(int x, int y, int w, int h, const char* label, uint16_t fillColor);
void sendPlayTestPhrase(void);
void sendMidiControlChange(uint8_t channel, uint8_t controller, uint8_t value);
void sendMidiProgramChange(uint8_t channel, uint8_t program);
void sendMidiPitchBend(uint8_t channel, uint16_t bendValue);
void sendPlayModeInit(bool resetProgramAndVolume);
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

NavButton playButtons[PLAY_BTN_COUNT];
NavButton playProgramPickerButtons[PLAY_PICKER_COUNT];
NavButton playProgramNameBar;
NavButton playTestBtn;
uint8_t playVolume = 100;
uint8_t playProgram = 0;
uint16_t playPitchBend = 8192;
bool playSustain = false;
uint8_t playActiveChannel = 0;
bool playProgramPickerOpen = false;
uint8_t playProgramPickerPage = 0;
const uint8_t PLAY_VOLUME_STEP = 8;
const uint16_t PLAY_PITCH_BEND_STEP = 256;
static constexpr uint8_t PLAY_PROGRAMS_PER_PAGE = 6;

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
char g_lastMidiRxLabel[32] = "RX:--";
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

static const char* const gUnitSynthInstrumentNames[] = {
  "Grand Piano 1", "Bright Piano 2", "El Grd Piano 3", "Honky-Tonk Piano",
  "Electric Piano 1", "Electric Piano 2", "Harpsichord", "Clavi",
  "Celesta", "Glockenspiel", "Music Box", "Vibraphone",
  "Marimba", "Xylophone", "Tubular Bells", "Santur",
  "Drawbar Organ", "Percussive Organ", "Rock Organ", "Church Organ",
  "Reed Organ", "Accordion French", "Harmonica", "Tango Accordion",
  "Acoustic Guitar Nylon", "Acoustic Guitar Steel", "Acoustic Guitar Jazz", "Acoustic Guitar Clean",
  "Acoustic Guitar Muted", "Overdriven Guitar", "Distortion Guitar", "Guitar Harmonics",
  "Acoustic Bass", "Finger Bass", "Picked Bass", "Fretless Bass",
  "Slap Bass 1", "Slap Bass 2", "Synth Bass 1", "Synth Bass 2",
  "Violin", "Viola", "Cello", "Contrabass",
  "Tremolo Strings", "Pizzicato Strings", "Orchestral Harp", "Timpani",
  "String Ensemble 1", "String Ensemble 2", "Synth Strings 1", "Synth Strings 2",
  "Choir Aahs", "Voice Oohs", "Synth Voice", "Orchestra Hit",
  "Trumpet", "Trombone", "Tuba", "Muted Trumpet",
  "French Horn", "Brass Section", "Synth Brass 1", "Synth Brass 2",
  "Soprano Sax", "Alto Sax", "Tenor Sax", "Baritone Sax",
  "Oboe", "English Horn", "Bassoon", "Clarinet",
  "Piccolo", "Flute", "Recorder", "Pan Flute",
  "Blown Bottle", "Shakuhachi", "Whistle", "Ocarina",
  "Lead 1 Square", "Lead 2 Sawtooth", "Lead 3 Calliope", "Lead 4 Chiff",
  "Lead 5 Charang", "Lead 6 Voice", "Lead 7 Fifths", "Lead 8 Bass + Lead",
  "Pad 1 Fantasia", "Pad 2 Warm", "Pad 3 PolySynth", "Pad 4 Choir",
  "Pad 5 Bowed", "Pad 6 Metallic", "Pad 7 Halo", "Pad 8 Sweep",
  "FX 1 Rain", "FX 2 Soundtrack", "FX 3 Crystal", "FX 4 Atmosphere",
  "FX 5 Brightness", "FX 6 Goblins", "FX 7 Echoes", "FX 8 Sci-Fi",
  "Sitar", "Banjo", "Shamisen", "Koto",
  "Kalimba", "Bag Pipe", "Fiddle", "Shanai",
  "Tinkle Bell", "Agogo", "Steel Drums", "Woodblock",
  "Taiko Drum", "Melodic Tom", "Synth Drum", "Reverse Cymbal",
  "Guitar Fret Noise", "Breath Noise", "Seashore", "Bird Tweet",
  "Telephone Ring", "Helicopter", "Applause", "Gunshot"
};

const char* getUnitSynthInstrumentName(uint8_t program) {
  if (program >= (sizeof(gUnitSynthInstrumentNames) / sizeof(gUnitSynthInstrumentNames[0]))) {
    return "Unknown";
  }
  return gUnitSynthInstrumentNames[program];
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
  const int W = SCREEN_WIDTH;
  const int H = SCREEN_HEIGHT;

  M5.Lcd.fillScreen(BLACK);

  // 上下のグラデーションバー (シアン→マゼンタ)
  for (int i = 0; i < 6; i++) {
    uint8_t r = (uint8_t)((i * 220) / 5);
    uint8_t g = (uint8_t)(180 - i * 28);
    uint8_t b = (uint8_t)(255 - i * 8);
    uint16_t c = M5.Lcd.color565(r, g, b);
    M5.Lcd.drawFastHLine(0, 26 + i, W, c);
    M5.Lcd.drawFastHLine(0, H - 32 + i, W, c);
  }

  // 左右の縦アクセント線
  M5.Lcd.drawFastVLine(6,     26, H - 58, M5.Lcd.color565(0, 200, 255));
  M5.Lcd.drawFastVLine(W - 7, 26, H - 58, M5.Lcd.color565(220, 0, 255));

  // タイトル "OWAMIDICON" — グロー風 3 段シャドウ
  uiFontHuge();
  M5.Lcd.setTextDatum(MC_DATUM);
  const char* title = "OWAMIDICON";
  const int titleY = H / 2 - 22;
  M5.Lcd.setTextColor(M5.Lcd.color565(0,  30,  90));
  M5.Lcd.drawString(title, W / 2 + 3, titleY + 3);
  M5.Lcd.setTextColor(M5.Lcd.color565(0, 110, 200));
  M5.Lcd.drawString(title, W / 2 + 1, titleY + 1);
  M5.Lcd.setTextColor(CYAN);
  M5.Lcd.drawString(title, W / 2,     titleY);

  // タイトル下の細い装飾ライン
  M5.Lcd.drawFastHLine(W / 2 - 90, titleY + 24, 180, M5.Lcd.color565(0, 180, 255));
  M5.Lcd.drawFastHLine(W / 2 - 60, titleY + 27, 120, M5.Lcd.color565(180, 0, 220));

  // サブタイトル
  uiFontSmall();
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.drawString("MIDI Transposer + MIDI Message Manager", W / 2, H / 2 + 28);

  // エディション表記
  M5.Lcd.setTextColor(M5.Lcd.color565(200, 80, 220));
  M5.Lcd.drawString("M5Stack Core2 Edition", W / 2, H / 2 + 50);

  // 下部ローディングバーのアニメーション
  const int barX = 30;
  const int barY = H - 50;
  const int barW = W - 60;
  const int barH = 4;
  M5.Lcd.drawRect(barX - 1, barY - 1, barW + 2, barH + 2, DARKGREY);
  for (int i = 0; i <= barW; i += 4) {
    uint8_t r = (uint8_t)((i * 220) / barW);
    uint8_t b = (uint8_t)(255 - (i * 60) / barW);
    uint16_t c = M5.Lcd.color565(r, 100, b);
    M5.Lcd.fillRect(barX + (i - 4 < 0 ? 0 : i - 4), barY, 4, barH, c);
    delay(4);
  }

  delay(700);
}

void setup() {
  M5.begin(true, true, true, true);

  // LCD に直接描画する (PSRAM スプライトの fillRect memcpy バグを回避)。
  // スクリーンショットも公式 M5Stack の TFT_Screen_Capture と同様、
  // LCD GRAM から行ごと readRectRGB で吸い出す。

  // for SD-Updater
  checkSDUpdater( SD, MENU_BIN, 2000, TFCARD_CS_PIN );

  Serial.begin(115200);

  // M5.begin(..., true) initializes Wire on Port A (G32/G33) for I2C.
  // We need those pins for UART instead, so release the I2C bus first.
  // (Touch / AXP / IMU live on the internal Wire1 bus on G21/G22 and are unaffected.)
  Wire.end();

  Serial2.begin(MIDI_UART_BAUD, SERIAL_8N1, MIDI_UART_RX_PIN, MIDI_UART_TX_PIN);
  Serial2.setRxBufferSize(1024);
  Serial2.setTxBufferSize(512);
  Serial2.setTimeout(10);
  // Per M5Unit-Synth: pull-up RX so the line does not float when Unit MIDI is idle.
  pinMode(MIDI_UART_RX_PIN, INPUT_PULLUP);
  
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
  enterDisplayMode(PLAY_MODE);

  // SDカード状態確認
  if (SD.cardType() == CARD_NONE) {
    Serial.println("[SD] WARNING: No SD card detected!");
  } else {
    Serial.printf("[SD] Card type: %d, Size: %lluMB\n", SD.cardType(), SD.cardSize() / (1024 * 1024));
  }

  // SDカードからシーケンスパターンを読み込み
  Serial.println("Loading sequence patterns from SD...");
  loadSequencesFromSD();

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
  showSplashScreen();
  drawInterface();

  Serial.println("MIDI Transposer Ready!");
  Serial.printf("Initial transpose: %d, Button 5 state: %s\n", transposeValue, transposeButtons[5] ? "ON" : "OFF");
  Serial.println("USB serial command interface ready. Type HELP for commands.");
}

void loop() {
  processMIDI();

  static unsigned long lastUICheck = 0;
  unsigned long now = millis();

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
  if (currentMode == PLAY_MODE) {
    uiDrawL("MIDI Player", 10, 2);
  } else if (currentMode == MIDI_MANAGE_MODE) {
    uiDrawL(midiManagePage == MIDI_PAGE_FILTER ? "MIDI Filter" : "MIDI Mapper", 10, 2);
  } else {
    uiDrawL("MIDI Transposer", 10, 2);
  }

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
    if (currentMode == PLAY_MODE) {
      hint = "Tap PRG name  B:Init  C:Panic";
    } else if (currentMode == MIDI_MANAGE_MODE) {
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
  } else if (currentMode == PLAY_MODE) {
    drawPlayMode();
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

void layoutPlayModeButtons() {
  const int marginX = 10;
  const int gap = 6;
  const int btnW = (SCREEN_WIDTH - marginX * 2 - gap * 3) / 4;
  const int btnH = 28;
  const int row1Y = 126;
  const int row2Y = 158;

  playButtons[PLAY_BTN_VOL_DOWN]   = { marginX + 0 * (btnW + gap), row1Y, btnW, btnH };
  playButtons[PLAY_BTN_VOL_UP]     = { marginX + 1 * (btnW + gap), row1Y, btnW, btnH };
  playButtons[PLAY_BTN_PROG_DOWN]  = { marginX + 2 * (btnW + gap), row1Y, btnW, btnH };
  playButtons[PLAY_BTN_PROG_UP]    = { marginX + 3 * (btnW + gap), row1Y, btnW, btnH };

  playButtons[PLAY_BTN_BEND_DOWN]  = { marginX + 0 * (btnW + gap), row2Y, btnW, btnH };
  playButtons[PLAY_BTN_BEND_UP]    = { marginX + 1 * (btnW + gap), row2Y, btnW, btnH };
  playButtons[PLAY_BTN_SUSTAIN]    = { marginX + 2 * (btnW + gap), row2Y, btnW, btnH };
  playButtons[PLAY_BTN_INIT]       = { marginX + 3 * (btnW + gap), row2Y, btnW, btnH };

  // 3 行ステータス (y=44/60/76, 各 16px 間隔) の下に PRG 名バー (h=28)
  playProgramNameBar = { 10, 94, SCREEN_WIDTH - 20, 28 };
  // TEST PHRASE は薄め (34 → 28)
  playTestBtn = { 10, 192, SCREEN_WIDTH - 20, 28 };
}

void layoutPlayProgramPicker() {
  const int x = 10;
  const int y = 44;
  const int w = SCREEN_WIDTH - 20;
  const int h = SCREEN_HEIGHT - y - 10;
  const int headerH = 20;
  const int footerH = 28;

  playProgramPickerButtons[PLAY_PICKER_PREV]  = { x, y + h - footerH, 74, footerH - 2 };
  playProgramPickerButtons[PLAY_PICKER_NEXT]  = { x + w - 74, y + h - footerH, 74, footerH - 2 };
  playProgramPickerButtons[PLAY_PICKER_CLOSE] = { x + 76, y + h - footerH, w - 152, footerH - 2 };
}

void drawPlayProgramPicker() {
  layoutPlayProgramPicker();

  const int x = 10;
  const int y = 44;
  const int w = SCREEN_WIDTH - 20;
  const int h = SCREEN_HEIGHT - y - 10;
  const int headerH = 20;
  const int itemH = 18;
  const int itemGap = 2;
  const int startProgram = playProgramPickerPage * PLAY_PROGRAMS_PER_PAGE;
  const int endProgram = min(128, startProgram + PLAY_PROGRAMS_PER_PAGE);

  // 下のレイヤ (PLAY 本体) の文字が右端に残らないよう、ピッカー領域より少し広く塗り潰す
  M5.Lcd.fillRect(0, 42, SCREEN_WIDTH, SCREEN_HEIGHT - 42, BLACK);

  M5.Lcd.fillRoundRect(x, y, w, h, 6, BLACK);
  M5.Lcd.drawRoundRect(x, y, w, h, 6, GREEN);

  uiFontSmall();
  M5.Lcd.setTextColor(CYAN);
  uiDrawL("Select Program", x + 8, y + 4);

  char pageLine[32];
  M5.Lcd.setTextColor(YELLOW);
  snprintf(pageLine, sizeof(pageLine), "%03d-%03d / 128",
           startProgram + 1, endProgram);
  uiDrawL(pageLine, x + w - 110, y + 4);

  for (int i = 0; i < PLAY_PROGRAMS_PER_PAGE; i++) {
    int program = startProgram + i;
    int itemY = y + headerH + 4 + i * (itemH + itemGap);
    if (program >= 128) {
      M5.Lcd.fillRect(x + 4, itemY, w - 8, itemH, BLACK);
      continue;
    }

    bool selected = (program == playProgram);
    uint16_t fillColor = selected ? GREEN : DARKGREY;
    uint16_t textColor = selected ? BLACK : WHITE;
    M5.Lcd.fillRoundRect(x + 4, itemY, w - 8, itemH, 4, fillColor);
    M5.Lcd.drawRoundRect(x + 4, itemY, w - 8, itemH, 4, WHITE);

    char itemLine[64];
    snprintf(itemLine, sizeof(itemLine), "%03d  %s", program + 1, getUnitSynthInstrumentName((uint8_t)program));
    M5.Lcd.setTextColor(textColor);
    uiDrawL(itemLine, x + 10, itemY + 1);
  }

  drawMidiActionButton(playProgramPickerButtons[PLAY_PICKER_PREV].x, playProgramPickerButtons[PLAY_PICKER_PREV].y,
                       playProgramPickerButtons[PLAY_PICKER_PREV].w, playProgramPickerButtons[PLAY_PICKER_PREV].h,
                       "< PREV", NAVY);
  drawMidiActionButton(playProgramPickerButtons[PLAY_PICKER_NEXT].x, playProgramPickerButtons[PLAY_PICKER_NEXT].y,
                       playProgramPickerButtons[PLAY_PICKER_NEXT].w, playProgramPickerButtons[PLAY_PICKER_NEXT].h,
                       "NEXT >", NAVY);
  drawMidiActionButton(playProgramPickerButtons[PLAY_PICKER_CLOSE].x, playProgramPickerButtons[PLAY_PICKER_CLOSE].y,
                       playProgramPickerButtons[PLAY_PICKER_CLOSE].w, playProgramPickerButtons[PLAY_PICKER_CLOSE].h,
                       "CLOSE", RED);
}

void drawPlayMode() {
  layoutPlayModeButtons();

  // 3 行のステータス。FSS9 の実フォント高は約 14px なので 16px 間隔で配置する
  uiFontSmall();
  char line[64];

  // 1段目 (y=44): タイトル
  M5.Lcd.setTextColor(CYAN);
  uiDrawL("SAM2695 Live", 10, 44);

  // 2段目 (y=60): VOL / PRG / PB
  M5.Lcd.setTextColor(YELLOW);
  snprintf(line, sizeof(line), "VOL:%03u  PRG:%03u  PB:%5u",
           playVolume, (unsigned)(playProgram + 1), (unsigned)playPitchBend);
  uiDrawL(line, 10, 60);

  // 3段目 (y=76): CH / SUS / OUT
  M5.Lcd.setTextColor(DARKGREY);
  snprintf(line, sizeof(line), "CH:%02u  SUS:%s  OUT:%lu",
           (unsigned)(playActiveChannel + 1), playSustain ? "ON " : "OFF", midiOutCount);
  uiDrawL(line, 10, 76);

  // PRG 名バー (h=30 に拡張)
  M5.Lcd.fillRoundRect(playProgramNameBar.x, playProgramNameBar.y,
                       playProgramNameBar.w, playProgramNameBar.h, 6,
                       playProgramPickerOpen ? DARKGREY : NAVY);
  M5.Lcd.drawRoundRect(playProgramNameBar.x, playProgramNameBar.y,
                       playProgramNameBar.w, playProgramNameBar.h, 6, WHITE);

  // バー内テキストは縦中央寄せ
  uiFontSmall();
  const int barTextY = playProgramNameBar.y + (playProgramNameBar.h - M5.Lcd.fontHeight()) / 2;

  // 右側の "TAP" ヒントを先に右寄せで配置し、その左に PRG 名を入れる
  M5.Lcd.setTextColor(LIGHTGREY);
  const char* hint = "TAP";
  const int hintW = M5.Lcd.textWidth(hint);
  const int hintX = playProgramNameBar.x + playProgramNameBar.w - hintW - 10;
  uiDrawL(hint, hintX, barTextY);

  M5.Lcd.setTextColor(WHITE);
  snprintf(line, sizeof(line), "PRG:%03u  %s",
           (unsigned)(playProgram + 1), getUnitSynthInstrumentName(playProgram));
  uiDrawL(line, playProgramNameBar.x + 8, barTextY);

  const uint16_t buttonColors[PLAY_BTN_COUNT] = {
    NAVY, NAVY, GREEN, GREEN, DARKGREY, DARKGREY, (playSustain ? GREEN : RED), RED
  };
  const char* labels[PLAY_BTN_COUNT] = {
    "VOL-", "VOL+", "PRG-", "PRG+", "PB-", "PB+", "SUS", "INIT"
  };

  for (int i = 0; i < PLAY_BTN_COUNT; i++) {
    drawMidiActionButton(playButtons[i].x, playButtons[i].y, playButtons[i].w, playButtons[i].h,
                         labels[i], buttonColors[i]);
  }

  drawMidiActionButton(playTestBtn.x, playTestBtn.y, playTestBtn.w, playTestBtn.h, "TEST PHRASE", ORANGE);

  if (playProgramPickerOpen) {
    drawPlayProgramPicker();
  }
}

void processPlayModeTouch(TouchPoint_t pos) {
  layoutPlayModeButtons();

  if (playProgramPickerOpen) {
    processPlayProgramPickerTouch(pos);
    return;
  }

  if (touchInRect(pos, playProgramNameBar.x, playProgramNameBar.y, playProgramNameBar.w, playProgramNameBar.h)) {
    playProgramPickerOpen = true;
    playProgramPickerPage = playProgram / PLAY_PROGRAMS_PER_PAGE;
    needFullRedraw = true;
    Serial.printf("Play program picker open: page=%u\n", playProgramPickerPage + 1);
    return;
  }

  if (touchInRect(pos, playTestBtn.x, playTestBtn.y, playTestBtn.w, playTestBtn.h)) {
    sendPlayTestPhrase();
    needFullRedraw = true;
    return;
  }

  for (int i = 0; i < PLAY_BTN_COUNT; i++) {
    if (!touchInRect(pos, playButtons[i].x, playButtons[i].y, playButtons[i].w, playButtons[i].h)) {
      continue;
    }

    switch (i) {
      case PLAY_BTN_VOL_DOWN:
        if (playVolume > PLAY_VOLUME_STEP) playVolume = (uint8_t)(playVolume - PLAY_VOLUME_STEP);
        else playVolume = 0;
        sendMidiControlChange(playActiveChannel, 7, playVolume);
        Serial.printf("Play volume: %u\n", playVolume);
        break;
      case PLAY_BTN_VOL_UP:
        if (playVolume >= 127 - PLAY_VOLUME_STEP) playVolume = 127;
        else playVolume = (uint8_t)(playVolume + PLAY_VOLUME_STEP);
        sendMidiControlChange(playActiveChannel, 7, playVolume);
        Serial.printf("Play volume: %u\n", playVolume);
        break;
      case PLAY_BTN_PROG_DOWN:
        if (playProgram > 0) playProgram--;
        else playProgram = 127;
        sendMidiProgramChange(playActiveChannel, playProgram);
        Serial.printf("Play program: %u\n", (unsigned)(playProgram + 1));
        break;
      case PLAY_BTN_PROG_UP:
        playProgram = (uint8_t)((playProgram + 1) & 0x7F);
        sendMidiProgramChange(playActiveChannel, playProgram);
        Serial.printf("Play program: %u\n", (unsigned)(playProgram + 1));
        break;
      case PLAY_BTN_BEND_DOWN:
        if (playPitchBend <= PLAY_PITCH_BEND_STEP) playPitchBend = 0;
        else playPitchBend = (uint16_t)(playPitchBend - PLAY_PITCH_BEND_STEP);
        sendMidiPitchBend(playActiveChannel, playPitchBend);
        Serial.printf("Play bend: %u\n", playPitchBend);
        break;
      case PLAY_BTN_BEND_UP:
        if (playPitchBend >= 16383 - PLAY_PITCH_BEND_STEP) playPitchBend = 16383;
        else playPitchBend = (uint16_t)(playPitchBend + PLAY_PITCH_BEND_STEP);
        sendMidiPitchBend(playActiveChannel, playPitchBend);
        Serial.printf("Play bend: %u\n", playPitchBend);
        break;
      case PLAY_BTN_SUSTAIN:
        playSustain = !playSustain;
        sendMidiControlChange(playActiveChannel, 64, playSustain ? 127 : 0);
        Serial.printf("Play sustain: %s\n", playSustain ? "ON" : "OFF");
        break;
      case PLAY_BTN_INIT:
        sendPlayModeInit(true);
        Serial.println("Play mode init");
        break;
    }

    needFullRedraw = true;
    break;
  }
}

void processPlayProgramPickerTouch(TouchPoint_t pos) {
  layoutPlayProgramPicker();

  if (touchInRect(pos, playProgramPickerButtons[PLAY_PICKER_CLOSE].x, playProgramPickerButtons[PLAY_PICKER_CLOSE].y,
                  playProgramPickerButtons[PLAY_PICKER_CLOSE].w, playProgramPickerButtons[PLAY_PICKER_CLOSE].h)) {
    playProgramPickerOpen = false;
    needFullRedraw = true;
    Serial.println("Play program picker close");
    return;
  }

  if (touchInRect(pos, playProgramPickerButtons[PLAY_PICKER_PREV].x, playProgramPickerButtons[PLAY_PICKER_PREV].y,
                  playProgramPickerButtons[PLAY_PICKER_PREV].w, playProgramPickerButtons[PLAY_PICKER_PREV].h)) {
    if (playProgramPickerPage > 0) playProgramPickerPage--;
    needFullRedraw = true;
    Serial.printf("Play program picker page: %u\n", playProgramPickerPage + 1);
    return;
  }

  if (touchInRect(pos, playProgramPickerButtons[PLAY_PICKER_NEXT].x, playProgramPickerButtons[PLAY_PICKER_NEXT].y,
                  playProgramPickerButtons[PLAY_PICKER_NEXT].w, playProgramPickerButtons[PLAY_PICKER_NEXT].h)) {
    uint8_t maxPage = 127 / PLAY_PROGRAMS_PER_PAGE;
    if (playProgramPickerPage < maxPage) playProgramPickerPage++;
    needFullRedraw = true;
    Serial.printf("Play program picker page: %u\n", playProgramPickerPage + 1);
    return;
  }

  const int x = 10;
  const int y = 44;
  const int headerH = 20;
  const int itemH = 18;
  const int itemGap = 2;
  const int startProgram = playProgramPickerPage * PLAY_PROGRAMS_PER_PAGE;

  for (int i = 0; i < PLAY_PROGRAMS_PER_PAGE; i++) {
    int program = startProgram + i;
    int itemY = y + headerH + 4 + i * (itemH + itemGap);
    if (program >= 128) continue;
    if (!touchInRect(pos, x + 4, itemY, SCREEN_WIDTH - 28, itemH)) continue;

    playProgram = (uint8_t)program;
    playProgramPickerOpen = false;
    sendMidiProgramChange(playActiveChannel, playProgram);
    needFullRedraw = true;
    Serial.printf("Play program select: %03u %s\n", (unsigned)(playProgram + 1), getUnitSynthInstrumentName(playProgram));
    return;
  }
}

void sendPlayTestPhrase() {
  const uint8_t channel = 0;

  Serial.printf("Play test phrase start: tx=ch%u program=%u %s\n",
                (unsigned)(channel + 1), (unsigned)(playProgram + 1), getUnitSynthInstrumentName(playProgram));

  sendAllNotesOff();

  // Make the test as loud and deterministic as possible.
  sendMidiControlChange(channel, 7, 127);
  sendMidiControlChange(channel, 11, 127);
  sendMidiProgramChange(channel, playProgram);
  sendMidiPitchBend(channel, 8192);

  // Opening of Debussy "Deux Arabesques" No. 1 (E major).
  // Right-hand triplet melody from m.6, then the famous descending arpeggio.
  // Each entry: { MIDI note, hold ms, gap ms after release }.
  struct PhraseNote { uint8_t note; uint16_t holdMs; uint16_t gapMs; };
  static const PhraseNote phrase[] = {
    // m.6: triplet figure rising and falling around E major
    { 76, 170, 5 }, { 78, 170, 5 }, { 80, 170, 5 },   // E5  F#5 G#5
    { 81, 170, 5 }, { 80, 170, 5 }, { 78, 170, 5 },   // A5  G#5 F#5
    { 80, 170, 5 }, { 78, 170, 5 }, { 76, 170, 5 },   // G#5 F#5 E5
    { 75, 170, 5 }, { 76, 340, 30 },                  // D#5 E5(longer)
    // m.7: characteristic descending cascade
    { 85, 130, 0 }, { 83, 130, 0 }, { 81, 130, 0 },   // C#6 B5  A5
    { 80, 130, 0 }, { 78, 130, 0 }, { 76, 130, 0 },   // G#5 F#5 E5
    { 75, 130, 0 }, { 73, 260, 20 },                  // D#5 C#5
    // soft cadence to the tonic
    { 71, 200, 0 }, { 73, 200, 0 }, { 76, 600, 0 },   // B4  C#5 E5
  };
  const size_t kPhraseLen = sizeof(phrase) / sizeof(phrase[0]);
  const uint8_t velocity = 96;

  for (size_t i = 0; i < kPhraseLen; i++) {
    const PhraseNote& n = phrase[i];
    Serial2.write(0x90 | channel);
    Serial2.write(n.note);
    Serial2.write(velocity);
    midiOutCount += 3;
    delay(n.holdMs);
    Serial2.write(0x80 | channel);
    Serial2.write(n.note);
    Serial2.write((uint8_t)0);
    midiOutCount += 3;
    if (n.gapMs) delay(n.gapMs);
  }
  Serial.println("Play test phrase done");
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
  // PLAY モードでは RX 状態は本体内の表示と重なるため右上には出さない
  if (currentMode != PLAY_MODE) {
    if (transposeValue > 0) {
      snprintf(buf, sizeof(buf), "Trans:+%d", transposeValue);
    } else if (transposeValue < 0) {
      snprintf(buf, sizeof(buf), "Trans:%d", transposeValue);
    } else {
      snprintf(buf, sizeof(buf), "Trans: 0");
    }
    M5.Lcd.setTextColor(YELLOW);
    uiDrawL(buf, 240, 2);
  }

  // I/O カウンタ (PLAY モードでは右端まで使えるので右寄せにする)
  M5.Lcd.setTextColor(DARKGREY);
  snprintf(buf, sizeof(buf), "I:%lu O:%lu", midiInCount, midiOutCount);
  if (currentMode == PLAY_MODE) {
    int w = M5.Lcd.textWidth(buf);
    uiDrawL(buf, SCREEN_WIDTH - w - 6, 2);
  } else {
    uiDrawL(buf, 165, 2);
  }
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

    if (currentMode == PLAY_MODE) {
      return;
    }

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

  if (currentMode == PLAY_MODE) {
    playPitchBend = clampInt(playPitchBend, 0, 16383);
    sendPlayModeInit(false);
  } else if (currentMode == DIRECT_MODE) {
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

  if (currentMode == PLAY_MODE) {
    DisplayMode restoreMode = isTransposeDisplayMode(lastTransposeMode) ? lastTransposeMode : DIRECT_MODE;
    enterDisplayMode(restoreMode);
    Serial.printf("Group: TRANSPOSE (%d)\n", restoreMode);
  } else if (isTransposeDisplayMode(currentMode)) {
    if (isTransposeDisplayMode(currentMode)) {
      lastTransposeMode = currentMode;
    }
    midiManagePage = MIDI_PAGE_FILTER;
    enterDisplayMode(MIDI_MANAGE_MODE);
    Serial.println("Group: MIDI_MANAGE");
  } else {
    enterDisplayMode(PLAY_MODE);
    Serial.println("Group: PLAY");
  }
}

void advanceSubMode() {
  if (currentMode == PLAY_MODE) {
    sendAllNotesOff();
    Serial.println("Play mode panic");
    return;
  }

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

  const char* modeNames[] = {"DIRECT", "KEY", "INSTANT", "SEQUENCE"};
  Serial.printf("Transpose mode: %s\n", modeNames[currentMode]);
}

void processHardwareButtons() {
  unsigned long now = millis();
  if (M5.BtnC.isPressed()) {
    if (!btnCLongPressHandled && M5.BtnC.pressedFor(MODE_LONG_PRESS_MS)) {
      handleButtonCLongAction();
      btnCLongPressHandled = true;
      lastButtonCheck = now;
    }
  } else {
    btnCLongPressHandled = false;
  }

  if (now - lastButtonCheck < BUTTON_DEBOUNCE) return;

  // 左ボタン（A）: All Notes Off切り替え
  if (M5.BtnA.wasPressed()) {
    handleButtonAAction();
    lastButtonCheck = now;
    return;
  }

  // 真ん中ボタン（B）
  if (M5.BtnB.wasPressed()) {
    handleButtonBAction();
    lastButtonCheck = now;
    return;
  }

  // 右ボタン（C）: モード切り替え（DIRECT→KEY→INSTANT→SEQUENCE→…）
  if (M5.BtnC.wasPressed() && !btnCLongPressHandled) {
    handleButtonCShortAction();
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
    const char* modeNames[] = {"DIRECT", "KEY", "INSTANT", "SEQUENCE"};
    Serial.printf("Mode: %s, Transpose: %d (maintained)\n",
                  modeNames[currentMode], transposeValue);
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
    case PLAY_MODE: return "PLAY";
    case DIRECT_MODE: return "DIRECT";
    case KEY_MODE: return "KEY";
    case INSTANT_MODE: return "INSTANT";
    case SEQUENCE_MODE: return "SEQUENCE";
    case MIDI_MANAGE_MODE: return "MIDI_MANAGER";
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
  if (currentMode == PLAY_MODE) {
    sendPlayModeInit(true);
    needFullRedraw = true;
    Serial.println("Play mode init");
    return;
  }

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
  if (currentMode == PLAY_MODE) {
    sendAllNotesOff();
    needFullRedraw = true;
    Serial.println("Play mode panic");
    return;
  }
  advanceSubMode();
}

void handleButtonCLongAction() {
  advanceDisplayMode();
}

void dispatchTouchPoint(TouchPoint_t pos) {
  if (currentMode == PLAY_MODE) {
    processPlayModeTouch(pos);
  } else
  if (currentMode == DIRECT_MODE) {
    processDirectModeTouch(pos);
  } else if (currentMode == KEY_MODE) {
    processKeyModeTouch(pos);
  } else if (currentMode == INSTANT_MODE) {
    processInstantModeTouch(pos);
  } else if (currentMode == SEQUENCE_MODE) {
    processSequenceModeTouch(pos);
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
  if (tokenEqualsIgnoreCase(groupName, "PLAY")) {
    enterDisplayMode(PLAY_MODE);
    return true;
  }
  if (tokenEqualsIgnoreCase(groupName, "TRANSPOSE")) {
    DisplayMode restoreMode = isTransposeDisplayMode(lastTransposeMode) ? lastTransposeMode : DIRECT_MODE;
    enterDisplayMode(restoreMode);
    return true;
  }
  if (tokenEqualsIgnoreCase(groupName, "MIDI") ||
      tokenEqualsIgnoreCase(groupName, "MANAGER") ||
      tokenEqualsIgnoreCase(groupName, "MIDI_MANAGER")) {
    enterDisplayMode(MIDI_MANAGE_MODE);
    midiManagePage = MIDI_PAGE_FILTER;
    needFullRedraw = true;
    return true;
  }
  return false;
}

bool setModeFromCommand(const char* modeName) {
  if (tokenEqualsIgnoreCase(modeName, "PLAY")) {
    enterDisplayMode(PLAY_MODE);
    return true;
  }
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
    midiManagePage = MIDI_PAGE_FILTER;
    needFullRedraw = true;
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
    (currentMode == PLAY_MODE) ? "PLAY" : (currentMode == MIDI_MANAGE_MODE ? "MIDI" : "TRANSPOSE"),
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
  Serial.println("MODE PLAY|DIRECT|KEY|INSTANT|SEQUENCE|FILTER|MAPPER|MIDI");
  Serial.println("GROUP PLAY|TRANSPOSE|MIDI");
  Serial.println("SET TRANSPOSE <-11..11>");
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
    Serial.println("ERR SET supports only TRANSPOSE");
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

void sendMidiControlChange(uint8_t channel, uint8_t controller, uint8_t value) {
  Serial2.write(0xB0 | (channel & 0x0F));
  Serial2.write(controller & 0x7F);
  Serial2.write(value & 0x7F);
  midiOutCount += 3;
}

void sendMidiProgramChange(uint8_t channel, uint8_t program) {
  Serial2.write(0xC0 | (channel & 0x0F));
  Serial2.write(program & 0x7F);
  midiOutCount += 2;
}

void sendMidiPitchBend(uint8_t channel, uint16_t bendValue) {
  if (bendValue > 16383) bendValue = 16383;
  Serial2.write(0xE0 | (channel & 0x0F));
  Serial2.write((uint8_t)(bendValue & 0x7F));
  Serial2.write((uint8_t)((bendValue >> 7) & 0x7F));
  midiOutCount += 3;
}

void sendPlayModeInit(bool resetProgramAndVolume) {
  const uint8_t channel = playActiveChannel & 0x0F;

  playProgramPickerOpen = false;

  // GS Reset
  const uint8_t gsReset[] = { 0xF0, 0x41, 0x10, 0x42, 0x12, 0x40, 0x00, 0x7F, 0x00, 0x41, 0xF7 };
  for (uint8_t b : gsReset) {
    Serial2.write(b);
    midiOutCount++;
  }

  // Reset controllers for a predictable performance state.
  sendMidiControlChange(channel, 121, 0);
  sendMidiControlChange(channel, 123, 0);
  sendMidiControlChange(channel, 120, 0);

  if (resetProgramAndVolume) {
    playVolume = 100;
    playProgram = 0;
    playPitchBend = 8192;
    playSustain = false;
  }

  sendMidiControlChange(channel, 7, playVolume);
  sendMidiControlChange(channel, 11, 127);
  sendMidiProgramChange(channel, playProgram);
  sendMidiPitchBend(channel, playPitchBend);
  sendMidiControlChange(channel, 64, playSustain ? 127 : 0);
}

void processMIDI() {
  bool sawInput = false;
  while (Serial2.available()) {
    uint8_t incomingByte = Serial2.read();
    sawInput = true;
    midiInCount++;
    processMIDIByte(incomingByte);
  }
  if (sawInput) {
    g_lastMidiInputAt = millis();
  }
}

void recordMidiRxDebug(const uint8_t* bytes, uint8_t length, bool hasChannel, int8_t channel) {
  const uint8_t status = (length > 0) ? bytes[0] : 0;
  const char* kindLabel = "Msg";
  switch (status) {
    case 0x80: kindLabel = "NoteOff"; break;
    case 0x90: kindLabel = "NoteOn"; break;
    case 0xA0: kindLabel = "KeyPrs"; break;
    case 0xB0: kindLabel = "CtrlChg"; break;
    case 0xC0: kindLabel = "PrgChg"; break;
    case 0xD0: kindLabel = "ChPrs"; break;
    case 0xE0: kindLabel = "Bend"; break;
    case 0xF0: kindLabel = "SysEx"; break;
    case 0xF1: kindLabel = "MTC"; break;
    case 0xF2: kindLabel = "SongPos"; break;
    case 0xF3: kindLabel = "SongSel"; break;
    case 0xF6: kindLabel = "TuneReq"; break;
    case 0xF8: kindLabel = "Clock"; break;
    case 0xFA: kindLabel = "Start"; break;
    case 0xFB: kindLabel = "Cont"; break;
    case 0xFC: kindLabel = "Stop"; break;
    case 0xFE: kindLabel = "ActSn"; break;
    case 0xFF: kindLabel = "Reset"; break;
    default: break;
  }
  if (hasChannel && channel >= 0) {
    snprintf(g_lastMidiRxLabel, sizeof(g_lastMidiRxLabel), "RX:%s C%02d", kindLabel, (int)channel + 1);
  } else {
    snprintf(g_lastMidiRxLabel, sizeof(g_lastMidiRxLabel), "RX:%s", kindLabel);
  }
  g_lastMidiInputAt = millis();
}

void handleParsedMidiMessage(const MidiMessage& inMsg) {
  recordMidiRxDebug(inMsg.bytes, inMsg.length, inMsg.hasChannel, inMsg.channel);

  if (currentMode == PLAY_MODE) {
    if (inMsg.hasChannel && inMsg.channel >= 0) {
      playActiveChannel = (uint8_t)inMsg.channel;
    }
    Serial.printf("[MIDI RX PLAY] %s\n", g_lastMidiRxLabel);
    uint8_t raw[3] = { 0, 0, 0 };
    for (int i = 0; i < inMsg.length; i++) raw[i] = inMsg.bytes[i];
    sendMIDIMessage(raw, inMsg.length);
    return;
  }

  if (!shouldAllowMidiMessage(inMsg)) return;
  MidiMessage mappedMsg = applyMidiMapper(inMsg);
  if (mappedMsg.hasChannel && mappedMsg.channel >= 0) {
    playActiveChannel = (uint8_t)mappedMsg.channel;
  }
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
    if (allowCurrentSysEx) {
      Serial2.write(midiData);
      midiOutCount++;
    }
    if (midiData == 0xF7) {
      inSysEx = false;
    }
    return;
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
  if (currentMode == PLAY_MODE) {
    for (int i = 0; i < length; i++) {
      Serial2.write(buffer[i]);
    }
    midiOutCount += length;
    return;
  }

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
