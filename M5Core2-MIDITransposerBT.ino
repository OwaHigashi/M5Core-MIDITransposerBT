#include <M5Core2.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "src/hid_l2cap.h"

// CORE2でのピンアサイン（M5 MIDI Module2）
#define RXD2 13
#define TXD2 14

// Bluetooth HID settings for SPT-10 foot pedal
#define TARGET_BT_ADDR  { 0xA1, 0x12, 0x12, 0x92, 0x0E, 0x23 } // SPT-10 (Bluetooth Music Pedal) MAC address
#define BT_CONNECT_TIMEOUT  10000

static const uint8_t PEDAL_LEFT_KEY  = 0x52; // HID keyboard Up Arrow
static const uint8_t PEDAL_RIGHT_KEY = 0x51; // HID keyboard Down Arrow

// 画面サイズ
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// モード定義
enum DisplayMode {
  DIRECT_MODE,
  KEY_MODE,
  RELATIVE_MODE,
  PRESET_MODE
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
TouchButton relativeButtons[8];
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

// Preset Mode用の設定
#define PRESET_SLOT_COUNT 6
int8_t presetTransposeValues[PRESET_SLOT_COUNT] = {0, 0, 0, 0, 0, 0}; // 初期値
int presetCursorPos = 0; // 初期カーソル位置（0から5の範囲）

struct PresetSlot {
  int slotX, slotY, slotW, slotH;  // スロット表示領域
  int upBtnX, upBtnY, upBtnW, upBtnH;  // 上ボタン
  int downBtnX, downBtnY, downBtnW, downBtnH;  // 下ボタン
};

PresetSlot presetSlots[PRESET_SLOT_COUNT];

// プリセットモード用の左右移動ボタン
struct PresetNavButton {
  int x, y, w, h;
};

PresetNavButton presetLeftBtn;
PresetNavButton presetRightBtn;

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

// アクティブなノート追跡（88鍵盤対応）
// 標準的な88鍵盤: A0(21) から C8(108)
#define PIANO_LOWEST_NOTE 21   // A0
#define PIANO_HIGHEST_NOTE 108 // C8
#define PIANO_KEY_COUNT 88

struct NoteState {
  bool isActive;
  int8_t originalTranspose;  // このノートが押された時の転調値
  uint8_t channel;
  uint8_t velocity;
};

// 現在押されている鍵盤の状態（88鍵盤分）
NoteState currentNoteStates[PIANO_KEY_COUNT];

// 転調変更時の一時保存用
NoteState savedNoteStates[PIANO_KEY_COUNT];
int8_t savedTranspose = 0;

// ユーティリティ：-12〜+12にクランプ
int8_t clampTranspose(int8_t v) {
  if (v < -12) return -12;
  if (v > 12) return 12;
  return v;
}

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

void setup() {
  M5.begin(true, true, true, true);
  
  Serial.begin(115200);
  Serial2.begin(31250, SERIAL_8N1, RXD2, TXD2);
  Serial2.setRxBufferSize(256);
  Serial2.setTxBufferSize(256);
  
  // ノート状態の初期化
  for (int i = 0; i < PIANO_KEY_COUNT; i++) {
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
  initRelativeModeButtons();
  initPresetModeButtons();
  
  // 最初の転調値0のボタンを確実に光らせる（-5から+6レンジでは5番目のボタン）
  transposeButtons[5] = true;  // RANGE_MINUS5_TO_6の場合、ボタン5が転調値0
  
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
  drawInterface();

  Serial.println("MIDI Transposer Ready!");
  Serial.printf("Initial transpose: %d, Button 5 state: %s\n", transposeValue, transposeButtons[5] ? "ON" : "OFF");
}

void loop() {
  processMIDI();

  static unsigned long lastUICheck = 0;
  unsigned long now = millis();

  if (now - lastUICheck >= 20) {
    M5.update();
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
  int buttonHeight = 52;
  int startX = 10;
  int startY = 60;  // ステータス領域を広げたため下げる
  int spacingX = 5;
  int spacingY = 8;
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
  int whiteKeyHeight = 85;
  int blackKeyHeight = 60;
  int startX = 10;
  
  // メジャーキー（上段）
  int majorY = 65;  // ステータス領域を広げたため下げる
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
  
  // マイナーキー（下段）
  int minorY = 160;  // ステータス領域を広げたため下げる
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

// 相対モードのボタン配置（2行×4列）- ボタンサイズを調整
void initRelativeModeButtons() {
  static const int8_t deltas[8] = {-5, -3, -2, -1, +1, +2, +3, +5};
  static char labels[8][4];

  int buttonWidth = 75;
  int buttonHeight = 52;
  int startX = 10;
  int startY = 130;
  int spacingX = 5;
  int spacingY = 5;
  int buttonsPerRow = 4;

  for (int i = 0; i < 8; i++) {
    int row = i / buttonsPerRow;
    int col = i % buttonsPerRow;

    relativeButtons[i].x = startX + col * (buttonWidth + spacingX);
    relativeButtons[i].y = startY + row * (buttonHeight + spacingY);
    relativeButtons[i].w = buttonWidth;
    relativeButtons[i].h = buttonHeight;
    relativeButtons[i].value = deltas[i];

    if (deltas[i] > 0) sprintf(labels[i], "+%d", deltas[i]);
    else sprintf(labels[i], "%d", deltas[i]);
    relativeButtons[i].label = labels[i];
  }
}

// プリセットモードのボタン配置（6つのスロット）
void initPresetModeButtons() {
  int slotWidth = 48;
  int slotSpacing = 5;
  int startX = 5;

  int buttonH = 45;     // 上下ボタンとスロットをすべて同じ高さに
  int upBtnY = 55;
  int slotY = 105;      // 上ボタンの下に配置
  int downBtnY = 155;   // スロットの下に配置

  for (int i = 0; i < PRESET_SLOT_COUNT; i++) {
    int x = startX + i * (slotWidth + slotSpacing);

    presetSlots[i].slotX = x;
    presetSlots[i].slotY = slotY;
    presetSlots[i].slotW = slotWidth;
    presetSlots[i].slotH = buttonH;

    presetSlots[i].upBtnX = x;
    presetSlots[i].upBtnY = upBtnY;
    presetSlots[i].upBtnW = slotWidth;
    presetSlots[i].upBtnH = buttonH;

    presetSlots[i].downBtnX = x;
    presetSlots[i].downBtnY = downBtnY;
    presetSlots[i].downBtnW = slotWidth;
    presetSlots[i].downBtnH = buttonH;
  }

  // 左右移動ボタンの配置
  int navBtnY = 205;
  int navBtnW = 80;
  int navBtnH = 35;
  presetLeftBtn.x = 10;
  presetLeftBtn.y = navBtnY;
  presetLeftBtn.w = navBtnW;
  presetLeftBtn.h = navBtnH;

  presetRightBtn.x = 230;
  presetRightBtn.y = navBtnY;
  presetRightBtn.w = navBtnW;
  presetRightBtn.h = navBtnH;
}

void drawInterface() {
  M5.Lcd.fillScreen(BLACK);
  
  // タイトル表示（左上）
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.print("MIDI Transposer");
  
  // ハードウェアボタンのガイド表示（1行目）
  M5.Lcd.setTextColor(DARKGREY);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 25);
  M5.Lcd.print("A:AllOff  B:Range  C:Mode");
  
  // All Notes Off状態表示（2行目左）
  M5.Lcd.setTextColor(allNotesOffEnabled ? GREEN : RED);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 40);
  M5.Lcd.printf("AllOff: %s", allNotesOffEnabled ? "ON" : "OFF");
  
  if (currentMode == DIRECT_MODE) {
    drawDirectMode();
  } else if (currentMode == KEY_MODE) {
    drawKeyMode();
  } else if (currentMode == RELATIVE_MODE) {
    drawRelativeMode();
  } else {
    drawPresetMode();
  }
  
  updateStatusArea();
}

void drawDirectMode() {
  for (int i = 0; i < 12; i++) {
    uint16_t color = transposeButtons[i] ? GREEN : DARKGREY;
    uint16_t textColor = transposeButtons[i] ? BLACK : WHITE;
    
    M5.Lcd.fillRect(directButtons[i].x, directButtons[i].y, 
                    directButtons[i].w, directButtons[i].h, color);
    M5.Lcd.drawRect(directButtons[i].x, directButtons[i].y, 
                    directButtons[i].w, directButtons[i].h, WHITE);
    
    M5.Lcd.setTextColor(textColor);
    M5.Lcd.setTextSize(3);
    
    int textX = directButtons[i].x + (directButtons[i].w - strlen(directButtons[i].label) * 18) / 2;
    int textY = directButtons[i].y + (directButtons[i].h - 24) / 2;
    M5.Lcd.setCursor(textX, textY);
    M5.Lcd.print(directButtons[i].label);
  }
}

void drawKeyMode() {
  M5.Lcd.setTextColor(CYAN);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 55);
  if (majorUpperTranspose) {
    M5.Lcd.print("Major Keys (Upper +):");
  } else {
    M5.Lcd.print("Major Keys (to C):");
  }
  
  M5.Lcd.setCursor(10, 150);
  if (minorUpperTranspose) {
    M5.Lcd.print("Minor Keys (Lower -):");
  } else {
    M5.Lcd.print("Minor Keys (to Am):");
  }
  
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
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(majorKeys[i].x + 5, majorKeys[i].y + majorKeys[i].h - 15);
      M5.Lcd.print(majorKeys[i].keyName);
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
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(majorKeys[i].x + 3, majorKeys[i].y + majorKeys[i].h - 15);
      M5.Lcd.print(majorKeys[i].keyName);
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
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(minorKeys[i].x + 5, minorKeys[i].y + minorKeys[i].h - 15);
      M5.Lcd.print(minorKeys[i].keyName);
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
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(minorKeys[i].x + 3, minorKeys[i].y + minorKeys[i].h - 15);
      M5.Lcd.print(minorKeys[i].keyName);
    }
  }
}

// 相対モード描画
void drawRelativeMode() {
  // 説明
  M5.Lcd.setTextColor(CYAN);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 55);
  M5.Lcd.print("Relative Mode: tap to add/subtract");

  // 現在の転調値を大きく中央表示
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.setTextSize(6); // 大きめ
  char buf[8];
  if (transposeValue > 0) sprintf(buf, "+%d", transposeValue);
  else sprintf(buf, "%d", transposeValue);

  // 文字幅・高さ（標準6x8フォント × TextSize）
  const int charW = 6 * 6;  // = 36px
  const int charH = 8 * 6;  // = 48px
  int tw = strlen(buf) * charW;

  // Xは左右中央
  int tx = (SCREEN_WIDTH - tw) / 2;
  if (tx < 0) tx = 0;

  // Yは「ボタン最上段の上半分」の中央に置く
  int topAreaBottom = relativeButtons[0].y;           // ボタン上端
  int ty = (topAreaBottom - charH) / 2;               // 上半分の中央
  if (ty < 65) ty = 65;                               // 上部ラベルと最低余白を確保

  M5.Lcd.setCursor(tx, ty);
  M5.Lcd.print(buf);

  // ボタン群
  for (int i = 0; i < 8; i++) {
    uint16_t color = DARKGREY;
    uint16_t border = WHITE;
    uint16_t txt = WHITE;
    M5.Lcd.fillRect(relativeButtons[i].x, relativeButtons[i].y, relativeButtons[i].w, relativeButtons[i].h, color);
    M5.Lcd.drawRect(relativeButtons[i].x, relativeButtons[i].y, relativeButtons[i].w, relativeButtons[i].h, border);
    M5.Lcd.setTextColor(txt);
    M5.Lcd.setTextSize(3);
    int textX = relativeButtons[i].x + (relativeButtons[i].w - (int)strlen(relativeButtons[i].label) * (6*3)) / 2; // 6*3=18
    int textY = relativeButtons[i].y + (relativeButtons[i].h - (8*3)) / 2;                                        // 8*3=24
    M5.Lcd.setCursor(textX, textY);
    M5.Lcd.print(relativeButtons[i].label);
  }
}

// プリセットモード描画
void drawPresetMode() {
  // 6つのスロットを描画
  for (int i = 0; i < PRESET_SLOT_COUNT; i++) {
    bool isCurrent = (i == presetCursorPos);

    // 上ボタン
    uint16_t upBtnColor = DARKGREY;
    M5.Lcd.fillRect(presetSlots[i].upBtnX, presetSlots[i].upBtnY,
                    presetSlots[i].upBtnW, presetSlots[i].upBtnH, upBtnColor);
    M5.Lcd.drawRect(presetSlots[i].upBtnX, presetSlots[i].upBtnY,
                    presetSlots[i].upBtnW, presetSlots[i].upBtnH, WHITE);

    // 上ボタンに三角形を描画（▲）
    int centerX = presetSlots[i].upBtnX + presetSlots[i].upBtnW / 2;
    int centerY = presetSlots[i].upBtnY + presetSlots[i].upBtnH / 2;
    int triSize = 12;
    M5.Lcd.fillTriangle(
      centerX, centerY - triSize,           // 上の頂点
      centerX - triSize, centerY + triSize, // 左下
      centerX + triSize, centerY + triSize, // 右下
      WHITE
    );

    // スロット（転調値表示）
    uint16_t slotColor = isCurrent ? GREEN : DARKGREY;
    uint16_t slotTextColor = isCurrent ? BLACK : WHITE;
    M5.Lcd.fillRect(presetSlots[i].slotX, presetSlots[i].slotY,
                    presetSlots[i].slotW, presetSlots[i].slotH, slotColor);
    M5.Lcd.drawRect(presetSlots[i].slotX, presetSlots[i].slotY,
                    presetSlots[i].slotW, presetSlots[i].slotH, WHITE);

    // 転調値を表示
    char valueStr[5];
    int8_t value = presetTransposeValues[i];
    if (value > 0) sprintf(valueStr, "+%d", value);
    else sprintf(valueStr, "%d", value);

    M5.Lcd.setTextColor(slotTextColor);
    M5.Lcd.setTextSize(3);
    int valueX = presetSlots[i].slotX + (presetSlots[i].slotW - strlen(valueStr) * 18) / 2;
    int valueY = presetSlots[i].slotY + (presetSlots[i].slotH - 24) / 2;
    M5.Lcd.setCursor(valueX, valueY);
    M5.Lcd.print(valueStr);

    // 下ボタン
    uint16_t downBtnColor = DARKGREY;
    M5.Lcd.fillRect(presetSlots[i].downBtnX, presetSlots[i].downBtnY,
                    presetSlots[i].downBtnW, presetSlots[i].downBtnH, downBtnColor);
    M5.Lcd.drawRect(presetSlots[i].downBtnX, presetSlots[i].downBtnY,
                    presetSlots[i].downBtnW, presetSlots[i].downBtnH, WHITE);

    // 下ボタンに三角形を描画（▼）
    centerX = presetSlots[i].downBtnX + presetSlots[i].downBtnW / 2;
    centerY = presetSlots[i].downBtnY + presetSlots[i].downBtnH / 2;
    M5.Lcd.fillTriangle(
      centerX, centerY + triSize,           // 下の頂点
      centerX - triSize, centerY - triSize, // 左上
      centerX + triSize, centerY - triSize, // 右上
      WHITE
    );
  }

  // 左右移動ボタンを描画
  // 左ボタン
  M5.Lcd.fillRect(presetLeftBtn.x, presetLeftBtn.y, presetLeftBtn.w, presetLeftBtn.h, BLUE);
  M5.Lcd.drawRect(presetLeftBtn.x, presetLeftBtn.y, presetLeftBtn.w, presetLeftBtn.h, WHITE);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(presetLeftBtn.x + (presetLeftBtn.w - 18) / 2, presetLeftBtn.y + (presetLeftBtn.h - 24) / 2);
  M5.Lcd.print("<");

  // 右ボタン
  M5.Lcd.fillRect(presetRightBtn.x, presetRightBtn.y, presetRightBtn.w, presetRightBtn.h, BLUE);
  M5.Lcd.drawRect(presetRightBtn.x, presetRightBtn.y, presetRightBtn.w, presetRightBtn.h, WHITE);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(presetRightBtn.x + (presetRightBtn.w - 18) / 2, presetRightBtn.y + (presetRightBtn.h - 24) / 2);
  M5.Lcd.print(">");
}

void updateStatusArea() {
  // 右上のステータス表示エリア（x座標を右にずらす）
  M5.Lcd.fillRect(200, 10, 115, 30, BLACK);  // x座標を155→200に変更、幅を165→115に調整

  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(205, 15);  // x座標を155→205に変更

  if (transposeValue > 0) {
    M5.Lcd.printf("Transpose: +%d", transposeValue);
  } else if (transposeValue < 0) {
    M5.Lcd.printf("Transpose: %d", transposeValue);
  } else {
    M5.Lcd.print("Transpose: 0");
  }

  // MIDI統計情報（2行目右）
  M5.Lcd.setTextColor(DARKGREY);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(205, 28);  // x座標を155→205に変更
  M5.Lcd.printf("I:%lu O:%lu", midiInCount, midiOutCount);
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

    if (currentMode == PRESET_MODE) {
      // PRESET_MODE: ペダルでカーソルを左右に移動
      if (leftJustPressed) {
        // 左ペダル: カーソルを左に移動
        presetCursorPos = (presetCursorPos > 0) ? presetCursorPos - 1 : PRESET_SLOT_COUNT - 1;
        handleTransposeChange(presetTransposeValues[presetCursorPos]);
        needFullRedraw = true;
        Serial.printf("Preset Mode (Pedal): Cursor moved to slot %d\n", presetCursorPos);
      } else if (rightJustPressed) {
        // 右ペダル: カーソルを右に移動
        presetCursorPos = (presetCursorPos < PRESET_SLOT_COUNT - 1) ? presetCursorPos + 1 : 0;
        handleTransposeChange(presetTransposeValues[presetCursorPos]);
        needFullRedraw = true;
        Serial.printf("Preset Mode (Pedal): Cursor moved to slot %d\n", presetCursorPos);
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
      } else { // RELATIVE_MODE
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
        // RelativeModeの場合、画面を更新
        needFullRedraw = true;
      }
    }
  }
}

void processHardwareButtons() {
  unsigned long now = millis();
  if (now - lastButtonCheck < BUTTON_DEBOUNCE) return;

  // 左ボタン（A）: All Notes Off切り替え
  if (M5.BtnA.wasPressed()) {
    allNotesOffEnabled = !allNotesOffEnabled;
    needFullRedraw = true;
    lastButtonCheck = now;
    Serial.printf("All Notes Off: %s\n", allNotesOffEnabled ? "ON" : "OFF");
  }

  // 真ん中ボタン（B）
  if (M5.BtnB.wasPressed()) {
    if (currentMode == DIRECT_MODE) {
      // DirectMode: 3つのレンジを順次切り替え
      if (transposeRange == RANGE_0_TO_12) {
        transposeRange = RANGE_MINUS12_TO_0;
      } else if (transposeRange == RANGE_MINUS12_TO_0) {
        transposeRange = RANGE_MINUS5_TO_6;
      } else {
        transposeRange = RANGE_0_TO_12;
      }
      updateDirectButtonLabels();
      setCurrentTransposeButton();  // 現在の転調値に対応するボタンを光らせる
      needFullRedraw = true;
    } else if (currentMode == KEY_MODE) {
      // KeyMode: 上位/下位転調切り替え
      majorUpperTranspose = !majorUpperTranspose;
      minorUpperTranspose = !minorUpperTranspose;
      selectedMajorKey = -1;
      selectedMinorKey = -1;
      needFullRedraw = true;
    } else {
      // RELATIVE_MODE, PRESET_MODE: 何もしない
    }
    lastButtonCheck = now;
    Serial.println("Range/Mode toggled (B)");
  }

  // 右ボタン（C）: モード切り替え（DIRECT→KEY→RELATIVE→PRESET→…）
  if (M5.BtnC.wasPressed()) {
    sendAllNotesOff();
    delay(10);

    if (currentMode == DIRECT_MODE) currentMode = KEY_MODE;
    else if (currentMode == KEY_MODE) currentMode = RELATIVE_MODE;
    else if (currentMode == RELATIVE_MODE) currentMode = PRESET_MODE;
    else currentMode = DIRECT_MODE;

    if (currentMode == DIRECT_MODE) {
      setCurrentTransposeButton();  // 現在の転調値に対応するボタンを光らせる
      Serial.println("DirectMode: Current transpose button selected");
    } else if (currentMode == KEY_MODE) {
      selectedMajorKey = -1;
      selectedMinorKey = -1;
      Serial.println("KeyMode: All keys deselected");
    } else if (currentMode == RELATIVE_MODE) {
      // 相対モード：特に選択状態は持たない
      Serial.println("RelativeMode entered");
    } else {
      // プリセットモード：カーソル位置の転調値を適用
      handleTransposeChange(presetTransposeValues[presetCursorPos]);
      Serial.println("PresetMode entered");
    }

    needFullRedraw = true;
    lastButtonCheck = now;
    Serial.printf("Mode: %s, Transpose: %d (maintained)\n",
                  currentMode == DIRECT_MODE ? "DIRECT" :
                  (currentMode == KEY_MODE ? "KEY" :
                  (currentMode == RELATIVE_MODE ? "RELATIVE" : "PRESET")),
                  transposeValue);
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

// 転調値変更時の処理（スムーズな転調）
void handleTransposeChange(int8_t newTransposeValue) {
  newTransposeValue = clampTranspose(newTransposeValue);
  if (newTransposeValue == transposeValue) return; // 変更なし
  
  if (allNotesOffEnabled) {
    sendAllNotesOff();
    delay(10);
    transposeValue = newTransposeValue;
    needPartialUpdate = true;
    return;
  }
  
  savedTranspose = transposeValue;
  for (int i = 0; i < PIANO_KEY_COUNT; i++) {
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
  if (currentMode == DIRECT_MODE) {
    processDirectModeTouch(pos);
  } else if (currentMode == KEY_MODE) {
    processKeyModeTouch(pos);
  } else if (currentMode == RELATIVE_MODE) {
    processRelativeModeTouch(pos);
  } else {
    processPresetModeTouch(pos);
  }
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

// 相対モードのタッチ処理
void processRelativeModeTouch(TouchPoint_t pos) {
  for (int i = 0; i < 8; i++) {
    if (pos.x >= relativeButtons[i].x && pos.x <= relativeButtons[i].x + relativeButtons[i].w &&
        pos.y >= relativeButtons[i].y && pos.y <= relativeButtons[i].y + relativeButtons[i].h) {
      int8_t delta = relativeButtons[i].value;
      handleTransposeChange(transposeValue + delta); // クランプは内側で実施
      needFullRedraw = true;
      return;
    }
  }
}

// プリセットモードのタッチ処理
void processPresetModeTouch(TouchPoint_t pos) {
  // 左ボタンのタッチ検出
  if (pos.x >= presetLeftBtn.x && pos.x <= presetLeftBtn.x + presetLeftBtn.w &&
      pos.y >= presetLeftBtn.y && pos.y <= presetLeftBtn.y + presetLeftBtn.h) {
    // カーソルを左に移動
    presetCursorPos = (presetCursorPos > 0) ? presetCursorPos - 1 : PRESET_SLOT_COUNT - 1;
    handleTransposeChange(presetTransposeValues[presetCursorPos]);
    needFullRedraw = true;
    return;
  }

  // 右ボタンのタッチ検出
  if (pos.x >= presetRightBtn.x && pos.x <= presetRightBtn.x + presetRightBtn.w &&
      pos.y >= presetRightBtn.y && pos.y <= presetRightBtn.y + presetRightBtn.h) {
    // カーソルを右に移動
    presetCursorPos = (presetCursorPos < PRESET_SLOT_COUNT - 1) ? presetCursorPos + 1 : 0;
    handleTransposeChange(presetTransposeValues[presetCursorPos]);
    needFullRedraw = true;
    return;
  }

  for (int i = 0; i < PRESET_SLOT_COUNT; i++) {
    // 上ボタンのタッチ検出
    if (pos.x >= presetSlots[i].upBtnX && pos.x <= presetSlots[i].upBtnX + presetSlots[i].upBtnW &&
        pos.y >= presetSlots[i].upBtnY && pos.y <= presetSlots[i].upBtnY + presetSlots[i].upBtnH) {
      // 転調値を増やす
      presetTransposeValues[i] = clampTranspose(presetTransposeValues[i] + 1);
      // カーソルをこのスロットに移動
      presetCursorPos = i;
      // 転調値を適用
      handleTransposeChange(presetTransposeValues[i]);
      needFullRedraw = true;
      return;
    }

    // 下ボタンのタッチ検出
    if (pos.x >= presetSlots[i].downBtnX && pos.x <= presetSlots[i].downBtnX + presetSlots[i].downBtnW &&
        pos.y >= presetSlots[i].downBtnY && pos.y <= presetSlots[i].downBtnY + presetSlots[i].downBtnH) {
      // 転調値を減らす
      presetTransposeValues[i] = clampTranspose(presetTransposeValues[i] - 1);
      // カーソルをこのスロットに移動
      presetCursorPos = i;
      // 転調値を適用
      handleTransposeChange(presetTransposeValues[i]);
      needFullRedraw = true;
      return;
    }

    // スロット自体のタッチ検出（カーソル移動と転調値適用）
    if (pos.x >= presetSlots[i].slotX && pos.x <= presetSlots[i].slotX + presetSlots[i].slotW &&
        pos.y >= presetSlots[i].slotY && pos.y <= presetSlots[i].slotY + presetSlots[i].slotH) {
      // カーソルをこのスロットに移動
      presetCursorPos = i;
      // 転調値を適用
      handleTransposeChange(presetTransposeValues[i]);
      needFullRedraw = true;
      return;
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
  
  for (int i = 0; i < PIANO_KEY_COUNT; i++) {
    currentNoteStates[i].isActive = false;
    savedNoteStates[i].isActive = false;
  }
  
  Serial.println("All Notes Off sent");
}

void processMIDI() {
  while (Serial2.available()) {
    uint8_t incomingByte = Serial2.read();
    midiInCount++;
    processMIDIByte(incomingByte);
  }
}

void processMIDIByte(uint8_t midiData) {
  static uint8_t midiBuffer[3];
  static int bufferIndex = 0;
  static uint8_t runningStatus = 0;
  static bool inSysEx = false;
  
  if (midiData >= 0xF8) {
    Serial2.write(midiData);
    midiOutCount++;
    return;
  }
  
  if (inSysEx) {
    Serial2.write(midiData);
    midiOutCount++;
    if (midiData == 0xF7) {
      inSysEx = false;
    }
    return;
  }
  
  if (midiData == 0xF0) {
    inSysEx = true;
    Serial2.write(midiData);
    midiOutCount++;
    return;
  }
  
  if (midiData & 0x80) {
    runningStatus = midiData;
    midiBuffer[0] = midiData;
    bufferIndex = 1;
    
    if (midiData >= 0xF0 && midiData < 0xF8) {
      Serial2.write(midiData);
      midiOutCount++;
      return;
    }
  } else if (runningStatus != 0) {
    midiBuffer[bufferIndex++] = midiData;
  } else {
    return;
  }
  
  int messageLength = getMIDIMessageLength(runningStatus);
  if (bufferIndex >= messageLength) {
    sendMIDIMessage(midiBuffer, messageLength);
    bufferIndex = 1;
  }
}

int getMIDIMessageLength(uint8_t status) {
  uint8_t messageType = status & 0xF0;
  
  switch (messageType) {
    case 0x80:
    case 0x90:
    case 0xA0:
    case 0xB0:
    case 0xE0:
      return 3;
    case 0xC0:
    case 0xD0:
      return 2;
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
    int pianoKeyIndex = getPianoKeyIndex(note);
    
    if (isNoteOn) {
      int16_t transposedNote = note + transposeValue;
      if (transposedNote >= 0 && transposedNote <= 127) {
        Serial2.write(status);
        Serial2.write((uint8_t)transposedNote);
        Serial2.write(velocity);
        midiOutCount += 3;
        
        if (pianoKeyIndex >= 0) {
          currentNoteStates[pianoKeyIndex].isActive = true;
          currentNoteStates[pianoKeyIndex].originalTranspose = transposeValue;
          currentNoteStates[pianoKeyIndex].channel = channel;
          currentNoteStates[pianoKeyIndex].velocity = velocity;
        }
      }
    } else {
      int16_t transposedNote;
      bool shouldSendNoteOff = true;
      
      if (pianoKeyIndex >= 0) {
        if (currentNoteStates[pianoKeyIndex].isActive) {
          transposedNote = note + currentNoteStates[pianoKeyIndex].originalTranspose;
          currentNoteStates[pianoKeyIndex].isActive = false;
        } else if (savedNoteStates[pianoKeyIndex].isActive) {
          transposedNote = note + savedNoteStates[pianoKeyIndex].originalTranspose;
          savedNoteStates[pianoKeyIndex].isActive = false;
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