#include <M5Core2.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "src/hid_l2cap.h"

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

// モード定義
enum DisplayMode {
  DIRECT_MODE,
  KEY_MODE,
  RELATIVE_MODE,
  SEQUENCE_MODE
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

// 汎用ナビゲーションボタン構造体
struct NavButton {
  int x, y, w, h;
};

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
  int w = strlen(msg) * 12 + 20;
  int h = 30;
  int x = (SCREEN_WIDTH - w) / 2;
  int y = (SCREEN_HEIGHT - h) / 2;
  M5.Lcd.fillRect(x, y, w, h, color);
  M5.Lcd.drawRect(x, y, w, h, WHITE);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(x + 10, y + 7);
  M5.Lcd.print(msg);
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
  initSequenceModeButtons();

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
        Serial.println("[BT_BOND] 3s stable - saving bond to SD...");
        if (saveBTBondToSD()) {
          showBTOverlay("Bond Saved!", 0x03E0, 1500);
        } else {
          showBTOverlay("Bond Save Err", RED, 2000);
        }
      }
    }

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

// シーケンスモードのボタン配置
void initSequenceModeButtons() {
  memset(seqPatterns, 0, sizeof(seqPatterns));

  int slotWidth = 48;
  int slotSpacing = 5;
  int startX = 5;

  // レイアウト: パターン選択(h=34) → 上ボタン(h=30) → 値(h=32) → 下ボタン(h=30) → ステップ移動(h=30)
  int patRowY = 52;   int patRowH = 34;
  int upBtnY  = 90;   int upBtnH  = 30;
  int slotY   = 122;  int slotH   = 32;
  int downBtnY = 156;  int downBtnH = 30;
  int navY    = 192;  int navH    = 32;

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

  // BT接続状態表示（2行目右）
  {
    BT_STATUS btSt = hid_l2cap_is_connected();
    uint16_t btColor;
    const char* btLabel;
    if (btSt == BT_CONNECTED)    { btColor = GREEN;    btLabel = "BT:ON"; }
    else if (btSt == BT_CONNECTING) { btColor = YELLOW; btLabel = "BT:.."; }
    else                            { btColor = RED;     btLabel = "BT:--"; }
    M5.Lcd.setTextColor(btColor);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(100, 40);
    M5.Lcd.print(btLabel);
  }
  
  if (currentMode == DIRECT_MODE) {
    drawDirectMode();
  } else if (currentMode == KEY_MODE) {
    drawKeyMode();
  } else if (currentMode == RELATIVE_MODE) {
    drawRelativeMode();
  } else {
    drawSequenceMode();
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

// シーケンスモード描画
void drawSequenceMode() {
  // パターン選択行（大きめボタン）
  // 左矢印ボタン
  M5.Lcd.fillRect(seqPatLeftBtn.x, seqPatLeftBtn.y,
                  seqPatLeftBtn.w, seqPatLeftBtn.h, BLUE);
  M5.Lcd.drawRect(seqPatLeftBtn.x, seqPatLeftBtn.y,
                  seqPatLeftBtn.w, seqPatLeftBtn.h, WHITE);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(seqPatLeftBtn.x + (seqPatLeftBtn.w - 18) / 2,
                   seqPatLeftBtn.y + (seqPatLeftBtn.h - 24) / 2);
  M5.Lcd.print("<");

  // パターン番号表示
  int patDispX = seqPatLeftBtn.x + seqPatLeftBtn.w + 3;
  int patDispW = seqPatRightBtn.x - patDispX - 3;
  M5.Lcd.fillRect(patDispX, seqPatLeftBtn.y, patDispW, seqPatLeftBtn.h, NAVY);
  M5.Lcd.drawRect(patDispX, seqPatLeftBtn.y, patDispW, seqPatLeftBtn.h, WHITE);
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.setTextSize(2);
  char patStr[20];
  snprintf(patStr, sizeof(patStr), "Pat %02d/%02d", seqCurrentPattern + 1, SEQ_PATTERN_COUNT);
  int patTextW = strlen(patStr) * 12;
  M5.Lcd.setCursor(patDispX + (patDispW - patTextW) / 2,
                   seqPatLeftBtn.y + (seqPatLeftBtn.h - 16) / 2);
  M5.Lcd.print(patStr);

  // 右矢印ボタン
  M5.Lcd.fillRect(seqPatRightBtn.x, seqPatRightBtn.y,
                  seqPatRightBtn.w, seqPatRightBtn.h, BLUE);
  M5.Lcd.drawRect(seqPatRightBtn.x, seqPatRightBtn.y,
                  seqPatRightBtn.w, seqPatRightBtn.h, WHITE);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(seqPatRightBtn.x + (seqPatRightBtn.w - 18) / 2,
                   seqPatRightBtn.y + (seqPatRightBtn.h - 24) / 2);
  M5.Lcd.print(">");

  // SAVEボタン
  M5.Lcd.fillRect(seqSaveBtn.x, seqSaveBtn.y,
                  seqSaveBtn.w, seqSaveBtn.h, RED);
  M5.Lcd.drawRect(seqSaveBtn.x, seqSaveBtn.y,
                  seqSaveBtn.w, seqSaveBtn.h, WHITE);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(seqSaveBtn.x + (seqSaveBtn.w - 48) / 2,
                   seqSaveBtn.y + (seqSaveBtn.h - 16) / 2);
  M5.Lcd.print("SAVE");

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

    M5.Lcd.setTextColor(slotTextColor);
    M5.Lcd.setTextSize(3);
    int vx = seqSteps[i].slotX + (seqSteps[i].slotW - (int)strlen(valueStr) * 18) / 2;
    int vy = seqSteps[i].slotY + (seqSteps[i].slotH - 24) / 2;
    M5.Lcd.setCursor(vx, vy);
    M5.Lcd.print(valueStr);

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
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(seqStepLeftBtn.x + (seqStepLeftBtn.w - 18) / 2,
                   seqStepLeftBtn.y + (seqStepLeftBtn.h - 24) / 2);
  M5.Lcd.print("<");

  M5.Lcd.fillRect(seqStepRightBtn.x, seqStepRightBtn.y,
                  seqStepRightBtn.w, seqStepRightBtn.h, BLUE);
  M5.Lcd.drawRect(seqStepRightBtn.x, seqStepRightBtn.y,
                  seqStepRightBtn.w, seqStepRightBtn.h, WHITE);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(seqStepRightBtn.x + (seqStepRightBtn.w - 18) / 2,
                   seqStepRightBtn.y + (seqStepRightBtn.h - 24) / 2);
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
      // RELATIVE_MODE, SEQUENCE_MODE: 何もしない
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
    else if (currentMode == RELATIVE_MODE) currentMode = SEQUENCE_MODE;
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
      // シーケンスモード：現在ステップの転調値を適用
      handleTransposeChange(seqPatterns[seqCurrentPattern][seqCurrentStep]);
      Serial.println("SequenceMode entered");
    }

    needFullRedraw = true;
    lastButtonCheck = now;
    const char* modeNames[] = {"DIRECT", "KEY", "RELATIVE", "SEQUENCE"};
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
    processSequenceModeTouch(pos);
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
    if (saveSequencesToSD()) {
      // 保存成功フィードバック：ボタンを緑に一瞬表示
      M5.Lcd.fillRect(seqSaveBtn.x, seqSaveBtn.y,
                      seqSaveBtn.w, seqSaveBtn.h, GREEN);
      M5.Lcd.drawRect(seqSaveBtn.x, seqSaveBtn.y,
                      seqSaveBtn.w, seqSaveBtn.h, WHITE);
      M5.Lcd.setTextColor(BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(seqSaveBtn.x + 18, seqSaveBtn.y + 3);
      M5.Lcd.print("OK!");
      delay(500);
    } else {
      // 保存失敗フィードバック
      M5.Lcd.fillRect(seqSaveBtn.x, seqSaveBtn.y,
                      seqSaveBtn.w, seqSaveBtn.h, ORANGE);
      M5.Lcd.drawRect(seqSaveBtn.x, seqSaveBtn.y,
                      seqSaveBtn.w, seqSaveBtn.h, WHITE);
      M5.Lcd.setTextColor(BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(seqSaveBtn.x + 10, seqSaveBtn.y + 3);
      M5.Lcd.print("ERR!");
      delay(500);
    }
    needFullRedraw = true;
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