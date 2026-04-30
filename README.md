# M5Core-MIDIXposeFilBTUM

M5Core2 + M5 Unit MIDI (SAM2695) を使った **MIDI 演奏機 / トランスポーザー / メッセージ管理ツール** です。  
起動直後は `PLAY` モード (Unit MIDI 内蔵 GM 音源を直接鳴らす) に入り、`C` 長押しで転調機能・MIDI 加工機能の各グループに移れます。

現在のスケッチ本体は [M5Core2-MIDIXposeFilBT.ino](/D:/M5/M5Core2-MIDIXposeFilBTUM/M5Core2-MIDIXposeFilBT.ino) です。

## ハードウェア

- **本体**: M5Stack Core2
- **音源**: M5 Unit MIDI (SAM2695)
- **接続**: Core2 の **Port A (Grove)** に Unit MIDI を挿す
- **UART**: G33=RX / G32=TX, 31250 bps (`Serial2`)
- **注意**: 起動時に `Wire.end()` を呼んで Port A の I2C を解放するため、Port A 上の他の I2C デバイスは併用できません。

## 概要 — 演奏 (PLAY) モード

起動直後の基本モードです。Unit MIDI 内蔵 GM 音源を直接鳴らせます。

- 外部 MIDI キーボードから届いた Note は、`FILTER / MAPPER / Transpose` を経由せずそのまま発音されます。
- 画面の音色名バーをタップすると 128 音色から選択できます (Page 切替対応)。
- `VOL- / VOL+` で CC#7 を、`PRG- / PRG+` で Program を、`PB- / PB+` で Pitch Bend を、`SUS` で CC#64 を、`INIT` で再初期化を送れます。
- `TEST PHRASE` を押すと、現在選択中の音色で **ドビュッシー「アラベスク第1番」冒頭** を試聴できます。
- `PLAY` 画面に入った時点で Roland `GS Reset` を送り、現在の `Volume` / `Program` / `Pitch Bend` / `Sustain` を再送します。

## 概要 — 転調 / MIDI 管理

`PLAY` 以外のモードでは、入力された MIDI メッセージに対して次の順序で処理します。

1. `FILTER`
2. `MAPPER`
3. `Transpose`
4. MIDI OUT

`FILTER` と `MAPPER` はそれぞれ独立して `BYPASS` / `ACTIVE` を切り替えられます。  
両方を `BYPASS` にすれば、従来どおりの低遅延な転調処理だけを使えます。

`Serial2` を関数で包むような変更は、速度向上や低遅延化にはほぼ寄与しません。  
今の実装でも UART の直接送受信が最短経路なので、性能面では現状維持で問題ありません。  
ラッパー化の主な利点は、将来別の MIDI ハードへ差し替えるときの保守性だけです。

## モード構成

画面は `演奏`、`転調`、`MIDI 管理` の 3 つを軸にした構成です。  
`C` 長押しでグループを巡回します: `演奏 -> 転調 -> MIDI 管理 -> 演奏 ...`

### 0. 演奏モード (PLAY) — 起動直後の基本モード

Unit MIDI 内蔵 SAM2695 を直接鳴らすモードです。

操作:

- 画面音色名バー: 128 音色ピッカーを開く
- `VOL- / VOL+`: CC#7 を 8 ステップ単位 (0–127)
- `PRG- / PRG+`: Program 番号を 1 つ前後
- `PB- / PB+`: Pitch Bend を 256 ステップ単位 (0–16383, 中央 8192)
- `SUS`: CC#64 (Sustain) を ON / OFF
- `INIT`: GS Reset → Volume / Program / Bend / Sustain を再送
- `TEST PHRASE`: アラベスク冒頭を演奏 (現在の音色で発音)
- `B`: 初期化 (`INIT` と同じ)
- `C` 短押し: パニック(All Notes Off を全 16ch に送出)
- `C` 長押し: `演奏 -> 転調`

### 1. 転調グループ

短押し `C` で次を巡回します。

- `DIRECT`
- `KEY`
- `INSTANT`
- `SEQUENCE`

### 2. MIDI 管理グループ

長押し `C` で `転調 -> MIDI 管理` に移ります。  
短押し `C` で次を巡回します。

- `FILTER`
- `MAPPER`

M5 Unit MIDI を接続している場合は、ここで処理した MIDI をそのまま Unit MIDI から発音できます。  
MIDI フィルタやマッパーを使いながら、実音を確認できます。

長押し `C` は `演奏 -> 転調 -> MIDI 管理 -> 演奏` の順で移動します。  
転調側へ戻るときは最後に使っていた転調サブモードへ戻ります。

## ハードウェアボタン

### 共通

- `A`: All Notes Off の有効/無効切替
- `C` 短押し: 現在グループ内の次モードへ
- `C` 長押し: `演奏 -> 転調 -> MIDI 管理 -> 演奏`

### 転調グループ中の `B`

- `DIRECT`: レンジ切替
  - `0..+11`
  - `-11..0`
  - `-5..+6`
- `KEY`: 上位転調/通常転調の切替
- `INSTANT`: 何もしない
- `SEQUENCE`: 何もしない

### MIDI 管理グループ中の `B`

- `FILTER`: `Type` を次のメッセージ種別へ進める
- `MAPPER`: `PG1/PG2` 切替

## 転調機能

### DIRECT

12 ボタンの直接選択方式です。  
現在レンジ内の転調値をそのまま選択します。

### KEY

メジャー/マイナーのキー指定で転調値を決定します。

### INSTANT

よく使う転調値をワンタップで呼び出します。

- `0`
- `+1`
- `+2`
- `+3`
- `+5`
- `-1`
- `-2`
- `-3`
- `-5`

### SEQUENCE

複数ステップの転調値パターンを順番に呼び出します。  
ステップ値編集、ステップ移動、パターン切替、SD 保存に対応しています。

## MIDI Manager

### FILTER

不要な MIDI メッセージをブロックします。  
一致したメッセージは `MAPPER` と `Transpose` に進まず、その場で破棄されます。

現状のルール定義項目:

- `EN/DIS`
- `Type`
- `Ch`
- `ADD`
- `DEL`
- `UP`
- `DOWN`

`Type` はタップまたは `B` ボタンで順送りします。  
`Ch` は `ALL` または `Ch1..Ch16` を切り替えます。

#### 対応メッセージ種別

- `NoteOff`
- `NoteOn`
- `KeyPrs` (Key Pressure)
- `PrgChg` (Program Change)
- `CtrlChg` (Control Change)
- `ChPrs` (Channel Pressure)
- `Bend` (Pitch Bend)
- `SysEx`
- `MTC`
- `SongPos`
- `SongSel`
- `TuneReq`
- `Clock`
- `Start`
- `Cont`
- `Stop`
- `ActSn`
- `Reset`

### MAPPER

MIDI メッセージの再割り当て/変換を行います。  
リスト先頭から順に評価し、最初に一致したルールだけを適用します。

現状のルール定義項目:

- `EN/DIS`
- `ADD`
- `DEL`
- `UP`
- `DOWN`
- `PG1/PG2`

#### PG1

- `Type`
- `Ch`
- `Data1`
- `Min`
- `Max`

#### PG2

- `Type`
- `Ch`
- `Data1`
- `Min`
- `Max`

意味は次のとおりです。

- `PG1`: 変換元条件
- `PG2`: 変換先設定

補足:

- `Data1` は `ANY` / `KEEP` を使う項目があります
- `Min/Max` は値レンジ変換に使います
- `FILTER` の後に `MAPPER` が動作します

## 基本的な使い方

### 転調だけを使う場合

1. 長押し `C` で `MIDI Manager` に入っている場合は、もう一度長押し `C` で転調グループへ戻します。
2. 必要に応じて短押し `C` で `DIRECT` / `KEY` / `INSTANT` / `SEQUENCE` を選びます。
3. `MIDI Manager` を経由させたくない場合は、`FILTER` と `MAPPER` の両方を `BYPASS` にして使います。

### FILTER を設定する場合

1. 長押し `C` で `MIDI Manager` に入ります。
2. 短押し `C` で `FILTER` を表示します。
3. `ADD` でルールを追加し、対象ルールを一覧から選びます。
4. `Type` をタップ、または `B` ボタンでブロック対象のメッセージ種別を切り替えます。
5. `Ch` で `ALL` または `Ch1..Ch16` を選びます。
6. `EN/DIS` でそのルールを有効化します。
7. 画面上部の `BYPASS` / `ACTIVE` で、FILTER 全体を即座に有効/無効化できます。

### MAPPER を設定する場合

1. `MIDI Manager` 内で短押し `C` を使って `MAPPER` を表示します。
2. `ADD` でルールを追加し、対象ルールを一覧から選びます。
3. `B` ボタンで `PG1` と `PG2` を切り替えます。
4. `PG1` で変換元の `Type` / `Ch` / `Data1` / `Min` / `Max` を設定します。
5. `PG2` で変換先の `Type` / `Ch` / `Data1` / `Min` / `Max` を設定します。
6. `UP` / `DOWN` でルール順を変更します。評価順はリスト先頭からなので、上にあるルールほど優先されます。
7. `EN/DIS` で個別ルールを有効化し、上部の `BYPASS` / `ACTIVE` で MAPPER 全体の有効/無効を切り替えます。

### すぐに効果を確認する場合

- `FILTER` または `MAPPER` を `ACTIVE` にすると、その場で受信 MIDI に対して処理が反映されます。
- 効果比較をしたい場合は、上部の `BYPASS` と `ACTIVE` を切り替えるだけで元の経路と比較できます。
- 低遅延の従来動作に戻したい場合は、`FILTER` と `MAPPER` の両方を `BYPASS` にします。

## タッチ操作

`FILTER` / `MAPPER` / `BYPASS(ACTIVE)` は上段の大ボタンです。  
一覧から対象ルールを選び、下段の操作ボタンと編集ボックスで設定します。

現状の UI 方針:

- 上段: ページ/バイパス切替
- 中段: ルール一覧
- 下段: ルール操作
- 最下段: 編集項目

## MIDI 処理仕様メモ

- Realtime / Common メッセージも分類して処理
- `FILTER` はメッセージ単位でブロック
- `MAPPER` は最初に一致した 1 ルールを適用
- `Transpose` は主に Note On / Note Off へ適用
- `All Notes Off` は全 16ch に送信

現状の制限:

- `SysEx` はフィルタ対象だが、ペイロード変換は未実装

## ファイル構成

- [M5Core2-MIDIXposeFilBT.ino](/D:/M5/M5Core2-MIDIXposeFilBT/M5Core2-MIDIXposeFilBT.ino): 現在のメインスケッチ
- `src/`: Bluetooth HID 関連コード

## ビルドと書き込み

`arduino-cli` はスケッチ名とフォルダ名の一致を要求するため、本リポジトリでは
ビルド用のサブディレクトリにコピーしてからコンパイルしています。

```bash
# 例: Git Bash 上での 1 セット
mkdir -p /tmp/sketch_build/M5Core2-MIDIXposeFilBTUM
cp M5Core2-MIDIXposeFilBT.ino /tmp/sketch_build/M5Core2-MIDIXposeFilBTUM/M5Core2-MIDIXposeFilBTUM.ino
cp -r src /tmp/sketch_build/M5Core2-MIDIXposeFilBTUM/

arduino-cli compile --fqbn m5stack:esp32:m5stack_core2 /tmp/sketch_build/M5Core2-MIDIXposeFilBTUM
arduino-cli upload  -p COM10 --fqbn m5stack:esp32:m5stack_core2 /tmp/sketch_build/M5Core2-MIDIXposeFilBTUM
```

`-p` オプションには本機が見えている COM ポート (USB) を指定します
(`arduino-cli board list` で確認可能)。

## USB シリアルコマンド

PC から USB シリアルで本体を操作できます。
シリアル速度は `115200bps`、改行は `LF` または `CRLF` です。

起動後に `HELP` を送ると、利用できるコマンド一覧を返します。

### 主なコマンド

- `HELP`
- `STATUS`
- `REDRAW`
- `BUTTON A`
- `BUTTON B`
- `BUTTON C`
- `BUTTON C LONG`
- `TOUCH x y`
- `MODE PLAY`
- `MODE DIRECT`
- `MODE KEY`
- `MODE INSTANT`
- `MODE SEQUENCE`
- `MODE FILTER`
- `MODE MAPPER`
- `GROUP PLAY`
- `GROUP TRANSPOSE`
- `GROUP MIDI`
- `SET TRANSPOSE n`
- `INFO SCREEN`
- `SCREENSHOT PPM`
- `SCREENSHOT RGB888`

### 使い方の考え方

- `BUTTON` は本体の A/B/C ボタン操作を外部から再現します。
- `TOUCH x y` は画面の指定座標をタップしたのと同じ扱いです。
- `MODE` と `GROUP` は、目的の画面へ直接切り替えたいときに使います。
- `STATUS` は現在モード、転調値、FILTER/MAPPER の状態、MIDI 入出力カウントなどを 1 行で返します。

## スクリーンキャプチャ

画面のスクリーンショットは USB シリアル経由で取得できます。

### `SCREENSHOT PPM`

初心者向けのマニュアル作成や静止画保存に向く形式です。
コマンド送信後、最初に次のようなヘッダ行が返ります。

```text
OK SCREENSHOT format=PPM width=320 height=240 bytes=230415
```

その直後に、バイナリの `PPM(P6)` データ本体が流れます。
指定バイト数を読み切ると、最後に `OK SCREENSHOT_DONE` が返ります。

### `SCREENSHOT RGB888`

PC 側 GUI で直接扱いやすい、生の `RGB888` バイト列です。
返し方は `PPM` と同じで、先頭ヘッダだけが `format=RGB888` になります。

### 利用イメージ

1. `INFO SCREEN` で画面サイズを取得
2. `SCREENSHOT PPM` で現在画面を保存
3. 必要に応じて `TOUCH x y` や `BUTTON ...` で画面を操作
4. 再度 `SCREENSHOT ...` を取得

この仕組みにより、後で PC 側から大画面 UI を作るときに、本体画面を見ながら既存 UI をそのまま遠隔操作できます。
