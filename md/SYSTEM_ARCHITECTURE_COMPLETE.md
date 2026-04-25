# etrobo 統合システム架構図 ～ ライントレーシング制御システム全体

## 概要

このシステムは、複数の層が協調して動作する統合的なロボット制御システムです。アプリケーションレベルの `app.c` だけでなく、RTOS、API層、ミドルウェア、デバイス抽象化層、そしてシミュレーション環境まで、すべてが連動して機能しています。

---

## 【全体システムアーキテクチャ】7層構造

```
┌───────────────────────────────────────────────────────────────────────┐
│ Layer 1: Application                                                  │
│ ┌─────────────┐      ┌──────────────┐      ┌────────────────┐        │
│ │  app.c      │      │ LineTracer.c │      │TraceDataLogger │        │
│ │(オーケストレ)│      │  (制御ロジック)│      │   .c(記録)     │        │
│ └─────────────┘      └──────────────┘      └────────────────┘        │
└───────────────────────────────────────────────────────────────────────┘
                                ↓ (SPIKE API)
┌───────────────────────────────────────────────────────────────────────┐
│ Layer 2: SPIKE API (spike-rt ハードウェア統一API)                     │
│ ┌─────────────────────────────────────────────────────────────────┐  │
│ │ #include "spikeapi.h"  pup_motor_*, pup_*_sensor_*             │  │
│ │ (モーター/センサーの統一インターフェース)                            │  │
│ └─────────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────────┘
                                ↓ (EV3 API Layer)
┌───────────────────────────────────────────────────────────────────────┐
│ Layer 3: EV3 API & Device Management (spikeapi/)                      │
│ ├─ ev3api.c: API初期化・共通処理                                      │
│ ├─ raspike_device.c: ポート管理テーブル (pbio_port_id_t → pup_device)│
│ ├─ raspike_motor.c: モーター制御                                      │
│ ├─ raspike_color.c: カラーセンサー入力                                │
│ └─ raspike_forcesensor.c: フォースセンサー入力                        │
└───────────────────────────────────────────────────────────────────────┘
                                ↓ (pbio API)
┌───────────────────────────────────────────────────────────────────────┐
│ Layer 4: pbio (Physical Block I/O) - ハードウェア抽象化層            │
│ ├─ pbio/port.h: pbio_port_id_t (PBIO_PORT_ID_A/B/D/E)                │
│ ├─ pbio/error.h: error handling                                      │
│ └─ pup_device_t: デバイス構造体（ポート、タイプ、状態）               │
└───────────────────────────────────────────────────────────────────────┘
                                ↓ (Device Drivers)
┌───────────────────────────────────────────────────────────────────────┐
│ Layer 5: Hardware Drivers (libraspike-art/drivers/spike/)             │
│ ├─ Motor Control Logic                                               │
│ ├─ Sensor Interface                                                  │
│ ├─ LED/Display Control                                               │
│ └─ Port Management                                                   │
└───────────────────────────────────────────────────────────────────────┘
                                ↓ (RTOS Kernel)
┌───────────────────────────────────────────────────────────────────────┐
│ Layer 6: TOPPERS/ASP3 RTOS Kernel                                    │
│ ├─ Task Management: main_task, tracer_task, trace_logger_task        │
│ ├─ Cyclic Handler: 100ms周期実行管理                                  │
│ ├─ Priority Scheduling: MAIN(5) > TRACER(9) > LOGGER(10)             │
│ └─ Memory Management: スタック、ヒープ                                │
└───────────────────────────────────────────────────────────────────────┘
                                ↓ (Simulator vdev UDP)
┌───────────────────────────────────────────────────────────────────────┐
│ Layer 7: Simulator Integration (vdev_udp - athrill2)                 │
│ ├─ UDP Port TX: 54001  RX: 54002                                      │
│ ├─ Virtual Device Interface (VDEV protocol)                           │
│ ├─ Memory Mapping: ROM 0x00000000, RAM 0x00200000 etc.                │
│ └─ Communication Protocol: VdevTxDataHeadType                         │
└───────────────────────────────────────────────────────────────────────┘
                                ↓ (external)
┌───────────────────────────────────────────────────────────────────────┐
│ External Simulator Environment (Unity EV3 Simulator)                 │
│ ├─ Robot Physics Model                                               │
│ ├─ Motor Actuator Simulation                                         │
│ ├─ Sensor Data Generation (Color, Force detection)                   │
│ └─ Course Track/Environment                                          │
└───────────────────────────────────────────────────────────────────────┘
```

---

## 【RTOS層の詳細】設定ファイルとタスク定義

### app.cfg の役割（RTOS設定の中枢）

アプリケーションのタスク定義はコンパイル時に `app.cfg` に記述されます：

```cfg
INCLUDE("app_common.cfg");    ← 共通RTOS設定（EV3 API層）
INCLUDE("tecsgen.cfg");       ← 自動生成コンポーネント設定

CRE_TSK( MAIN_TASK, ... );           ← メインタスク定義
CRE_TSK( LINE_TRACER_TASK, ... );    ← ライントレースタスク定義
CRE_TSK( TRACE_LOGGER_TASK, ... );   ← ロガータスク定義

CRE_CYC( LINE_TRACER_TASK_CYC, ... );  ← 周期タスク（100ms）
CRE_CYC( TRACE_LOGGER_TASK_CYC, ... ); ← 周期タスク（100ms）
```

### 設定ファイル読み込み階層

```
┌──────────────────────────────────────────────────────┐
│ Step 1: app.cfg (アプリケーション固有)              │
│  ├─ INCLUDE("app_common.cfg")                       │
│  │   └─ EV3標準設定                                 │
│  │       ├─ INCLUDE("ev3.cfg")                     │
│  │       └─ INCLUDE("ev3api.cfg")                  │
│  │           └─ spikeapi/ API設定                  │
│  └─ INCLUDE("tecsgen.cfg")                         │
│      └─ 自動生成コンポーネント                      │
└──────────────────────────────────────────────────────┘
            ↓ (TECSGEN による処理)
┌──────────────────────────────────────────────────────┐
│ Step 2: kernel_cfg.c/.h 自動生成                     │
│  ├─ テーブル生成（タスク、セマフォ、イベント等）     │
│  ├─ メモリマップ最適化                               │
│  └─ CPUレジスタ設定                                  │
└──────────────────────────────────────────────────────┘
            ↓ (compilation)
┌──────────────────────────────────────────────────────┐
│ Step 3: asp バイナリ（実行形式）                    │
│  └─ athrill2 で実行可能                             │
└──────────────────────────────────────────────────────┘
```

### タスク構成表

| タスク | 優先度 | 周期 | 処理内容 |
|-------|--------|------|--------|
| **MAIN_TASK** | 5（最高） | イベント駆動 | システム初期化、イベント処理 |
| **LINE_TRACER_TASK** | 9 | 100ms | センサー読込→差分計算→モーター制御 |
| **TRACE_LOGGER_TASK** | 10 | 100ms | 走行データ記録（メモリバッファ保存） |

---

## 【デバイス抽象化】ハードウェアマッピング

### ポート定義の階層

```
Application Level
    ↓ (symbolic names)
app.c: "left_motor_port = PBIO_PORT_ID_B"
    ↓ (pbio port constants)
pbio/port.h: PBIO_PORT_ID_B = 1
    ↓ (device table lookup)
raspike_device.c: device_table[1] → pup_motor_t
    ↓ (actual hardware)
libraspike-art/drivers: Port B → LEFT MOTOR HW
```

### ハードウェア I/O マッピング表

| ポート記号 | pbio const | 物理HW | 入出力 | 用途 |
|----------|-----------|--------|-------|------|
| `PBIO_PORT_A` | 0 | RIGHT MOTOR | Out | 右モーター（時計回転） |
| `PBIO_PORT_B` | 1 | LEFT MOTOR | Out | 左モーター（反時計回転） |
| `PBIO_PORT_D` | 3 | FORCE SENSOR | In | フォースセンサー |
| `PBIO_PORT_E` | 4 | COLOR SENSOR | In | カラーセンサー |
| Hub.RIGHT_BTN | - | Button | In | 右ボタン |

---

## 【ビルドプロセス】完全フロー

### Makefileチェーン

```
$ make img=sample_c5_study
    ↓
Makefile.workspace (統合)
    ├─ Makefile.prj.common (SPIKE API設定読み込み)
    ├─ app.cfg 検出
    ├─ TECSGEN実行: app.cfg → kernel_cfg.h
    ├─ cfg.rb(Ruby): cfg解析 → kernel_cfg.c
    ├─ オブジェクトビルド
    │   ├─ app.o
    │   ├─ LineTracer.o
    │   ├─ TraceDataLogger.o
    │   ├─ spikeapi/ev3api.o
    │   ├─ spikeapi/raspike_*.o
    │   └─ kernel_cfg.o
    ├─ ライブラリリンク: libraspike-art.a
    └─ aspバイナリ生成
         → sdk/workspace/simdist/sample_c5_study/asp
```

### ビルドステップ詳細

| Step | 処理 | 入力 | 出力 |
|-----|------|------|------|
| 1 | Workspace Assembly | Makefile.workspace | - |
| 2 | TECSGEN実行 | app.cfg | cfg1_out.c |
| 3 | C前処理 | cfg1_out.c | offset.h, kernel_cfg.h |
| 4 | テンプレート処理 | offset.h | kernel_cfg.c |
| 5 | コンパイル | *.c files | *.o files |
| 6 | リンク | *.o, library.a | asp binary |

---

## 【シミュレーション環境連携】device_config.txt & memory.txt

### device_config.txt: Athrill2初期化パラメータ

ファイル位置: `sdk/common/device_config.txt`

```
# デバイス名前空間マッピング
DEVICE_CONFIG_UART_BASENAME  __ev3rt_uart
DEVICE_CONFIG_BT_BASENAME    __ev3rt_bt
DEVICE_CONFIG_VIRTFS_TOP     __ev3rtfs

# 仮想デバイス (vdev) による UDP通信
DEBUG_FUNC_ENABLE_VDEV       1      # ← UDP通信有効化スイッチ

# UDP通信ポート設定
DEBUG_FUNC_VDEV_TX_PORTNO    54001  # athrill2が受信するポート
DEBUG_FUNC_VDEV_TX_IPADDR    127.0.0.1
DEBUG_FUNC_VDEV_RX_PORTNO    54002  # athrill2が送信するポート
DEBUG_FUNC_VDEV_RX_IPADDR    127.0.0.1

# シミュレータ時間同期
DEBUG_FUNC_ENABLE_SKIP_CLOCK 1
DEBUG_FUNC_ENABLE_SYNC_TIME  0
```

### memory.txt: メモリマップ定義

ファイル位置: `sdk/common/memory.txt`

```
ROM,  0x00000000, 2048   # ROM領域（カーネルコード）
RAM,  0x00200000, 2048   # RAM領域1（スタック）
RAM,  0x05FF7000, 10240  # RAM領域2（ヒープ/グローバル）
RAM,  0x07FF7000, 10240  # RAM領域3（I/Oメモリマップ）
```

### UDP 通信プロトコル (VDEV)

```
athrill2                              Unity Simulator
  ↓ (Port 54001)  ──[VDEV Packet]──→   ↑ (Port 54002)
  ↑                                    ↓
  ←──────[Response Packet]──────────
```

#### VDEVパケット構造

```c
typedef struct VdevTxDataHeadType {
  char header[4];       // "VDEV"
  uint32_t version;     // Protocol version
  uint32_t reserve[2];  // Reserved
  uint64_t unity_time;  // Simulator time (microseconds)
  uint32_t ext_off;     // Extension offset
  uint32_t ext_size;    // Extension size
  uint8_t data[...];    // Memory dump/update data
} VdevTxDataHeadType;
```

---

## 【実行時フロー】統合ビュー

### システム初期化 → 実行 → 停止

```
┌─────────────────────────────────────────────────┐
│ [1] INITIALIZATION                              │
├─────────────────────────────────────────────────┤
│ $ make start                                    │
│  ↓                                              │
│ athrill2 -c1 -m memory.txt -d device_config.txt │
│          -t -1 asp                              │
│  ↓                                              │
│ [athrill2 starts]                               │
│ 1. device_config.txt読込 (UDP設定)              │
│ 2. memory.txt読込 (メモリ初期化)                 │
│ 3. aspバイナリ読込・実行開始                     │
│ 4. main_task() 起動                            │
└─────────────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────┐
│ [2] KERNEL BOOT                                 │
├─────────────────────────────────────────────────┤
│ main_task()                                     │
│  ├─ LineTracer_Configure()                      │
│  └─ TraceDataLogger_Configure()                 │
│      ↓ (デバイステーブル初期化)                 │
│ printf("Press force sensor to start")           │
└─────────────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────┐
│ [3] WAITING                                     │
├─────────────────────────────────────────────────┤
│ while (!pup_force_sensor_touched())             │
│   dly_tsk(10ms)  ← RTOS待機                     │
│      ↓ [User presses force sensor]             │
│  UDP Response: sensor state update              │
│      ↓                                          │
│  pup_force_sensor_touched() → true              │
└─────────────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────┐
│ [4] 100ms 周期実行 (RUNNING)                    │
├─────────────────────────────────────────────────┤
│ T=100ms: [CYC Handler]                          │
│   ├─ LINE_TRACER_TASK                           │
│   │  ├─ steering_calculation()                  │
│   │  └─ motor_drive_control()                   │
│   │     └─ pup_motor_set_power() ──UDP──→      │
│   │                                             │
│   ├─ TRACE_LOGGER_TASK                          │
│   │  ├─ Read sensor values                      │
│   │  └─ Store to buffer[]                       │
│   └─ dly_tsk(100ms)                             │
│                                                 │
│ T=200ms: [Repeat]                               │
│  ...                                            │
│                                                 │
│ T=N: [Hub button pressed]                       │
│   ├─ stp_cyc(TRACER_CYC)                       │
│   ├─ stp_cyc(LOGGER_CYC)                       │
│   ├─ LineTracer_Stop() → Motor PWM=0           │
│   ├─ TraceDataLogger_DumpCsv() → FILE          │
│   └─ Wait for force sensor again               │
└─────────────────────────────────────────────────┘
                   ↓
┌─────────────────────────────────────────────────┐
│ [5] OUTPUT & LOOP                               │
├─────────────────────────────────────────────────┤
│ CSV File: simdist/sample_c5_study/               │
│           line_trace_log.csv                    │
│ ────────────────────────────────────────────    │
│ run_id,seq,reflection,left_count,right_count    │
│ 1,0,25,-100,-95                                 │
│ 1,1,24,-200,-195                                │
│ ...                                             │
│                                                 │
│ → LOOP: フォースセンサー待機に戻る              │
└─────────────────────────────────────────────────┘
```

---

## 【各層の相互作用】依存関係図

### 1. Application ↔ RTOS Kernel

```
┌──────────────────────┐
│ Application (app.c)  │
│                      │
│ main_task()          │ RTOS API:
│ tracer_task()        │ ・sta_cyc() - タスク起動
│ trace_logger_task()  │ ・stp_cyc() - タスク停止
└──────────────────────┘ ・dly_tsk() - 待機
          ↕              ・ext_tsk() - 終了
┌──────────────────────────────────────────┐
│ TOPPERS/ASP3 Kernel                      │
│ ├─ Task Manager (CRE_TSK)               │
│ ├─ Cyclic Handler (CRE_CYC@100ms)       │
│ ├─ Scheduler (Pri 5<9<10)               │
│ └─ Timer (100ms周期)                     │
└──────────────────────────────────────────┘
```

### 2. RTOS Kernel ↔ SPIKE API

```
RTOS Kernel
    ↓ (Task context)
SPIKE API: pup_motor_set_power()
           pup_color_sensor_reflection()
    ↓ (Device lookup)
EV3 API: raspike_motor.c, raspike_color.c
```

### 3. API ↔ Hardware Driver ↔ vdev_udp ↔ Simulator

```
SPIKE API: pup_motor_set_power(motor, 30)
             ↓
EV3 API: ev3_motor_set_power()
             ↓ (pbio port lookup)
Driver: motor_control.c
             ↓
PAD Memory: 0x7ff70000 + offset = 30 (write)
             ↓ (detect write)
vdev_udp.c: Monitors PAD memory
             ↓
UDP Socket: send to 127.0.0.1:54001
             ↓
External Simulator (Unity)
    Parse VDEV packet
    Update motor model: power=30
    Simulate physics
    Encoder feedback
             ↓
UDP Socket: send to 127.0.0.1:54002 (response)
             ↓
vdev_udp.c: Receive sensor data
    Update PAD memory: reflection_value
             ↓
Application continues
    pup_color_sensor_reflection() → reads value
```

---

## 【アプリケーション層の詳細】モジュール構成

### 1. Main Task (app.c)

**役割**: システム全体のオーケストレーション

```
初期化フェーズ
   ↓
[LineTracer_Configure(ports)]
[TraceDataLogger_Configure(ports)]
[フォースセンサー待機]
   ↓
[フォースセンサー押下検出]
   ├─ sta_cyc(LINE_TRACER_TASK_CYC)
   ├─ sta_cyc(TRACE_LOGGER_TASK_CYC)
   └─ 100ms周期実行スタート
   ↓
[右ボタン押下検出]
   ├─ stp_cyc(両タスク周期)
   ├─ LineTracer_Stop()
   ├─ TraceDataLogger_DumpCsv()
   └─ フォースセンサー待機に戻る → LOOP
```

### 2. LineTracer Module (LineTracer.c/h)

**処理フロー**:

1. **輝度読み込み**: `pup_color_sensor_reflection()` → 0-100
2. **目標値計算**: `(WHITE_BRIGHTNESS + BLACK_BRIGHTNESS) / 2`
3. **差分計算**: `target - current`
4. **操舵量計算**: `差分 × STEERING_COEF (1.5)`
5. **モーター駆動**: 左右モーター異なる速度

**制御ルール**:

```
黒線上     → 低輝度 (10)    → 負の差分 → 右カーブ
白地上     → 高輝度 (40)    → 正の差分 → 左カーブ
エッジ上   → 中間値 (25)    → 差分≈0   → 直進
```

### 3. TraceDataLogger Module (TraceDataLogger.c/h)

**記録データ**:

| 項目 | 用途 |
|------|------|
| run_id | セッション識別 |
| seq | 周期シーケンス番号 |
| reflection | センサー輝度値 |
| left_count | 左モーター回転数 |
| right_count | 右モーター回転数 |
| current_position | 推定走行距離 |
| left_power | 左モーター出力 |
| right_power | 右モーター出力 |

**出力形式** (line_trace_log.csv):

```csv
run_id,seq,reflection,left_count,right_count,current_position,left_power,right_power
1,0,25,-100,-95,0,30,30
1,1,24,-200,-195,10,32,28
...
```

---

## 【拡張ポイント】今後の改善案

### 現在の特徴

✅ **モジュール分離** - 各機能が独立  
✅ **イベント駆動** - 応答性が高い  
✅ **RTOS統合** - マルチタスク対応  
✅ **シミュレーション連携** - 開発効率が高い

### 追加可能な機能

| 機能 | 層 | 実装方法 |
|------|-----|--------|
| **PID制御** | App | LineTracer.c に Kp, Ki, Kd追加 |
| **IMU統合** | Layer 3 | raspike_imu.c 追加 |
| **ジャイロ** | Layer 3 | raspike_gyro.c 追加 |
| **Bluetooth通信** | Layer 3 | BT middleware統合 |
| **USB Camera** | Layer 3 | Camera driver追加 |
| **音声出力** | Layer 3 | Audio API追加 |
| **mROS連携** | Layer 3 | mROS middleware |

---

## 【まとめ】システム全体の概観

```
┌─────────────────────────────────────────────┐
│ 7層統合制御システム                          │
├─────────────────────────────────────────────┤
│ ┌─────────────────────────────────────────┐ │
│ │ Layer 1: Application                    │ │
│ │ ├─ main_task (orchestration)            │ │
│ │ ├─ LineTracer (control logic)           │ │
│ │ └─ Logger (data recording)              │ │
│ └─────────────────────────────────────────┘ │
│                   ↓                         │
│ ┌─────────────────────────────────────────┐ │
│ │ Layers 2-5: APIs & Drivers              │ │
│ │ ├─ SPIKE API (high-level)               │ │
│ │ ├─ EV3 API (mid-level)                  │ │
│ │ ├─ pbio API (abstraction)               │ │
│ │ └─ Hardware Drivers (low-level)         │ │
│ └─────────────────────────────────────────┘ │
│                   ↓                         │
│ ┌─────────────────────────────────────────┐ │
│ │ Layer 6: TOPPERS/ASP3 RTOS              │ │
│ │ ├─ Task Scheduler                       │ │
│ │ ├─ Cyclic Timer (100ms)                 │ │
│ │ └─ Memory Management                    │ │
│ └─────────────────────────────────────────┘ │
│                   ↓                         │
│ ┌─────────────────────────────────────────┐ │
│ │ Layer 7: Simulator Integration (vdev)   │ │
│ │ ├─ UDP Protocol (54001/54002)           │ │
│ │ ├─ Memory Mapping                       │ │
│ │ └─ Sensor/Actuator Simulation           │ │
│ └─────────────────────────────────────────┘ │
│                   ↓                         │
│ ┌─────────────────────────────────────────┐ │
│ │ External: Physics Engine (Unity)        │ │
│ │ ├─ Motor Simulation                     │ │
│ │ ├─ Sensor Data Generation               │ │
│ │ └─ Course Environment                   │ │
│ └─────────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
```

### 重要な設定ファイル

| ファイル | 役割 |
|---------|------|
| `app.cfg` | RTOS タスク定義 |
| `app.h` | 優先度・周期定義 |
| `device_config.txt` | Athrill2 初期化パラメータ |
| `memory.txt` | メモリマップ定義 |
| `Makefile.workspace` | ビルド統合 |
| `LineTracer.h` | 制御パラメータ |

このシステムは、**複数の抽象化層が明確に分離された、スケーラブルで拡張性の高い設計** になっています。各層を独立して改善・交換することで、より高度なロボット制御を実現できます。
