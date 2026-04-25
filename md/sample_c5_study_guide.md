# sample_c5_study 実装ガイド

## 1. シミュレーション実行方法

### 基本手順

```bash
cd /home/kaito/etrobo
source scripts/etroboenv.sh silent
make app=sample_c5_study
```

### 実行時の操作

| アクション | 内容 |
|-----------|------|
| フォースセンサー押下 | トレース開始 |
| RIGHTボタン長押し | トレース停止 |
| フォースセンサー再押下 | トレース再開 |

### 出力

```
実行物配置先: raspike-athrill-v850e2m/sdk/workspace/simdist/sample_c5_study/
ファイル: sample_c5_study.asp
```

---

## 2. 機能変更方法（make時に反映）

### ライントレース制御の変更

**ファイル**: `sample_c5_study/LineTracer/LineTracer.h`

| パラメータ | 説明 |
|-----------|------|
| `WHITE_BRIGHTNESS` | 目標輝度（白） |
| `BLACK_BRIGHTNESS` | 目標輝度（黒） |
| `STEERING_COEF` | 操舵係数 |
| `BASE_SPEED` | 基準速度 |
| `LEFT_EDGE` / `RIGHT_EDGE` | 走行エッジ選択 |

**実装ファイル**: `sample_c5_study/LineTracer/LineTracer.c`
- `steering_amount_calculation()` : 操舵量計算
- `motor_drive_control()` : モータ出力制御

### タスク開始停止の変更

**ファイル**: `sample_c5_study/app.c`
- フォースセンサー/ボタン判定
- 周期タスク開始・停止（`sta_cyc` / `stp_cyc`）

### ビルド時コンパイルオプション

**ファイル**: `sample_c5_study/Makefile.inc`

```makefile
# CSV出力ON/OFF切替
COPTS += -DENABLE_TRACE_CSV
```

---

## 3. 追加したデバッグ機能

### CSV自動ロギング

**実装位置**: `sample_c5_study/TraceLogger/`

- 独立周期タスク（100msec毎）でセンサー・モータ値をサンプリング
- `app.c` でトレース開始・停止時に呼び出し

### CSV出力内容

```
run_id,seq,reflection,left_count,right_count,current_position,left_power,right_power
```

| 項目 | 説明 |
|------|------|
| `run_id` | 開始～停止の1サイクルID |
| `seq` | サイクル内のサンプル番号 |
| `reflection` | カラーセンサ反射光値 |
| `left_count`, `right_count` | モータ回転角（現在値） |
| `current_position` | 位置推定値（左右カウント平均） |
| `left_power`, `right_power` | モータ出力（現在値） |

### CSV有効化手順

1. `sample_c5_study/Makefile.inc` の以下をコメント外す:
   ```makefile
   COPTS += -DENABLE_TRACE_CSV
   ```

2. ビルド実行:
   ```bash
   make app=sample_c5_study
   ```

3. 出力先:
   ```
   simdist/sample_c5_study/line_trace_log.csv
   ```

### CSV無効時の挙動

- 実行停止時にコンソールへCSV内容をダンプ
- メモリバッファ（最大4096サンプル）に記録

---

## 4. APIの定義場所

### フォースセンサ

| 種類 | パス |
|------|------|
| 宣言 | `libraspike-art/drivers/include/spike/pup/forcesensor.h` |
| 実装 | `libraspike-art/src/raspike_forcesensor.c` |

### Hubボタン

| 種類 | パス |
|------|------|
| 宣言 | `libraspike-art/drivers/include/spike/hub/button.h` |
| 実装 | `libraspike-art/src/raspike_hub.c` |

### モータ制御

```
libraspike-art/drivers/include/spike/pup/motor.h
```

### カラーセンサ

```
libraspike-art/drivers/include/spike/pup/colorsensor.h
```

### RTOS呼び出し

| 種類 | パス |
|------|------|
| 宣言 | `include/kernel.h` |
| 実装 | `kernel/task_sync.c`, `kernel/task_term.c`, `kernel/cyclic.c` |

### 型定義

```
pbio_port_id_t: libraspike-art/external/libpybricks/lib/pbio/include/pbio/port.h
pbio_error_t: libraspike-art/external/libpybricks/lib/pbio/include/pbio/error.h
pup_device_t: libraspike-art/drivers/spike/pup_device.h
```

---

## 5. 主な課題と対応

### 現状課題

- 右側モーターのみ回るケースあり

### 確認すべき項目

- `LineTracer.c` の初期化（モータ方向設定）
- `LineTracer.c` の操舵計算と出力値
- トレースログの `left_power` / `right_power` 値

### 改善案

- `STEERING_COEF` の再調整
- `BASE_SPEED` の再検討
- 出力クリップ処理の追加
- 必要に応じてP制御 → PI/PID制御へ拡張
