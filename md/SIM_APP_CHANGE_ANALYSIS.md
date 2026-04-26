# ETロボコン シミュレーション環境
# app変更で挙動を変えるための最小理解ガイド

作成日: 2026-04-25
対象: etrobo 配下ドキュメントおよび関連実装ファイル

---

## 0. この文書の目的

ETロボコンのシミュレーション環境において、`app` を変更して挙動を変える開発者が、
最低限押さえるべき情報を「経路」「設定」「変更範囲」「拡張観点」で整理する。

---

## 1. センサー値が app 側へ届く経路

結論として、以下の経路で取得される。

1. app側タスクがセンサーAPIを呼ぶ
2. SPIKE API実装が vdev RX メモリを参照する
3. Athrill `vdev_udp` がUDP受信データを vdev 領域に反映する
4. 外部シミュレータ（Unity側）がセンサー値を生成して返す

### 代表的な根拠

- 全体フロー記載
  - `md/SYSTEM_ARCHITECTURE.md`
    - 実行時フロー (`app -> API -> driver -> Athrill UDP -> 外部sim -> app`)
- app側センサー取得
  - `raspike-athrill-v850e2m/sdk/workspace/sample_c5_study/LineTracer/LineTracer.c`
    - `pup_color_sensor_reflection(...)`
- API実装（RXメモリ読出し）
  - `raspike-athrill-v850e2m/sdk/common/spikeapi/src/raspike_color.c`
    - `sil_rew_mem((const uint32_t *)EV3_SENSOR_ADDR_REFLECT(...))`
- vdev RXアドレス定義
  - `raspike-athrill-v850e2m/target/v850_gcc/pil/include/ev3_vdev.h`
    - `EV3_SENSOR_ADDR_*` は `VDEV_RX_DATA_BASE` 起点
- Athrill受信反映
  - `athrill-target-v850e2m/src/device/peripheral/vdev/vdev_udp.c`
    - `udp_comm_read(...)`
    - `memcpy(... region->data ...)` でRXデータ反映

### 1.1 現状の環境で取得しやすい主な値（センサ以外も含む）

| 区分 | 取得例API | 取得できる値の例 | 備考 |
|---|---|---|---|
| カラーセンサ | `pup_color_sensor_reflection` | 反射光 | ライントレースで利用中 |
| カラーセンサ | `pup_color_sensor_rgb` | RGB | より詳細な路面判定に使える |
| カラーセンサ | `pup_color_sensor_ambient` | 環境光 | 外乱評価に使える |
| カラーセンサ | `pup_color_sensor_hsv` | HSV | `surface`引数は制限あり |
| フォースセンサ | `pup_force_sensor_touched` | 押下有無 | 開始トリガで利用中 |
| 超音波センサ | `pup_ultrasonic_sensor_distance` | 距離 | API表で対応あり |
| モータ | `pup_motor_get_count` | 回転角 | 位置推定やログで利用可能 |
| モータ | `pup_motor_get_power` | 現在出力 | 出力検証に有効 |
| Hubボタン | `hub_button_is_pressed` | ボタン状態 | 停止操作で利用中 |
| IMU | `hub_imu_get_acceleration` など | 加速度/角速度 | 対応表上は利用可能 |

補足:

- シミュレータ上で未対応/制限付きAPIがあるため、追加前に `raspike-athrill-v850e2m/README.md` の API対応表を確認する。
- センサ値だけでなく、モータカウント/出力/ボタン状態も挙動分析では重要な観測値。

### 1.2 センサー値を追加取得する場合の書き方

最短手順は次の通り。

1. 使用するセンサAPIのヘッダを追加
2. `*_get_device(...)` でデバイスポインタを初期化
3. 周期タスク内で値を取得
4. ログ出力（CSVやprintf）へ追加

#### 例A: 既存の `trace_logger_task` へ超音波距離を追加

```c
/* 追加 include */
#include "spike/pup/ultrasonicsensor.h"

/* 追加: デバイスポインタ */
static pup_device_t *s_ultrasonic_sensor;

/* Configureで初期化 */
s_ultrasonic_sensor = pup_ultrasonic_sensor_get_device(PBIO_PORT_ID_F);

/* 周期処理で取得 */
int32_t distance = pup_ultrasonic_sensor_distance(s_ultrasonic_sensor);

/* 必要ならCSV列へ追加 */
/* distance を s_samples に保存して出力 */
```

#### 例B: 既存のカラーセンサ値に RGB を追加

```c
/* 既存の color sensor device を流用 */
pup_color_rgb_t rgb = pup_color_sensor_rgb(s_color_sensor);

/* 例: ログへ追加 */
printf("rgb=%ld,%ld,%ld\n", (long)rgb.r, (long)rgb.g, (long)rgb.b);
```

#### 例C: app開始条件にフォース値を増やす（押下以外を使う場合）

```c
/* touched だけでなく、必要に応じて force APIを検討 */
/* ただしシミュレータ制限があるため README の対応表を先に確認 */
bool pressed = pup_force_sensor_touched(force_sensor);
if (pressed) {
    sta_cyc(LINE_TRACER_TASK_CYC);
}
```

実装時の注意:

- 追加取得を行う処理は、できるだけ `tracer_task` か `trace_logger_task` の周期内に寄せる。
- センサー追加時は「取得」だけでなく「ポート割当」「CSV列追加」「停止時ダンプ整合」まで同時に更新する。
- まず `printf` で値が動くことを確認し、その後CSV列を増やすと切り分けが速い。

---

## 2. appのモータ出力がシミュレータへ反映される経路

結論として、以下の経路で反映される。

1. app側制御ロジックが `pup_motor_set_power(...)` を呼ぶ
2. SPIKE API実装が vdev TX メモリへ書き込む
3. Athrill `vdev_udp` がTX領域をUDP送信
4. 外部シミュレータが受信し、モータモデルへ反映

### 代表的な根拠

- app側モータ出力
  - `raspike-athrill-v850e2m/sdk/workspace/sample_c5_study/LineTracer/LineTracer.c`
    - `pup_motor_set_power(...)`
- API実装（TXメモリ書込み）
  - `raspike-athrill-v850e2m/sdk/common/spikeapi/src/raspike_motor.c`
    - `sil_wrw_mem((uint32_t*)EV3_MOTOR_ADDR_INX(...), power)`
- vdev TXアドレス定義
  - `raspike-athrill-v850e2m/target/v850_gcc/pil/include/ev3_vdev.h`
    - `EV3_MOTOR_ADDR_*` は `VDEV_TX_DATA_BASE` 起点
- Athrill UDP送信処理
  - `athrill-target-v850e2m/src/device/peripheral/vdev/vdev_udp.c`
    - `vdev_udp_put_data8(...)` 内で `udp_comm_remote_write(...)`

---

## 3. シミュレーション実行に必要な設定情報（ファイル整理）

| ファイル | 主な役割 | 変更時の影響 |
|---|---|---|
| `sdk/workspace/<app>/app.c` | 実行フロー（開始・停止・再開） | 実行条件/イベントの挙動が変わる |
| `sdk/workspace/<app>/app.cfg` | RTOSタスク・周期定義 (`CRE_TSK`, `CRE_CYC`) | 実行タイミング・並列構造が変わる |
| `sdk/workspace/<app>/app.h` | 優先度、周期、スタック等の定義 | スケジューリングや周期挙動が変わる |
| `sdk/workspace/<app>/Makefile.inc` | 追加ソース、マクロ、include | 機能有効/無効、ビルド対象が変わる |
| `sdk/common/Makefile.workspace` | ワークスペース全体ビルド統合 | buildフロー、ターゲット解決に影響 |
| `sdk/common/device_config.txt` | Athrill vdev/UDP通信設定 | 通信断・接続不可・IP/Port競合に直結 |
| `sdk/common/memory.txt` | Athrill仮想メモリマップ | メモリアクセス/デバイス反映に影響 |

補足:

- `app.cfg` は `app_common.cfg` を経由して `ev3.cfg` / `ev3api.cfg` へ接続される。
- `device_config.txt` の `DEBUG_FUNC_ENABLE_VDEV`, `*_TX_*`, `*_RX_*` は疎通確認で最優先。

---

## 4. app変更で挙動を変えるために最低限理解すべき流れ

1. `main_task` が開始/停止イベントを管理する
2. `sta_cyc` で周期タスクを開始する
3. `tracer_task` がセンサー読取→演算→モータ出力を行う
4. API層がvdevメモリに反映し、AthrillがUDPで外部simと同期する
5. 停止時に `stp_cyc` と停止処理で制御を終了する

この流れを切り分けると、問題発生時に「appロジック」「RTOS周期」「通信設定」「基盤実装」のどこかを速く特定できる。

---

## 5. `app.c` / `app.cfg` / `app.h` / `Makefile.inc` の役割と影響

### `app.c`

- 役割: 実行フローの司令塔
- 影響: 開始条件、停止条件、どの周期処理を起動するかが決まる

### `app.cfg`

- 役割: RTOSオブジェクト定義
- 影響: タスク構成・周期通知・アクティベーション方式が決まる

### `app.h`

- 役割: 優先度/周期/宣言の共通設定
- 影響: 応答性、タスク競合時の実行順、制御周期が変わる

### `Makefile.inc`

- 役割: アプリ固有ビルド設定
- 影響: 追加モジュール反映、デバッグマクロ有効化、リンク構成が変わる

---

## 6. タスク・周期実行の定義場所と app 実行タイミングへの影響

定義は主に次の2箇所。

1. `app.cfg`
   - `CRE_TSK(...)` でタスク定義
   - `CRE_CYC(...)` で周期ハンドラ定義
2. `app.h`
   - `*_PRIORITY` と `*_PERIOD` の定義

実行開始のトリガは `app.c` の `sta_cyc(...)`。
したがって、定義 (`app.cfg`/`app.h`) と起動条件 (`app.c`) の両方を見ないと挙動は理解できない。

---

## 7. 通常変更してよいファイル vs 基本触らない基盤ファイル

### 通常変更してよい（アプリ開発者領域）

- `sdk/workspace/<app>/app.c`
- `sdk/workspace/<app>/app.cfg`
- `sdk/workspace/<app>/app.h`
- `sdk/workspace/<app>/Makefile.inc`
- `sdk/workspace/<app>/LineTracer/*.c, *.h` など app配下モジュール

### 原則変更しない（基盤領域）

- `sdk/common/spikeapi/src/*`（SPIKE API実装）
- `athrill-target-v850e2m/src/device/peripheral/vdev/*`（vdev実装）
- `kernel/*`, `target/*`（RTOS/ターゲット基盤）

### 必要時のみ変更（影響範囲大）

- `sdk/common/device_config.txt`
- `sdk/common/memory.txt`
- `sdk/common/Makefile.workspace`

---

## 8. 今後の拡張・改善・レベルアップ方針の記載有無

記載あり。

### 主な該当箇所

- `md/SYSTEM_ARCHITECTURE_COMPLETE.md`
  - 「拡張ポイント 今後の改善案」
  - PID制御、IMU統合、ジャイロ、Bluetooth、mROS等の追加案
- `md/sample_c5_study_guide.md`
  - 現状課題、確認項目、改善案（係数調整、出力クリップ、PI/PID拡張）
- `raspike-athrill-v850e2m/libraspike-art/README.md`
  - API未対応項目への将来対応/改善コメント

---

## 9. 追加で確認すべき重要観点（提案）

1. 通信前提の整合
   - `device_config.txt` の VDEV有効化、TX/RX IP・ポートの競合確認
2. 周期設計の妥当性
   - 周期変更が制御性能/負荷/ログ粒度に与える影響を同時評価
3. APIサポート制限の把握
   - シミュレータ環境で未対応APIを設計時に除外
4. ポート割当整合
   - app側ポート定義と環境想定のズレ確認
5. 観測性の確保
   - CSV/ログを先に整えて、変更前後を比較可能にする

---

## 最小チェックリスト（実務向け）

- [ ] `app.c` の開始/停止トリガが意図通りか
- [ ] `app.cfg` の `CRE_CYC` と `app.h` の周期が一致しているか
- [ ] `LineTracer` 等の出力値が飽和/符号反転していないか
- [ ] `device_config.txt` のVDEV TX/RX設定が実行環境に一致しているか
- [ ] `TraceLogger` 等で変更前後の比較データが取れているか
- [ ] 基盤側を触る前に app層で再現条件を切り分けたか
