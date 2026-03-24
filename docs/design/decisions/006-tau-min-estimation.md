<!--
種別: decisions
対象: τ_min推定パイプラインのアーキテクチャ設計
作成日: 2026-03-24
更新日: 2026-03-24
担当: AIエージェント
-->

# τ_min推定パイプライン設計

## 概要

別リポジトリ `~/lab/tau_min_estimation/` として構築する τ_min推定パイプラインの技術スタック、2段階の実行タイミング、controller_try との連携方式を定める。

## 全体構成

```
RealSense D435i
     │
     ▼
tau_min_estimation (Python)
     │
     ├── 段階①：把持前（1回）
     │     RGB-D → YOLOv8 → Depth体積 → m_est → τ_min_initial
     │     → JSON出力
     │
     └── 段階②：把持中（リアルタイム連続）
           Depth重心追跡 → スリップ判定 → フラグ出力
           → controller_try が読み取り τ_min 引き上げ
```

## 設計判断

### 判断1: 技術スタック — Python + YOLOv8 + OpenCV + pyrealsense2

**問題**: τ_min推定パイプラインの技術スタックをどうするか

**選択肢**:
1. Python + YOLOv8 (ultralytics) + OpenCV + pyrealsense2
2. Python + 独自検出（OpenCVのみ、YOLOv8なし）
3. C++（RealSense SDK + OpenCV C++）

**決定**: 選択肢1 — Python + YOLOv8 + OpenCV + pyrealsense2

**理由**:
- YOLOv8 は ultralytics パッケージで数行で fine-tune・推論可能
- pyrealsense2 は Python バインディングが公式提供されており、RGB-D取得が容易
- OpenCV の Sobel フィルタで液面検出を実装（tau_min_estimation.md §3.3 の手法）
- Pythonなのでプロトタイピングが高速。推論速度は把持前1回なので問題なし

**トレードオフ**:
- **利点**: 開発速度、YOLOv8エコシステム、OpenCVの画像処理が豊富
- **欠点**: C++の controller_try とは別プロセス（連携はファイル/フラグ経由）

**依存ライブラリ**:
```
ultralytics       # YOLOv8
opencv-python     # 画像処理・液面検出
pyrealsense2      # RealSense D435i
numpy             # 数値計算
```

### 判断2: 実行タイミング — 把持前1回 + 把持中リアルタイム連続

**問題**: 推定パイプラインの実行タイミングをどうするか

**選択肢**:
1. 把持前に1回だけ τ_min_initial を推定
2. 把持前1回（τ_min_initial）+ 把持中リアルタイム（スリップ検出）
3. 全てリアルタイム連続

**決定**: 選択肢2 — 把持前1回 + 把持中リアルタイム連続

**理由**:
- step1_formulation.md の段階①②と完全に一致する設計
- 段階①（把持前）: RGB-D → YOLOv8 → 質量推定 → τ_min_initial。1回の推論で十分
- 段階②（把持中）: Depth重心追跡でスリップ検出。カメラのフレームレート（30fps）で連続処理
- 段階①と②は性質が異なる（①は重い推論、②は軽い追跡）が、同一プロセス内でモード切替え可能

**プロセスのライフサイクル**:
```
1. 起動 → カメラストリーム開始
2. 把持前モード:
   - YOLOv8で物体検出・分類
   - Depth → V_max推定
   - PETボトルなら液面検出 → R_fill
   - m_est → τ_min_initial を JSON出力
   - 「把持開始」シグナルを待つ
3. 把持中モード:
   - Depth重心のy座標を毎フレーム追跡
   - 移動平均でノイズ除去
   - Δy_mm > δ_slip_mm → slip_detected フラグを出力
4. 「把持終了」シグナルで再び把持前モードへ
```

**トレードオフ**:
- **利点**: τ_min の初期推定とリアルタイム補正の両方をカバー、step1_formulation.md と整合
- **欠点**: 把持中モードではカメラが物体を常に見える位置にある必要がある

### 判断3: controller_try との連携 — 共有ファイル/フラグ方式

**問題**: Python プロセス（tau_min_estimation）と C++ プロセス（controller_try）の間でデータをどう受け渡すか

**選択肢**:
1. 共有ファイル（JSON + フラグファイル）
2. ZeroMQ などのメッセージングライブラリ
3. 共有メモリ（POSIX shm）

**決定**: 選択肢1 — 共有ファイル方式

**理由**:
- step1_formulation.md の「フラグ方式」と一致
- controller_try の制御ループ（10kHz）がファイルを読むオーバーヘッドは無視できる（stat() + read() で数μs）
- 追加ライブラリ不要。Python は open/write、C++ は fopen/fread だけ
- デバッグ時にファイルの中身を直接確認・手動編集できる

**インターフェース仕様**:

段階①の出力（把持前、1回）:
```
/tmp/tau_min_estimation/tau_min_initial.json
{
  "tau_min_initial": 0.025,
  "m_est": 0.30,
  "category": "pet_bottle",
  "r_fill": 0.6,
  "v_max_ml": 500,
  "confidence": 0.92,
  "timestamp": "2026-03-24T10:30:00"
}
```

段階②の出力（把持中、リアルタイム）:
```
/tmp/tau_min_estimation/slip_flag
```
- ファイルが存在する → slip_detected = true
- controller_try が読み取り後に削除 → フラグリセット
- Python は検出のたびにファイルを作成

**トレードオフ**:
- **利点**: 実装が極めて単純、デバッグ容易、追加依存なし
- **欠点**: レイテンシ（ファイルI/O）。ただし30fps（33ms間隔）のスリップ検出と10kHz制御ループの間では問題にならない

### 判断4: リポジトリ構造

**決定**:
```
~/lab/tau_min_estimation/
├── README.md
├── requirements.txt
├── config/
│   ├── camera.json           # RealSense設定（解像度、fps等）
│   ├── categories.json       # カテゴリ別パラメータ（a, b, η, default_R_fill）
│   └── slip_detection.json   # スリップ検出パラメータ（δ_slip_mm, β等）
├── src/
│   ├── main.py               # エントリポイント（モード管理）
│   ├── detector.py           # YOLOv8カテゴリ分類
│   ├── depth_volume.py       # Depth → V_max推定
│   ├── liquid_level.py       # 液面検出（Sobelエッジベース）
│   ├── estimator.py          # 統合: m_est → τ_min_initial
│   └── slip_tracker.py       # Depth重心追跡 → スリップ検出
├── models/                   # YOLOv8学習済みモデル
├── data/
│   ├── calibration/          # m_empty校正データ
│   └── training/             # YOLOv8学習データ
└── scripts/
    └── calibrate.py          # m_empty線形モデル校正スクリプト
```

### 判断5: 質量推定の物理モデル

**決定**: tau_min_estimation.md §3 の物理モデルをそのまま採用

```
m_est = m_empty + ρ_liquid × V_max × η × R_fill
m_empty = a × V_max + b    （カテゴリ別線形モデル）
τ_min_initial = (m_est / 1000) × g × r_eff / (2μ)
```

| パラメータ | 値 | 決定方法 |
|-----------|-----|---------|
| r_eff | 0.0725 m | 実測（ハンド機構の設計値） |
| μ_assumed | 0.3 | 保守的に低め設定 |
| ρ_liquid | 1.0 g/mL | 水を仮定 |
| default_R_fill | 0.1 | 低めに設定（過小推定→スリップ検出が補正） |

カテゴリ別パラメータ（a, b, η）は校正データから線形回帰で決定。

**低めに推定する設計思想**:
- 過大推定 → 強く握りすぎ → 柔らかい物体で G < 0（把持不可能の誤判定）
- 過小推定 → 弱く握る → スリップ → カメラベーススリップ検出が補正
- **低めに外す方が安全側**

## 関連ドキュメント

- [002-system-architecture.md](./002-system-architecture.md) — 3リポジトリ構成、JSONインターフェース
- [005-safety-parameters.md](./005-safety-parameters.md) — τ_min の制御側での扱い
- `~/lab/exp/ideas/icm_paper/tau_min_estimation.md` — 理論・実装の詳細ガイド
- `~/lab/exp/ideas/icm_paper/step1_formulation.md` §1-4 — 段階①②の理論定式化
- `~/lab/exp/ideas/icm_paper/icm_paper_plan.md` — 画像推定の実装作業一覧
