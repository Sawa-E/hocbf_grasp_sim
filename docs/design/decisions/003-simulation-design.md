<!--
種別: decisions
対象: シミュレータの設計判断
作成日: 2026-03-24
更新日: 2026-03-24
担当: AIエージェント
-->

# シミュレーション設計

## 概要

hocbf_grasp_sim のシミュレーションモード（`#define SIMULATOR`）での設計判断を定める。controller_try のフレームワーク上にHOCBF-QP把持モードを追加する構成。

## 設計判断

### 判断1: コード構成 — controller_try のモード追加方式

**問題**: HOCBF-QP把持制御のコードをどう組み込むか

**選択肢**:
1. controller.cc に新モード `HOCBF_grasp` をラムダ式で追加（controller_try方式）
2. 5モジュール分離（plant/observer/controller/adaptive/sim）を独立ファイルで作成

**決定**: 選択肢1 をベースに、HOCBF固有のロジックはヘッダファイルに分離

**理由**:
- controller_try の規約（モード追加 = ラムダ式）に従い、GUIボタンが自動生成される
- ただし controller.cc のラムダ式内に全ロジックを書くと巨大化するため、以下のヘッダに分離:
  - `inc/hocbf_qp.h` — HOCBF-QP の u_lb, u_ub, clamp 計算
  - `inc/rfob.h` — RFOB（K̂_obj推定）
  - `inc/adaptive_theta_max.h` — θ_max適応則
  - `inc/contact_detector.h` — 接触検出
  - `inc/object_model.h` — 弾塑性遷移モデル（シミュレーション用）
- controller.cc のラムダ式はこれらを呼び出す薄いラッパー

**構成イメージ**:
```cpp
// mode_definition.h に追加
enum control_mode {
    // ... 既存モード ...
    HOCBF_grasp,       // HOCBF-QP把持制御
    HOCBF_grasp_data,  // データ記録モード
    control_mode_size
};

// controller.cc に追加
controller[mc::HOCBF_grasp] = [](robot_system &robot) {
    // inc/hocbf_qp.h, inc/rfob.h 等を使用
    // フェーズ管理、DOB、RFOB、接触検出、CBF-QP、適応則
};
```

**トレードオフ**:
- **利点**: controller_try と同じ方式、GUIが使える、実機移行が自然
- **欠点**: controller.cc が長くなる（ヘッダ分離で緩和）

### 判断2: シミュレーションモード — controller_try の SIMULATOR define

**問題**: シミュレーションをどう実行するか

**決定**: controller_try の `#define SIMULATOR` をそのまま使用

- `main.cc` の `#ifdef SIMULATOR` ブロックで `simulator<double>` が reader/writer として登録される
- `simulator.h` の `flush()` が前進オイラー法（`ddx = f/M`, `dx += ddx*dt`, `x += dx*dt`）を実行
- PCI I/O の代わりに `simulator` がセンサ読取・出力書込を担当
- **物体モデル**: `simulator.h` を拡張するか、controller.cc のラムダ式内で物体の反力を計算

**トレードオフ**:
- **利点**: 実機コードと全く同じ制御ループが動く
- **欠点**: simulator.h のデフォルトは `ddx = f/M` だけなので、物体の反力モデルを追加する必要がある

### 判断3: 物体モデルの組込み — simulator拡張

**問題**: 弾塑性遷移物体モデル（ADR-004）をシミュレータにどう組み込むか

**選択肢**:
1. simulator.h の flush() を拡張して物体モデルを含める
2. controller.cc のラムダ式内で物体反力を計算し、simulated_robot に書き戻す

**決定**: 選択肢1 — simulator.h を拡張

**理由**:
- 物体モデルは制御器ではなく環境（プラント）の一部
- simulator の flush() 内で `tau_ext = K_obj(theta_rel) * theta_rel` を計算し、`f_sim` に反映
- 制御器（controller.cc）は物体の真のパラメータを知らない（推定値のみ使用）

### 判断4〜6: 時間パラメータ・実行単位・比較手法

変更なし。dt=100μs固定、単一シナリオ、提案手法のみ（旧ADR-003と同一）。

### 判断7: フェーズ構成 — 接近からフルシミュレーション

変更なし。Phase 0→1→2→3（旧ADR-003と同一）。

## CSV出力カラム

変更なし（旧ADR-003と同一）。controller.cc のラムダ式内で fprintf でCSV出力。

| カラム | 変数 | 単位 |
|--------|------|------|
| `t` | シミュレーション時刻 | s |
| `phase` | 現在のフェーズ（0-3） | — |
| `x1` | θ_rel | rad |
| `x2` | θ̇_rel | rad/s |
| `u_nom` | 名目制御入力 | Nm |
| `u_star` | CBF-QP出力 | Nm |
| `u_lb` | HOCBF下界 | Nm |
| `u_ub` | HOCBF上界 | Nm |
| `tau_ext` | 真の外乱トルク | Nm |
| `f_dis_hat` | DOB推定外乱 | Nm |
| `K_obj_true` | 真の物体剛性 | Nm/rad |
| `K_obj_hat` | RFOB推定剛性 | Nm/rad |
| `theta_max` | 適応的θ_max | rad |
| `tau_min` | τ_min | Nm |
| `h_lb` | h₁ = K̂_obj×x₁ - τ_min | Nm |
| `h_ub` | h₂ = θ_max - x₁ | rad |
| `G` | Graspability Index | Nm |

## 関連ドキュメント

- [001-technology-stack.md](./001-technology-stack.md) — controller_try ベース
- [004-object-model.md](./004-object-model.md) — 弾塑性遷移モデル
- [008-contact-detection.md](./008-contact-detection.md) — フェーズ遷移
- `~/lab/controller_try/inc/simulator.h` — シミュレータの flush()
- `~/lab/controller_try/src/controller.cc` — モード追加の方式
