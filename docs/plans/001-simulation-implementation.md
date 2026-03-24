<!--
種別: enhancement
優先度: 高
作成日: 2026-03-24
更新日: 2026-03-24
担当: AIエージェント
-->

# hocbf_grasp_sim 実装計画

## 概要

controller_try をベースにコピーし、HOCBF-QP把持制御モード（`HOCBF_grasp`）を追加する。`#define SIMULATOR` でシミュレーション、`#define PCI_MODE` で実機として動作する。

## スコープ

### 対応範囲
- controller_try のコードをコピーしてベースを構築
- HOCBF-QP固有のヘッダ群を新規作成（hocbf_qp.h, rfob.h, adaptive_theta_max.h, contact_detector.h, object_model.h）
- controller.cc に `HOCBF_grasp` モードをラムダ式で追加
- simulator.h を拡張して弾塑性遷移物体モデルを組込
- config JSON（物体パラメータ）とPythonプロットスクリプト
- Phase A（推定誤差なし）の動作確認まで

### 対応外
- tau_min_estimation リポジトリ（別計画）
- Phase B/C（RFOB遅延あり・ロバスト版）
- 論文執筆（Step 7）

## 依存関係図

```
#001-01 controller_tryコピー + ビルド確認
    ↓
#001-02 mode_definition + joint config
    ↓
#001-03 object_model.h (simulator拡張)  ←──── #001-07 config/objects JSON
    ↓
#001-04 hocbf_qp.h + rfob.h + contact_detector.h + adaptive_theta_max.h
    ↓
#001-05 controller.cc に HOCBF_grasp モード実装
    ↓
#001-06 CSV出力 + Python plot.py
    ↓
#001-08 統合テスト・チューニング
```

## タスク一覧

---

### #001-01: controller_try コピー + ビルド確認

**対象**: プロジェクト全体

**実装内容**:
- `~/lab/controller_try/` の以下をコピー:
  - `CMakeLists.txt`, `src/`, `inc/`, `lib/`, `config/`, `scripts/`
- `.claude/` と `docs/` は維持（上書きしない）
- CMakeLists.txt で `add_definitions(-DSIMULATOR)` を有効化（`-DPCI_MODE` をコメントアウト）
- `cmake .. && make` でビルドが通ることを確認
- 既存モード（idle等）がシミュレーションモードで動作することを確認

**完了条件**: `./control` が起動し、GUIが表示される

---

### #001-02: mode_definition + 関節設定

**対象ファイル**: `inc/mode_definition.h`, `src/main.cc`, `config/`

**実装内容**:
- `mode_definition.h` に新モード追加:
  ```cpp
  HOCBF_grasp,
  HOCBF_grasp_data,
  ```
- `main.cc` の関節登録を把持ハンド用に調整:
  - 把持に使うモータ（4018_finger）2軸の設定
  - シミュレータモードの virtual_robot も同様に設定
- 必要に応じて新しい joint config JSON を作成

**完了条件**: GUIに `HOCBF_grasp` ボタンが表示される

---

### #001-03: object_model.h + simulator 拡張

**対象ファイル**: `inc/object_model.h`, `inc/simulator.h`

**実装内容**:
- `object_model.h`: 弾塑性遷移モデル（ヘッダオンリー）
  ```cpp
  class object_model {
      double K_0_, theta_yield_, beta_, theta_contact_;
  public:
      double compute_K_obj(double theta);      // 弾塑性遷移
      double compute_tau_ext(double theta);     // K_obj × theta_rel
      double get_K_obj_true();                  // 検証用
  };
  ```
- `simulator.h` の `flush()` を拡張:
  - 物体モデルの反力 τ_ext を計算
  - `f_sim = u - tau_ext` → `ddx = f_sim / M` → オイラー積分
- 物体パラメータは JSON から読込（#001-07 で作成）

**完了条件**: シミュレーションで物体接触時に反力が発生する

---

### #001-04: HOCBF-QP 関連ヘッダ群

**対象ファイル**: `inc/hocbf_qp.h`, `inc/rfob.h`, `inc/contact_detector.h`, `inc/adaptive_theta_max.h`

**実装内容**:

**hocbf_qp.h** — HOCBF-QP安全フィルタ:
- `compute_u_ub(x1, x2, K_hat, theta_max, theta_max_dot, M, gamma_pos)` → u_ub
- `compute_u_lb(x1, x2, K_hat, tau_min, M, gamma_force)` → u_lb
- `compute_u_star(u_nom, u_lb, u_ub)` → clamp
- `compute_G(u_ub, u_lb)` → Graspability Index
- 数式は step1_formulation.md §1-3 そのまま

**rfob.h** — RFOB:
- `update(f_dis_hat, theta_rel, dt, g_rfob)` → K_obj_hat
- controller_try の signal_processing.h のLPFを内部で使用

**contact_detector.h** — 接触検出:
- リングバッファで移動平均・標準偏差
- 条件A (DOB外乱) AND 条件B (K̂_obj) の二重判定
- ADR-008 のアルゴリズム

**adaptive_theta_max.h** — θ_max適応則:
- K̂_obj 変化率の判定（ε, ε_danger）
- θ_max 更新（α_relax, α_shrink）
- α_shrink の動的上界制限
- ADR-005 のパラメータ

**完了条件**: 各ヘッダが単独でコンパイル可能

---

### #001-05: controller.cc に HOCBF_grasp モード実装

**対象ファイル**: `src/controller.cc`

**実装内容**:
- `controller[mc::HOCBF_grasp]` ラムダ式:
  ```
  Phase 0: PD位置制御、DOB動作、接触検出監視
  接触検出 → θ_hold記録、Phase 2へ
  Phase 2: CBF-QP有効、RFOB、適応則、高周波注入
  Phase 3: 定常保持（Phase 2と同ロジック、ログ区別のみ）
  ```
- static 変数でフェーズ状態・θ_hold・θ_max等を保持（controller_try の他モードと同じ方式）
- #001-04 のヘッダを全て呼び出す
- controller_try の既存マクロ（`x_res(n)`, `f_out(n)`, `M(n)` 等）を使用

**完了条件**: HOCBF_grasp モードで把持シミュレーションが動作する

---

### #001-06: CSV出力 + Python plot.py

**対象ファイル**: `src/controller.cc`（出力部分）, `scripts/plot.py`

**実装内容**:
- `controller[mc::HOCBF_grasp_data]` モード:
  - controller_try の Record モードと同様に fprintf で CSV 出力
  - ADR-003 の17カラム仕様
  - HOCBF_grasp モードで動作した後にこのモードに切り替えてデータ保存
  - または HOCBF_grasp 内に直接 fprintf を埋め込む
- `scripts/plot.py`:
  - CSV読込 → 4パネル図
  - フェーズ境界を縦線で表示
  - `~/lab/exp/cbf_qp_figures.py` のスタイル参考

**完了条件**: CSV出力 → plot.py でプロットが生成される

---

### #001-07: config JSON（物体パラメータ）

**対象ファイル**: `config/objects/*.json`

**実装内容**:
- ADR-004 のパラメータ表に基づく5物体のJSON:
  - `aluminum_can.json`, `pet_bottle_full.json`, `pet_bottle_empty.json`, `paper_cup_empty.json`, `paper_cup_full.json`
- HOCBF-QPパラメータのJSON（γ_pos, γ_force, 適応則パラメータ等）
- object_model.h が読み込める形式

**完了条件**: 5物体分のJSONが揃い、object_model が読込可能

---

### #001-08: 統合テスト・パラメータチューニング

**実装内容**:
- PETボトル（水入り）で基本動作確認:
  - [ ] Phase 0→2 の遷移が正常に発生
  - [ ] u_lb ≤ u* ≤ u_ub（CBF-QP）
  - [ ] h_lb(t) ≥ 0 かつ h_ub(t) ≥ 0（安全集合の前方不変性）
  - [ ] G(t) > 0（実行可能性）
  - [ ] K̂_obj が K_obj_true に追従
  - [ ] θ_max(t) が適応的に変化
- 5物体全てで実行:
  - [ ] 空アルミ缶: θ_max が θ_max_ceiling まで緩和
  - [ ] PETボトル空: K_obj急変で θ_max 縮小
  - [ ] 紙コップ空: θ_max がほとんど緩和しない
  - [ ] 紙コップ水入り: G(t) が小さくなることを確認
- γ_pos, γ_force のチューニング

**完了条件**: 全物体で安全集合の前方不変性が成立、プロットが論文に使える品質

## マイルストーン

| マイルストーン | タスク | 完了条件 |
|--------------|--------|---------|
| **M1: ビルド通る** | #001-01 | controller_try ベースで `make` 成功、GUI起動 |
| **M2: 自由空間で動く** | #001-02, 03 | HOCBF_grasp モードで Phase 0（PD制御）が動作 |
| **M3: 把持が動く** | #001-04, 05 | Phase 0→2 遷移。CBF-QP有効。プロット可能 |
| **M4: 全機能統合** | #001-06, 07 | CSV出力、5物体JSON、プロット完成 |
| **M5: 検証完了** | #001-08 | 全物体で安全性確認、γチューニング完了 |

## 関連ドキュメント

- [ADR 001-008](../design/decisions/) — 全設計判断
- [初期仕様書](../archive/initial_plan.md) — Step 2 の検証シナリオ
- `~/lab/controller_try/` — ベースフレームワーク
- `~/lab/exp/ideas/icm_paper/step1_formulation.md` — HOCBF-QP理論定式化
