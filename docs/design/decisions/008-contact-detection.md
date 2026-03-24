<!--
種別: decisions
対象: 接触検出の設計
作成日: 2026-03-24
更新日: 2026-03-24
担当: AIエージェント
-->

# 接触検出設計

## 概要

把持シミュレーションの Phase 0（接近）→ Phase 1（接触検出）→ Phase 2（把持）の遷移条件と、接触検出アルゴリズムを定める。

## 設計判断

### 判断1: 検出条件 — DOB外乱の適応的閾値 AND K̂_obj 閾値の二重条件

**問題**: 物体への接触をどう検出するか

**選択肢**:
1. DOB外乱の適応的閾値（移動平均+k×標準偏差）AND K̂_obj > K_contact の二重条件
2. DOB外乱の固定閾値のみ

**決定**: 選択肢1 — 二重条件AND

**理由**:
- 条件A単独では、高周波注入や振動によるDOB外乱の変動で誤検出するリスクがある
- 条件B単独では、K̂_obj は RFOB の LPF を通るため応答が遅く、接触初期に反応しない
- AND条件にすることで「普段と違う外乱がある」かつ「剛性が存在する」を同時に確認し、誤検出を防ぐ
- icm_paper_plan.md（Step 4-2）の計画と一致

**条件A: DOB外乱の適応的閾値**

```
接触判定 = f̂_dis > μ_dis + k_sigma × σ_dis
```

- μ_dis: f̂_dis の移動平均（直近 N サンプル）
- σ_dis: f̂_dis の移動標準偏差（同）
- k_sigma: 感度パラメータ（大きいほど鈍感）

初期値:
```
N = 1000 サンプル = 0.1秒分（dt=100μs）
k_sigma = 3.0（3σルール）
```

**条件B: K̂_obj 閾値**

```
接触判定 = K̂_obj > K_contact_threshold
```

- K_contact_threshold: 自由空間の K̂_obj ノイズレベルより十分大きい値

初期値:
```
K_contact_threshold = 0.05 [Nm/rad]
```

根拠: 自由空間では K̂_obj ≈ 0（物体がない）。最も柔らかい紙コップ（K_0=0.2）の1/4程度に設定。

**最終判定**:
```cpp
bool contact_A = (f_dis_hat > mu_dis + k_sigma * sigma_dis);
bool contact_B = (K_obj_hat > K_contact_threshold);
bool contact_detected = contact_A && contact_B;
```

**トレードオフ**:
- **利点**: 誤検出が少ない、物理的に意味のある2条件
- **欠点**: 条件Bは RFOB の応答遅延（g_rfob=100 → ~16Hz）があるため、接触検出に数十ms遅れる可能性

### 判断2: フェーズ遷移方式 — 不可逆

**問題**: 接触検出後のフェーズ遷移を不可逆にするか可逆にするか

**選択肢**:
1. 不可逆（接触検出 → Phase 2 に遷移し、Phase 0 に戻らない）
2. 可逆（外乱が消えたら Phase 0 に戻る）

**決定**: 選択肢1 — 不可逆

**理由**:
- 実機の把持制御も同じ構成（一度把持を開始したら制御モードは戻さない）
- Phase 2 で CBF-QP を有効化した後に Phase 0 に戻すと、安全集合の初期化が必要になり複雑化する
- 「触れたが離れた」ケースは把持失敗であり、安全離脱（G < 0 プロトコル）で処理すべき

**トレードオフ**:
- **利点**: 状態管理が単純、安全集合の一貫性が保たれる
- **欠点**: 接触の誤検出があった場合に自動復帰できない（だが二重条件で誤検出は十分低い）

## フェーズ遷移の全体像

```
Phase 0: 接近
  ├── PD位置制御（u_nom のみ、CBF-QP 無効）
  ├── DOB・疑似微分は動作中
  ├── RFOB は θ_rel > 0 でないため K̂_obj ≈ 0
  ├── 移動平均 μ_dis, 標準偏差 σ_dis を蓄積
  └── 条件A AND 条件B が成立 → 接触検出

     ┃ 接触検出（不可逆遷移）
     ┃ θ_hold = θ_res（接触時の角度を記録）
     ▼

Phase 1: 接触確認（瞬間的、1ステップ）
  ├── θ_hold を記録
  ├── x₁ = θ - θ_hold = 0 から開始
  ├── θ_max(0) = θ_max_0 = 0.005 rad
  ├── τ_min = τ_min_initial（config値）
  └── 即座に Phase 2 へ

     ┃
     ▼

Phase 2: 把持
  ├── CBF-QP 有効: u* = clamp(u_nom, u_lb, u_ub)
  ├── RFOB-triggered adaptation 開始
  ├── θ_max(t) 適応更新
  ├── Graspability Index G(t) モニタリング
  └── x₁ が安定 → Phase 3 へ（or 継続）

     ┃ x₂ ≈ 0 かつ θ_max 安定
     ▼

Phase 3: 保持（定常状態）
  ├── CBF-QP 継続
  ├── 適応則は α_relax モード（緩和のみ）
  └── G(t), h_lb, h_ub のモニタリング継続
```

**Phase 2 → Phase 3 の遷移条件**:
```
|x₂| < x2_threshold  AND  |θ̇_max| < theta_max_dot_threshold
```
- x2_threshold = 0.01 rad/s（ほぼ静止）
- theta_max_dot_threshold = 0.01 rad/s（適応が落ち着いた）

Phase 3 は実質的に Phase 2 の特殊ケース（全て同じロジックが動作、ログ上の区別のみ）。

## 接触検出のパラメータ一覧

| パラメータ | 値 | 単位 | 根拠 |
|-----------|-----|------|------|
| N（移動平均窓） | 1000 | サンプル | 0.1秒分 |
| k_sigma | 3.0 | — | 3σルール |
| K_contact_threshold | 0.05 | Nm/rad | 紙コップK_0の1/4 |
| x2_threshold | 0.01 | rad/s | Phase 3 遷移 |
| theta_max_dot_threshold | 0.01 | rad/s | Phase 3 遷移 |

## 関連ドキュメント

- [003-simulation-design.md](./003-simulation-design.md) — 4フェーズ構成の定義
- [005-safety-parameters.md](./005-safety-parameters.md) — θ_max_0, τ_min_initial
- [007-observer-design.md](./007-observer-design.md) — DOB・RFOB のフィルタゲイン
- `~/lab/exp/ideas/icm_paper/icm_paper_plan.md` — Step 4-2 接触検出の実装計画
