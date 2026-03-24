<!--
種別: decisions
対象: オブザーバ（DOB/RFOB）の設計
作成日: 2026-03-24
更新日: 2026-03-24
担当: AIエージェント
-->

# オブザーバ設計

## 概要

DOB（外乱オブザーバ）とRFOB（反力オブザーバ）の実装方式、K̂_obj 推定の具体的な計算手順、高周波注入の設計を定める。

## DOB の実装

controller_try の `signal_processing.h` と同一のアルゴリズムを使用する。

### 離散時間実装（前進オイラー）

```cpp
// DOB: 外乱トルク f̂_dis を推定
// f_out: 制御出力（トルク指令 u）
// dx: 角速度 θ̇（疑似微分で取得）
// M: 慣性モーメント
// g_dis: DOBカットオフ周波数 [rad/s]

f_dis = buf - g_dis * M * dx;
buf += dt * g_dis * (f_out + M * g_dis * dx - buf);
```

### 疑似微分（角速度の推定）

```cpp
// 擬似微分: エンコーダ位置 θ → 角速度 θ̇
// g_diff: カットオフ周波数 [rad/s]

buf += dt * output;
output = g_diff * (input - buf);
```

## 設計判断

### 判断1: RFOB実装方式 — DOB外乱÷変位で K̂_obj 推定

**問題**: 物体の接触剛性 K̂_obj をどう推定するか

**選択肢**:
1. DOB推定外乱を接触後の変位で割る: K̂_obj = f̂_dis / θ_rel（LPF付き）
2. 逐次最小二乗法（controller_try の screw_motion モードの重回帰分析を流用）
3. 周波数応答ベース（高周波注入の応答からインピーダンスを推定、Kitamura(2025)の精密手法）

**決定**: 選択肢1 — DOB外乱÷変位

**理由**:
- 最も単純で計算コストが低い（割り算1回 + LPF）
- 1-DOF・線形接触モデル（τ_ext = K_obj × θ_rel）の仮定下では理論的に正確
- Kitamura(2025) も同モータで同方式を使用しており、実績がある
- 逐次最小二乗法は K, D, M, C の4パラメータを同時推定するが、本研究では K_obj のみ必要

**実装**:
```cpp
// 接触後のみ計算（θ_rel > 0）
if (theta_rel > theta_rel_min) {
    double K_raw = f_dis_hat / theta_rel;
    // LPFで平滑化
    K_obj_hat_buf += dt * (K_raw - K_obj_hat);
    K_obj_hat = g_rfob * K_obj_hat_buf;
}
```

- `theta_rel_min`: ゼロ割り防止の最小変位（例: 0.001 rad）
- `g_rfob`: RFOB のLPFカットオフ周波数（DOBより低く設定）

**トレードオフ**:
- **利点**: 単純、高速、Kitamura論文と整合
- **欠点**: D（粘性）や静摩擦の影響を無視。弾性域では K_obj ≈ 定数なので問題ないが、塑性域移行時は一時的に誤差が大きくなる

### 判断2: 高周波注入 — シミュレーションでも実装

**問題**: RFOBの持続的励起のための高周波注入をシミュレーションで実装するか

**選択肢**:
1. シミュレーションでも実装（実機と同じ構成を再現）
2. シミュレーションでは省略（Phase A では K̂_obj = K_obj、Phase B では注入なしで推定可能か確認）

**決定**: 選択肢1 — シミュレーションでも実装

**理由**:
- 実機と同じ構成を再現し、注入が制御性能・安全性に与える影響を検証する
- 注入トルクが u_lb / u_ub のマージンにどう影響するかを事前に確認できる
- controller_try の screw_motion モードに既に注入の実装がある（正弦波の重ね合わせ）

**注入信号の設計**:

controller_try の screw_motion モードから:
```cpp
f_noise = f_n_amp * (sin(2π × 50 × t) + sin(2π × 60 × t) +
                     sin(2π × 70 × t) + sin(2π × 80 × t) +
                     sin(2π × 90 × t)) / 5.0;
```

5つの正弦波（50-90Hz）の重ね合わせ。振幅 `f_n_amp` は適応的に調整される。

**シミュレーション用の簡略化**:
```
u_inject = A_inject × sin(2π × f_inject × t)
```
- A_inject: 注入振幅 [Nm]。安全余裕 G の一部を消費する
- f_inject: 注入周波数 [Hz]。DOBのカットオフ（600/2π ≈ 95Hz）より低く設定

初期値:
```
A_inject = 0.01 [Nm]   （力制限の1%程度）
f_inject = 50 [Hz]      （controller_try と同等）
```

制約: 注入による u の変動が安全余裕を食いつぶさないこと:
```
A_inject < G_min / 2
```

**トレードオフ**:
- **利点**: 実機と同一構成、注入の影響を事前評価、RFOB推定精度の検証
- **欠点**: Phase A（K̂_obj=K_obj）では注入は意味がない（だがPhase Bで必要になる）

### 判断3: オブザーバのフィルタゲイン設計

**問題**: DOB、疑似微分、RFOBの各フィルタゲインをどう設定するか

**決定**: controller_try の実機パラメータを基本とし、RFOBのみ新規設計

| フィルタ | ゲイン | 値 | 根拠 |
|---------|--------|-----|------|
| 疑似微分 | g_diff | 600 rad/s | controller_try 4018_finger.json |
| DOB | g_dis | 600 rad/s | 同上 |
| RFOB (LPF) | g_rfob | 100 rad/s | DOBより低く設定（下記参照） |

**g_rfob を DOB より低く設定する理由**:
```
DOB (g_dis=600): 外乱トルクの推定。高速応答が必要
RFOB (g_rfob=100): K̂_obj = f̂_dis / θ_rel の除算結果を平滑化。
  - 除算はノイズを増幅するため、強いLPFが必要
  - K_obj の変化速度は物体の塑性変形速度に依存し、数Hz程度
  - g_rfob = 100 rad/s ≈ 16Hz のカットオフで十分
```

**帯域の階層**:
```
制御ループ:     10kHz (dt=100μs)
疑似微分:       ~95Hz  (g_diff=600)
DOB:           ~95Hz  (g_dis=600)
RFOB (LPF):    ~16Hz  (g_rfob=100)
適応則 (ε):     ~数Hz   (K̂_obj変化率の判定)
```

## 関連ドキュメント

- [003-simulation-design.md](./003-simulation-design.md) — 前進オイラー法、dt=100μs
- [005-safety-parameters.md](./005-safety-parameters.md) — g_dis, g_diff, g_rfob のパラメータ値
- `~/lab/controller_try/inc/signal_processing.h` — DOB・LPF・疑似微分の実装
- `~/lab/controller_try/src/controller.cc` screw_motion モード — 高周波注入・重回帰分析
- `~/lab/controller_try/config/blue_scara/4018_finger.json` — g_dis=600, g_diff=600
- `~/lab/exp/thesis/senior/kitamura_2025_adaptive_position_force_control.pdf` — RFOB実装の参考
