# hocbf_grasp_sim 初期仕様書

## プロジェクト概要

適応的HOCBF-QPによる力センサレス安全把持制御のC++シミュレータ。研究ロードマップ（`~/lab/exp/ideas/icm_paper/research_roadmap.md`）のStep 2に対応する。

## 動機

- Step 1（理論定式化）で導出したHOCBF-QPの安全保証を数値的に検証する
- 実機実験（Step 3-6）の前にパラメータの妥当性を確認する
- γ_pos, γ_force のチューニング指針を得る
- 固定θ_max vs 適応的θ_max の比較データを取得する

## コア機能

### 2-1. 基本シミュレータ
- 1-DOFハンドの数値シミュレーション（Mθ̈ = u - K_obj × x₁）
- DOBモデル: LPFで τ_ext を推定
- RFOBモデル: K̂_obj の推定（LPF付き）
- 名目制御器（PD位置制御）
- HOCBF-QPの clamp 実装

### 2-2. 固定パラメータでの検証
- 固定 θ_max, 固定 τ_min でCBF-QPが安全集合を維持することの確認
- 卒論の等重み合成との比較
- γ_pos, γ_force のチューニング

### 2-3. 適応的θ_maxのシミュレーション
- K_obj が時変する物体モデル（弾性→塑性遷移）
- RFOB-triggered adaptationで θ_max が適応的に変化
- 固定θ_maxだと物体が壊れるケースを提案手法が防ぐことの確認

### 2-4. Feasibility判定
- G = u_ub - u_lb < 0 になるケースの再現
- G(t) の時系列プロット

### 2-5. τ_minロバスト性
- τ_min_initial を真値の 0.5倍/1.0倍/1.5倍に設定して安全性を確認

## アーキテクチャ

5モジュール分離構成:
- **plant**: 1-DOFハンドモデル + 弾塑性遷移物体モデル
- **observer**: DOB + RFOB
- **controller**: HOCBF-QP安全フィルタ + 名目PD制御
- **adaptive**: θ_max適応則 + τ_min更新 + Graspability Index
- **csv_writer**: CSV出力

## 技術要件

- C++17（実機コードとの整合性）
- JSON設定ファイル（Boost.PropertyTree）
- CSV出力 + Python/Matplotlib可視化
- controller_try の信号処理アルゴリズム（DOB, LPF, 疑似微分）と同一の実装

## 配布方法

配布しない。研究用ローカルコード。

## ターゲットユーザー

著者自身（榎本爽我）。修論・ICM論文のシミュレーション検証に使用。
