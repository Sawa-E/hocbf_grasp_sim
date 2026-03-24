<!--
種別: decisions
対象: 研究全体のシステム構成と3リポジトリの役割分担
作成日: 2026-03-24
更新日: 2026-03-24
担当: AIエージェント
-->

# 研究全体のシステム構成

## 概要

修論・IEEE ICM論文に向けた研究システム全体を、3つのリポジトリに分割し、それぞれの役割・インターフェース・開発方針を定める。

## 全体像

```
~/lab/
├── hocbf_grasp_sim/       C++  controller_tryベースのHOCBF-QP把持制御
│                               シミュレーション(SIMULATOR)でも実機(PCI_MODE)でも動く
├── tau_min_estimation/     Python  τ_min推定パイプライン（Step 5と並行）
├── controller_try/         C++  研究室共有フレームワーク（参照元・変更しない）
└── exp/                    研究資料・理論・論文（参照のみ）
```

```
  controller_try（研究室共有、読み取り専用）
        │
        │ コピーしてベースに使用
        ▼
  hocbf_grasp_sim（HOCBF-QP把持制御を追加）
        │
        ├── #define SIMULATOR → シミュレーションモード（Step 2）
        └── #define PCI_MODE  → 実機モード（Step 3-6）
                                    ▲
                                    │ JSON経由で τ_min_initial を受け取る
                                    │
                          tau_min_estimation（Python）
```

## 設計判断

### 判断1: hocbf_grasp_sim の位置づけ — controller_try のフォーク

**問題**: HOCBF-QP把持制御コードをどこに作るか

**選択肢**:
1. controller_try をベースにコピーし、hocbf_grasp_sim として独立管理。ここにHOCBF-QPを実装
2. 純粋なシミュレータを新規作成し、後で controller_try に移植
3. controller_try に直接追加

**決定**: 選択肢1 — controller_try をベースにフォーク

**理由**:
- controller_try のフレームワーク（robot_system, joint, signal_processing, thread_controller, GUI, #define SIMULATOR）がそのまま使える
- `mode_definition.h` に新モードを追加するだけで GUI にボタンが自動生成される
- シミュレーションモードと実機モードが CMake の `#define` で切り替えられる
- controller_try は研究室共有コードなので直接変更しない（選択肢3は不採用）
- 純粋なシミュレータ新規作成（選択肢2）は、フレームワークの再実装が必要で非効率

**トレードオフ**:
- **利点**: フレームワーク再実装ゼロ、シミュ/実機を1コードで、GUI付き
- **欠点**: controller_try の不要コード（バイラテラル等）も含まれる。controller_try の上流更新は手動マージ

### 判断2: リポジトリ間インターフェース — JSONファイル経由

**決定**: ADR-006 で定めた通り。tau_min_estimation が τ_min_initial を JSON出力、hocbf_grasp_sim が config として読込。

### 判断3: controller_try との関係 — 参照元として読み取り専用

**決定**: controller_try は変更しない。hocbf_grasp_sim は controller_try の独立コピーとして管理。

### 判断4: 各リポジトリの責務

| リポジトリ | 責務 | 言語 | 研究ステップ |
|-----------|------|------|------------|
| **hocbf_grasp_sim** | HOCBF-QP把持制御の実装・シミュレーション検証・実機実験 | C++17 | Step 2-6 |
| **tau_min_estimation** | 画像→質量推定→τ_min_initial 推論パイプライン | Python3 | Step 5 並行 |
| **controller_try** | 研究室共有フレームワーク。参照元。変更しない | C++17 | — |
| **exp** | 理論定式化、論文、アイデア、参考文献 | — | 全ステップ |

## 関連ドキュメント

- [001-technology-stack.md](./001-technology-stack.md) — controller_try ベースの技術スタック
- [006-tau-min-estimation.md](./006-tau-min-estimation.md) — τ_min推定パイプライン・JSON連携
- `~/lab/exp/ideas/icm_paper/research_roadmap.md` — 研究全体のStep 1-7
