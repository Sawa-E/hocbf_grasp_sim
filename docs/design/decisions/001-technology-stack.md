<!--
種別: decisions
対象: 技術スタック選定
作成日: 2026-03-24
更新日: 2026-03-24
担当: AIエージェント
-->

# 技術スタック選定

## 概要

hocbf_grasp_sim の技術スタックを選定する。controller_try をベースフレームワークとして使用する。

## 設計判断

### 判断1: ベースフレームワーク — controller_try をコピーして拡張

**問題**: プロジェクトのコードベースをどう構築するか

**選択肢**:
1. controller_try をベースにコピーし、HOCBF-QP把持モードを追加
2. ゼロから5モジュール分離のシミュレータを新規作成
3. controller_try に直接コードを追加

**決定**: 選択肢1 — controller_try をベースにコピーして拡張

**理由**:
- controller_try のフレームワーク（robot_system, joint, signal_processing, thread_controller, GUI等）がそのまま使える
- `#define SIMULATOR` で PCI なしシミュレーションが可能（controller_try に既にある）
- `#define PCI_MODE` で実機でもそのまま動作する
- 新モード `HOCBF_grasp` を `mode_definition.h` に追加し、`controller.cc` にラムダ式で実装するだけ
- 選択肢3は研究室共有コードに直接手を入れるリスクがある

**トレードオフ**:
- **利点**: フレームワーク実装ゼロ、実機互換、GUI付き、DOBが動作済み
- **欠点**: 不要なコード（バイラテラル制御等）も含まれる。ただし害はない

### 判断2: 言語 — C++17

**決定**: C++17（controller_try と同一）

### 判断3: パラメータ管理 — JSON (Boost.PropertyTree)

**決定**: controller_try と同一方式。config/ のJSON + `joint.hpp` の自動読込。
物体パラメータは `robot.set_to_dict()` / `robot.get_from_dict()` で管理するか、新規JSONファイルで管理。

### 判断4: 可視化 — CSV出力 + Python/Matplotlib

**決定**: controller_try の Record モード（fprintf → CSV）と同じ方式。
加えて `scripts/plot.py` で論文品質のプロット生成。

### 判断5: ビルド — CMake + 既存依存

**決定**: controller_try の CMakeLists.txt をベースに使用。

依存ライブラリ（controller_try と同一）:
- Boost（PropertyTree）
- OpenGL / GLFW / GLEW（GUI）
- PCI utils（`#ifdef PCI_MODE` のみ）
- Eigen（lib/ に同梱済み）
- imgui（lib/ に同梱済み）

## 技術スタック一覧

| レイヤー | 技術 | 用途 |
|---------|------|------|
| ベース | controller_try フレームワーク | robot_system, joint, signal_processing, GUI等 |
| 言語 | C++17 | 実機互換 |
| ビルド | CMake | controller_try と同一 |
| JSON | Boost.PropertyTree | パラメータ読込 |
| GUI | OpenGL + imgui | リアルタイムモニタリング・モード切替 |
| 可視化 | Python3 + Matplotlib | 論文品質プロット |
| データ | CSV (fprintf) | 時系列データ出力 |

## 関連ドキュメント

- `~/lab/controller_try/` — ベースフレームワーク
- [002-system-architecture.md](./002-system-architecture.md) — リポジトリ構成
