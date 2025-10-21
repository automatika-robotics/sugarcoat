<picture>
  <source media="(prefers-color-scheme: dark)" srcset="_static/SUGARCOAT_DARK.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/SUGARCOAT_LIGHT.png">
  <img alt="Sugarcoat Logo" src="_static/SUGARCOAT_DARK.png"  width="50%">
</picture>
<br/><br/>

> 🌐 [English Version](../README.md) | 🇨🇳 [简体中文](README.zh.md)

Sugarcoat 🍬 は、ROS2 でイベント駆動型マルチノードシステムを構築するための豊富なシンタックスシュガーを、直感的な Python API で提供するメタフレームワークです。

- Sugarcoat の[**設計思想**](https://automatika-robotics.github.io/sugarcoat/design/index.html)について詳しく学ぶ 📚
- Sugarcoat を使って[**独自の ROS2 パッケージを作成する**](https://automatika-robotics.github.io/sugarcoat/use.html)方法を学ぶ 🚀

## Sugarcoat を使用して作成されたパッケージ

- [**Kompass**](https://automatikarobotics.com/kompass/)：使いやすく直感的な Python API を使用して、堅牢で包括的なイベント駆動型ナビゲーションスタックを構築するためのフレームワーク
- [**EmbodiedAgents**](https://automatika-robotics.github.io/embodied-agents/)：環境からのコンテキスト情報を理解し、記憶し、行動できるインタラクティブな物理エージェントを作成するためのフル機能のフレームワーク。

## 概要

Sugarcoat は、使いやすく、フォールバックと耐障害性を内蔵し、直感的な Python API で設定および起動できるイベント駆動型マルチノードシステムを作成したい ROS2 開発者向けに構築されています。イベント駆動型ソフトウェアの精神に基づき、ROS ノード、およびノードを開始/停止/変更できるイベント/アクションを記述するためのプリミティブを提供します。Sugarcoat は ROS Launch API の代替としても機能します。

[コンポーネント](https://automatika-robotics.github.io/sugarcoat/design/component.html)は Sugarcoat の主要な実行単位であり、各コンポーネントは[入力/出力](https://automatika-robotics.github.io/sugarcoat/design/topics.md)と[フォールバック](https://automatika-robotics.github.io/sugarcoat/design/fallbacks.html)動作で設定されます。さらに、各コンポーネントは自身の[ヘルスステータス](https://automatika-robotics.github.io/sugarcoat/design/status.html)を更新します。コンポーネントは[イベント](https://automatika-robotics.github.io/sugarcoat/design/events.html)と[アクション](https://automatika-robotics.github.io/sugarcoat/design/actions.html)を使用して実行時に動的に処理および再構成できます。イベント、アクション、およびコンポーネントは[ランチャー](https://automatika-robotics.github.io/sugarcoat/design/launcher.html)に渡され、ランチャーはマルチスレッドまたはマルチプロセス実行を使用してコンポーネントのセットを実行します。ランチャーはまた、内部の[モニター](https://automatika-robotics.github.io/sugarcoat/design/monitor.html)を使用して、コンポーネントを追跡し、イベントを監視します。

## ベースコンポーネント

<p align="center">
<picture align="center">
  <source media="(prefers-color-scheme: dark)" srcset="_static/images/diagrams/component_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/images/diagrams/component_light.png">
  <img alt="Base Component" src="_static/images/diagrams/component_light.png" width="75%">
</picture>
</p>

## マルチプロセス実行

<p align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="_static/images/diagrams/multi_process_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/images/diagrams/multi_process_light.png">
  <img alt="Multi-process execution" src="_static/images/diagrams/multi_process_light.png" width="80%">
</picture>
</p>

## インストール

ROS のバージョンが _humble_ 以上の場合、パッケージマネージャーを使って Sugarcoat をインストールできます。たとえば、Ubuntu では以下のように実行します：

`sudo apt install ros-$ROS_DISTRO-automatika-ros-sugar`

あるいは、[リリースページ](https://github.com/automatika-robotics/sugarcoat/releases) からお好みの deb パッケージをダウンロードし、次のコマンドでインストールしてください：

`sudo dpkg -i ros-$ROS_DISTRO-automatica-ros-sugar_$version$DISTRO_$ARCHITECTURE.deb`

パッケージマネージャーからインストールされる attrs のバージョンが 23.2 より古い場合は、以下のように pip を使ってインストールしてください：

`pip install 'attrs>=23.2.0'`

## ソースからのビルド

```shell
mkdir -p ros-sugar-ws/src
cd ros-sugar-ws/src
git clone https://github.com/automatika-robotics/sugarcoat && cd ..
pip install numpy opencv-python-headless 'attrs>=23.2.0' jinja2 msgpack msgpack-numpy setproctitle pyyaml toml
colcon build
source install/setup.bash
```

## 著作権

本配布物内のコードは、明示的に示されていない限り、著作権 (c) 2024 Automatika Robotics に属します。

Sugarcoat は MIT ライセンスの下で提供されます。詳細は [LICENSE](LICENSE) ファイルで確認できます。

## 貢献

Sugarcoat は [Automatika Robotics](https://automatikarobotics.com/) と [Inria](https://inria.fr/) の共同開発です。コミュニティからの貢献を歓迎します。
