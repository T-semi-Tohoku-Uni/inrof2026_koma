# inrof2026_koma

# ロボット
知能ロボコン2026に出場しました．

![robot](images/robot_1.jpg)
![robot](images/robot_2.jpg)

# 開発環境

本リポジトリはROS 2の`colcon`/`ament_cmake`ワークスペースです。
開発とCIの基準環境は以下の通りです。

| 項目 | 基準環境 |
| --- | --- |
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| 主な言語 | C++17、Python（launchファイル） |
| ビルド | colcon、ament_cmake、CMake、GCC/G++ |
| DDS | Fast DDS（`rmw_fastrtps_cpp`） |
| シミュレータ | Gazebo Sim、`ros_gz_sim`、`ros_gz_bridge` |
| 可視化 | RViz2、Foxglove Bridge |
| 主な外部依存 | BehaviorTree.CPP 4.7.0、LibTorch 2.7.0 CPU |

- Linuxでの開発はUbuntu 22.04を推奨します。
- Windowsにネイティブ環境の設定はありません。WSL2上のUbuntu 22.04、またはDocker Desktopを使ってください。
- Apple Silicon Macでは、Linux/amd64 Docker環境でビルドし、Pixiの`osx-arm64`環境でRViz2を起動します。
- Raspberry Piなどのaarch64環境では、別途aarch64用LibTorchが必要です。

# 環境構築

## 1. ROS 2のインストール

[ROS 2 Humbleの公式手順](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)に従って、Ubuntu 22.04にROS 2 Humbleをインストールしてください。
シミュレーションやRViz2を使う開発PCでは`ros-humble-desktop`を推奨します。

ROS 2をインストールした後、シェルに読み込みます。

```bash
source /opt/ros/humble/setup.bash
```

Zshを使う場合は`setup.bash`を`setup.zsh`に読み替えてください。

## 2. リポジトリの取得

URDFとLiDARドライバはGitサブモジュールで管理しているため、サブモジュールも同時に取得します。

```bash
cd
git clone --recurse-submodules git@github.com:T-semi-Tohoku-Uni/inrof2026_koma.git
cd inrof2026_koma
```

既にclone済みの場合は、次のコマンドでサブモジュールを取得します。

```bash
git submodule update --init --recursive
```

`.gitmodules`のURLはSSH形式のため、GitHubのSSH鍵設定が必要です。

## 3. ビルドツールと依存パッケージ

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  clang-format \
  cmake \
  git \
  libboost-dev \
  libdw-dev \
  libopenblas-dev \
  libqt5svg5-dev \
  libspdlog-dev \
  libzmq3-dev \
  python3-colcon-common-extensions \
  python3-rosdep \
  qtbase5-dev \
  ros-humble-ament-cmake-clang-format \
  ros-humble-ament-clang-format \
  unzip \
  wget
```

`rosdep`を初めて使うPCでは初期化します。

```bash
sudo rosdep init
```

`rosdep init`が`already initialized`になる場合は、そのまま次へ進んでください。

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 4. BehaviorTree.CPP 4.7.0

Docker/CI環境と同じバージョンをソースからインストールします。

```bash
git clone --depth 1 --branch 4.7.0 \
  https://github.com/BehaviorTree/BehaviorTree.CPP.git \
  /tmp/BehaviorTree.CPP
cmake -S /tmp/BehaviorTree.CPP -B /tmp/BehaviorTree.CPP/build
cmake --build /tmp/BehaviorTree.CPP/build -j"$(nproc)"
sudo cmake --install /tmp/BehaviorTree.CPP/build
sudo ldconfig
```

## 5. LibTorch 2.7.0 CPU

### x86_64

```bash
cd src/komarm
wget -O /tmp/libtorch.zip \
  'https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcpu.zip'
rm -rf libtorch
unzip /tmp/libtorch.zip
rm /tmp/libtorch.zip
cd ../..
```

### aarch64（Raspberry Piなど）

[libtorch_aarch64のRelease](https://github.com/T-semi-Tohoku-Uni/libtorch_aarch64/releases/tag/torch)からビルド済みバイナリをダウンロードし、`src/komarm`に配置します。

```bash
cd src/komarm
unzip lib.linux-aarch64-cpython-310.zip
rm lib.linux-aarch64-cpython-310.zip
cd ../..
```

## 6. Git hookの設定

```bash
chmod a+x .githooks/*
git config --local core.hooksPath .githooks
```

## プログラムのフォーマット

```bash
find ./src \( -path './src/*/include/*.hpp' -o -path './src/*/src/*.cpp' \) $(git submodule status | awk '{print "! -path ./" $2 "/*"}') -print0 | xargs -0 ament_clang_format
```

# 実行
## ビルド
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
## シミュレーションの実行
```bash
source install/setup.bash
ros2 launch inrof2026_koma simulation.launch.py
```

## 実機での実行
```bash
source install/setup.bash
ros2 launch inrof2026_koma real.launch.py
```

## 別端末でrvizのみ表示したい場合
```bash
colcon build --symlink-install --packages-select visualizer
source install/setup.bash
ros2 launch visualizer rviz.launch.py
```

Zshの場合は、上記の`setup.bash`を`setup.zsh`に読み替えてください。

## Apple Silicon MacでRViz2だけ表示

[Docker Desktop](https://www.docker.com/products/docker-desktop/)と[Pixi](https://pixi.sh/) が必要です。
Docker環境でLinux側のワークスペースをビルドし、Pixiの`osx-arm64`環境でRViz2を起動します。
以下はcloneしたリポジトリのルートで実行してください。

Dockerイメージとコンテナを作成します。

```bash
docker build --platform linux/amd64 -t inrof2026_koma:latest -f docker/Dockerfile docker
docker run -d --name "inrof2026_koma" -v ".:/work/inrof2026_koma" -w /work/inrof2026_koma "inrof2026_koma:latest" tail -f /dev/null
```

LibTorchと`rosdep`の環境を整えます。

```bash
docker exec inrof2026_koma bash -lc "
  set -e
  apt-get update
  apt-get install -y --no-install-recommends wget unzip libopenblas-dev
  cd /work/inrof2026_koma/src/komarm
  wget -O /tmp/libtorch.zip 'https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcpu.zip'
  rm -rf libtorch
  unzip /tmp/libtorch.zip
  rm -f /tmp/libtorch.zip
"
```

```bash
docker exec inrof2026_koma bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  apt-get update
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
"
```

`colcon build`を実行します。

```bash
docker exec inrof2026_koma bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install
"
```

Dockerコンテナ外のmacOSターミナルで、Pixi環境からRViz2を起動します。

```bash
pixi install
pixi shell
source install/setup.zsh
ros2 launch visualizer rviz.launch.py
```
