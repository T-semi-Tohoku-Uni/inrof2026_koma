# inrof2026_koma

# 環境構築
必要なライブラリのダウンロード
```bash
sudo apt update
sudo apt install -y \
  clang-format \
  ros-humble-ament-cmake-clang-format \
  ros-humble-ament-clang-format
```

commitメッセージのフォーマット設定
```bash
chmod a+x .githooks/*
git config --local core.hooksPath .githooks
```

## プログラムのフォーマット設定
```bash
find ./src \( -path './src/*/include/*.hpp' -o -path './src/*/src/*.cpp' \) -print0 | xargs -0 ament_clang_format --reformat
```
## libtorchのダウンロード
x86環境の場合
```bash
cd inrof2026_koma/src/komarm
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.7.0+cpu.zip
rm -rf libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcpu.zip
```
aarch64環境の場合（ラズパイなど）
https://github.com/T-semi-Tohoku-Uni/libtorch_aarch64/releases/tag/torchから，ビルド済みのバイナリをダウンロードし，`inrof2026_koma/src/komarm`に配置
```bash
unzip lib.linux-aarch64-cpython-310.zip
rm -rf lib.linux-aarch64-cpython-310.zip
```

# 実行
## ビルド
```
colcon build --symlink-install
```

## 別端末でrvizのみ表示したい場合
```
colcon build --symlink-install --packages-select visualizer
source install/setup.zsh
ros2 launch visualizer rviz.launch.py
```


