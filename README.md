# inrof2026_koma

## 環境構築
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

## プログラムのフォーマット
```bash
find ./src \( -path './src/*/include/*.hpp' -o -path './src/*/src/*.cpp' \) -print0 | xargs -0 ament_clang_format --reformat
```
libtorchのダウンロード
```bash
cd inrof2026_koma/src/komarm
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.7.0+cpu.zip
rm -rf libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcpu.zip
LD_LIBRARY_PATH=${HOME}/inrof2026_koma/src/komarm/libtorch/lib:$LD_LIBRARY_PATH
```

