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