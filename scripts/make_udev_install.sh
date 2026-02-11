#!/bin/bash

set -e

# スクリプト自身のあるディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "udevファイルをコピーします"

sudo cp "${SCRIPT_DIR}/udev/90-aero.rules" /etc/udev/rules.d/

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "完了しました"