#!/bin/bash
# Запуск 3D-модели SpotMicro: локальный сервер + браузер
cd "$(dirname "$0")"
pgrep -f "http.server 8765" >/dev/null || (nohup python3 -m http.server 8765 --bind 127.0.0.1 >/dev/null 2>&1 &)
sleep 1
xdg-open "http://127.0.0.1:8765/index.html"
