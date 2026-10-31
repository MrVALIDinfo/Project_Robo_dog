# Генератор картонного макета

```bash
pip install numpy matplotlib shapely
cd tools/cardboard
python cb_sheets.py ../../cardboard                   # шаблоны
python cb_manual.py ../../cardboard/instrukciya.pdf   # инструкция
```

Толщина картона задаётся в начале `cb_geom.py` (`T = 3.0`). Поменяй её и запусти оба скрипта — прорези пересчитаются.
