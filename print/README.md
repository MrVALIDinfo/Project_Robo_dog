# Детали для печати

Полный комплект на одну собаку под [список деталей от 23.10.2026](../README.md#проверка-комплектующих-список-от-23102026): рама KDY0523 в варианте `_cls` (под PDI-HV5523MG, как у mike4192), детали mike4192 и наши. Всё проверено в 3D-модели.

## Рама и ноги — `kdy/`

| Файл | Шт. | Что это |
|---|---|---|
| `kdy/L_arm_cls_cable.stl`, `kdy/R_arm_cls_cable.stl` | 2 + 2 | Бедро `_cls` с пазом 2×5.6 мм под вывод кабеля бедренного серво (в оригинале торец серво стоит к стенке вплотную) |
| `kdy/L_wrist_cls.stl`, `kdy/R_wrist_cls.stl` | 2 + 2 | Голень, в ней коленное серво |
| `kdy/foot.stl` | 4 | Стопа |
| `kdy/L_arm_joint_cls_for_shoulder_reinforcement.stl` | 2 + 2 зеркально | Кронштейн плеча под усиление mike4192 (в нём плечевое серво) |
| `kdy/L_arm_cover_for_shoulder_reinfrocement.stl` | 2 + 2 зеркально | Крышка бедра под усиление mike4192 |
| `kdy/shoulder_reinforcement.stl` | 2 + 2 зеркально | Скоба усиления: вторая опора оси бедра на подшипнике |
| `kdy/I_shoulder_cls.stl` | 2 | Внутренняя переборка плеч |
| `kdy/O_shoulder.stl` | 2 | Наружная переборка плеч |
| `kdy/non-mega_L_side_plate.stl`, `kdy/R_side_plate.stl` | 1 + 1 | Боковины |
| `kdy/non-mega_T_cover_cls.stl` | 1 | Верхняя крышка |
| `kdy/non-mega_B_cover_cls.stl` | 1 | Нижняя крышка (отсек аккумулятора) |
| `kdy/F_cover.stl` | 1 | Морда |
| `kdy/plate_plain.stl` | 1 | Гладкая плита электроники (mike4192) |
| `kdy/raspberry_pi_platform.stl`, `kdy/servo_board_platform.stl` | 1 + 1 | Площадки Raspberry Pi и PCA9685 (mike4192), к плите — двусторонним скотчем |

«Зеркально» — mike4192 даёт только левые детали усиления, правые печатаются отражёнными в слайсере. Без усиления можно печатать стандартные `kdy/L_arm_joint_cls.stl`, `R_arm_joint_cls.stl`, `L_arm_cover.stl`, `R_arm_cover.stl`, но тогда ось плеча держится только на валу серво и люфтит (mike4192). `kdy/L_arm_cls.stl`, `R_arm_cls.stl` и `R_cover.stl` — исходники без доработок, печатать их не нужно.

## Наши детали

| Файл | Шт. | Что это | Крепёж |
|---|---|---|---|
| `R_cover_alatoo.stl` | 1 | Задняя крышка вместо `R_cover`: гравировка «K.D.Y» залита, на её месте выпуклая надпись «ALA-TOO» (заглавные 6 мм, выступ 0.8 мм). Отверстия под LCD и выключатель 19×13 мм — штатные. | как у оригинальной крышки |
| `cam_mount.stl` | 1 | Кронштейн Raspberry Pi Camera v2 в морду. Камера (25 мм) не входит между рёбрами морды (24.5 мм), поэтому кронштейн надевается на передние кромки рёбер С-образными пазами, а камера стоит на клине перед ними и смотрит вперёд с наклоном 10° вниз. Шлейф выходит вниз между рёбрами. | 4× M2×6 + гайки M2 (гайки вдавить в бобышки); к рёбрам — посадка в пазы + капля термоклея |

Печать: PLA/PETG, 0.2 мм, 3 периметра, заполнение 30 %. `cam_mount.stl` уже уложен на стол гранью с минимумом нависаний, поддержки не нужны; пазы под рёбра — с зазором 0.25 мм на сторону, если сидит туго, пройти надфилем. Детали `kdy/` — в ориентации, как их выложили авторы на Thingiverse.

## Пересобрать

```bash
pip install numpy scipy trimesh manifold3d networkx shapely matplotlib rtree
g++ -O2 -o tools/kdy/inside tools/kdy/inside.cpp
python tools/kdy/fit_kdy.py groove     # паз в бедре (нужен tools/kdy/fit.json)
python tools/kdy/fit_kdy.py build      # детали для 3D-модели
python tools/mounts/make_rear.py       # задняя крышка ALA-TOO
python tools/mounts/make_mounts.py     # кронштейн камеры (наклон TILT)
python tools/kdy/cable_routes.py       # трассы кабелей серв
```

Подгонка с нуля (`fit_kdy.py fit <папка>`) и посадка серво (`fit_kdy.py seats`) нужны только при смене деталей: для `fit` — распакованные zip с Thingiverse 3445283, 4591999, 4596267.
