# Печать

Всё на одну собаку. Пластик PLA или PETG, слой 0.2 мм, 3 периметра, заполнение 30 %. Ноги лучше печатать из PETG.

## Рама и ноги (`kdy/`)

| Файл | Сколько |
|---|---|
| `L_arm_cls_cable.stl`, `R_arm_cls_cable.stl` — бедро с пазом под кабель | 2 + 2 |
| `L_wrist_cls.stl`, `R_wrist_cls.stl` — голень | 2 + 2 |
| `foot.stl` — стопа | 4 |
| `L_arm_joint_cls_for_shoulder_reinforcement.stl` — кронштейн плеча | 2 + 2 зеркально |
| `L_arm_cover_for_shoulder_reinfrocement.stl` — крышка бедра | 2 + 2 зеркально |
| `shoulder_reinforcement.stl` — усиление плеча | 2 + 2 зеркально |
| `I_shoulder_cls.stl` — внутренняя переборка | 2 |
| `O_shoulder.stl` — наружная переборка | 2 |
| `non-mega_L_side_plate.stl`, `R_side_plate.stl` — боковины | 1 + 1 |
| `non-mega_T_cover_cls.stl` — верхняя крышка | 1 |
| `non-mega_B_cover_cls.stl` — нижняя крышка | 1 |
| `F_cover.stl` — морда | 1 |
| `plate_plain.stl` — плита электроники | 1 |
| `raspberry_pi_platform.stl`, `servo_board_platform.stl` — площадки Pi и PCA9685 | 1 + 1 |

«Зеркально» — правые детали печатаются отражёнными в слайсере. Файлы `L_arm_cls`, `R_arm_cls` и `R_cover` — исходники, их печатать не нужно.

## Наши детали

| Файл | Сколько | Что это |
|---|---|---|
| `R_cover_alatoo.stl` | 1 | Задняя крышка с надписью ALA-TOO |
| `cam_mount.stl` | 1 | Кронштейн камеры в морду: 4 винта M2×6 с гайками, к рёбрам — каплей клея |

## Пересобрать файлы

```bash
pip install numpy scipy trimesh manifold3d networkx shapely matplotlib rtree
g++ -O2 -o tools/kdy/inside tools/kdy/inside.cpp
python tools/kdy/fit_kdy.py build
python tools/mounts/make_rear.py
python tools/mounts/make_mounts.py
```
