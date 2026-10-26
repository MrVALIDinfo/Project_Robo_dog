# Источники и лицензии

Этот репозиторий использует сторонние материалы.

| Материал | Источник | Лицензия |
|---|---|---|
| STL корпуса и ног, URDF (`3d-viewer/stl`, `3d-viewer/spot_micro.urdf`) | [mike4192/spotMicro](https://github.com/mike4192/spotMicro), `spot_micro_rviz`; рама Spot Micro — KDY0523 ([Thingiverse](https://www.thingiverse.com/thing:3445283)) | MIT (mike4192) |
| Модель платы PCA9685 (`3d-viewer/parts/pca_c.stl`) | [adafruit/Adafruit_CAD_Parts](https://github.com/adafruit/Adafruit_CAD_Parts), «815 Servo Driver 16 Channel» | MIT |
| three.js, URDFLoader (`3d-viewer/lib`) | [mrdoob/three.js](https://github.com/mrdoob/three.js), [gkjohnson/urdf-loaders](https://github.com/gkjohnson/urdf-loaders) | MIT |
| Состав деталей и схема подключения | README и `spot_micro_motion_cmd.yaml` проекта mike4192/spotMicro | MIT |
| Кронштейн камеры (`print/cam_mount.stl`, `3d-viewer/parts/cam_mount.stl`) | сделан в этом репозитории (`tools/mounts/make_mounts.py`) по геометрии `frontpart.stl` | как у репозитория |
| Задняя крышка без гравировки K.D.Y и с надписью ALA-TOO (`3d-viewer/stl/backpart_filled.stl`, `3d-viewer/parts/rear_text.stl`, `print/backpart_alatoo.stl`) | изменённый `backpart.stl` mike4192/spotMicro (`tools/mounts/make_rear.py`) | MIT (mike4192) |

Логотип (`docs/logo_source.png`, `3d-viewer/logo.svg`) принадлежит его правообладателю и используется как оформление учебной модели.
