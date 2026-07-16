# Обучение GFSX_Brain — гайд для команды

Ветка с RL-версией: **`wexler-rl`**. В ней: `RobotBrain.cs` (агент, награды, спавн по регламенту),
`ObstacleRandomizer.cs` (рандом препятствий каждый эпизод), `DomainRandomizer.cs` (Sim2Real-шум, пока выключен),
префаб `TrainingAreaW` (8 клонов в SceneW), `config.yaml` (PPO+LSTM), обученная модель
`results/.../GFSX_Brain-852440.onnx` (~+6 reward на фиксированной карте).

---

## 0. Клон и ветка

```bash
git clone git@github.com:DrPepper18/YandexCamp_Robotics.git
cd YandexCamp_Robotics
git checkout wexler-rl
```

Открыть проект в Unity 6 (6000.5.3f1), сцена **Assets/Scenes/SceneW**.

## 1. Python-окружение (один раз, локально и на сервере одинаково)

```bash
# нужна conda (miniconda)
conda create -n camp python=3.10 -y
conda activate camp
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install grpcio==1.48.2 protobuf==3.20.3 "setuptools<70"
pip install mlagents==1.1.0
mlagents-learn --help   # должна вывестись справка
```

## 2. Проверка сцены перед любым запуском

На `robot` внутри префаба `TrainingAreaW` (правки — только через Prefab Mode!):

| Что | Значение |
|---|---|
| Behavior Parameters → Behavior Name | `GFSX_Brain` |
| Space Size / Stacked | 15 / 4 |
| Continuous / Discrete | 3 / 1 ветка размером 3 |
| **Behavior Type** | **Default** (для обучения). Heuristic Only = ручное WASD, Inference Only = прогон готовой .onnx |
| Model | None (при обучении) |
| Track Controller | галка ✓, Use Keyboard Input ☐ |
| ROS Connection, Brain Debug HUD | ☐ |
| DomainRandomizer → Enable Randomization | ☐ (включаем только для Sim2Real-прогона) |

## 3. Локальное обучение в редакторе (для отладки/наблюдения)

```bash
conda activate camp
cd <корень проекта>
mlagents-learn config.yaml --run-id=<НОВЫЙ_уникальный_id>
# дождаться "press Play" -> нажать Play в Unity
```

Смотреть графики: `tensorboard --logdir results` → http://localhost:6006 →
`Environment/Cumulative Reward` (должен расти) и `Episode Length` (падает, когда робот начинает хватать).

⚠️ **Правила run-id:** каждый запуск — новый id. Никогда не использовать `--force`
и не писать в старый run-id повторно: это затирает checkpoint.pt обученной модели.

## 4. Сборка Linux-билда (для быстрого обучения и облака)

1. File → Build Profiles → платформа **Linux**, Scene List: только `SceneW`, Development Build ☐ → **Build** → в папку `Build_Linux`, имя `learning`.
2. **Обязательно после каждой сборки** (иначе тренер молча не соединится — известный баг ML-Agents на Linux):
```bash
./fix_build.sh
# скрипт копирует libgrpc_csharp_ext.x64.so из Plugins/AnyCPU в Managed и чистит мусор
```
3. Быстрая проверка:
```bash
mlagents-learn config.yaml --run-id=check_<дата> --env=Build_Linux/learning.x86_64 --num-envs=2 --no-graphics
# ждём "Connected new brain: GFSX_Brain" и первую сводку Step: 20000 -> Ctrl+C
```

## 5. Облачное обучение (Яндекс.Клауд, 16 vCPU)

Билд и чекпоинты в гит не коммитятся — их передаёт Wexler архивом `gfsx_cloud_package.zip`
(внутри: `Build_Linux/` с gRPC-фиксом, `config.yaml`, `results_init/` = чекпоинт 852k для продолжения).

На ВМ (Ubuntu 22/24):
```bash
# залить: scp -i <ключ> gfsx_cloud_package.zip <логин>@<IP>:~/
ssh -i <ключ> <логин>@<IP>
sudo apt-get update && sudo apt-get install -y unzip tmux htop
unzip gfsx_cloud_package.zip
# окружение python: раздел 1 этого гайда (плюс перед conda create:
#   conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
#   conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r )

# подготовить чекпоинт для продолжения:
mkdir -p ~/results
cp -r ~/cloud_package/results_init ~/results/gfsx_comp_editor
chmod +x ~/cloud_package/Build_Linux/learning.x86_64
```

Запуск (обязательно в tmux, чтобы пережил разрыв SSH):
```bash
tmux
conda activate camp
cd ~
mlagents-learn /home/<логин>/cloud_package/config.yaml \
  --run-id=gfsx_cloud_01 \
  --initialize-from=gfsx_comp_editor \
  --env=/home/<логин>/cloud_package/Build_Linux/learning.x86_64 \
  --num-envs=2 --no-graphics \
  --env-args -logFile /dev/null
# отцепиться: Ctrl+B, затем D.  вернуться: tmux attach
```

Проверка, что пошло: в течение минуты `Connected new brain: GFSX_Brain` (дважды) и строка
`Initializing from .../checkpoint.pt`, дальше каждые пару минут `Step: N. Mean Reward: ...`.
`htop` — все ядра загружены (если нет — поднять `--num-envs` до 3–4).

**Критично:** `-logFile /dev/null` не убирать (забьёт диск), пути только абсолютные,
`--force` не использовать.

## 6. Мониторинг и остановка

- Смотреть: `tmux attach`. Сводки `Mean Reward` должны расти; чекпоинты сохраняются каждые 500k шагов автоматически.
- TensorBoard с ВМ: на ВМ `tensorboard --logdir ~/results --port 6006`, на своей машине
  `ssh -i <ключ> -L 6006:127.0.0.1:6006 <логин>@<IP>` → http://localhost:6006
- Остановка: Ctrl+C **один раз**, дождаться строки `Exported ... .onnx` (это и есть сохранение).
- `max_steps: 5000000` в конфиге остановит обучение сам, модель экспортируется автоматически.

## 7. Забрать модель и посмотреть в Unity

```bash
scp -i <ключ> <логин>@<IP>:~/results/gfsx_cloud_01/GFSX_Brain/GFSX_Brain.onnx .
```
В Unity: файл в `Assets/ModelsW/` → Prefab Mode TrainingAreaW → robot → Behavior Parameters →
Model = этот onnx, Behavior Type = **Inference Only** → Play. Робот играет обученной политикой в реальном времени.
После просмотра вернуть Model = None, Behavior Type = Default.

## Частые ошибки

| Симптом | Причина / фикс |
|---|---|
| `UnityTimeOutException` при `--env` | Забыли `./fix_build.sh` после сборки (gRPC-либа не в Managed) |
| `mlagents-learn: command not found` | Не активирован conda env: `conda activate camp` |
| `run-id already exists` | Взять новый run-id (не `--force`!) |
| Робот в билде не едет | В префабе выключен Track Controller или Behavior Type ≠ Default |
| `Can't use InferenceOnly without a model` | Behavior Type = Inference Only, а Model пустой |
| Reward стоит на месте >300k шагов | Писать Wexler'у :) |
