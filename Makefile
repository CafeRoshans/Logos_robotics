# ==============================================================================
# GFSX Robot — ML-Agents workflow
# ==============================================================================
# make setup       — создать venv и поставить mlagents (один раз)
# make build       — собрать Unity-билд в Build/GFSX_Simulator.app
# make train       — запустить обучение с новым run-id
# make resume      — продолжить последнее обучение
# make editor      — запустить обучение из открытого редактора (без билда)
# make tensorboard — открыть tensorboard
# make clean-build — удалить билд
# make clean-run   — удалить папку results (все обучения!)
# ==============================================================================

# ---------- пути / версии ----------
UNITY_VERSION := 6000.5.3f1
UNITY         := /Applications/Unity/Hub/Editor/$(UNITY_VERSION)/Unity.app/Contents/MacOS/Unity
PROJECT       := $(CURDIR)/Unity
BUILD_DIR     := $(CURDIR)/Build
BUILD_OUTPUT  := $(BUILD_DIR)/GFSX_Simulator.app
CONFIG        := $(PROJECT)/config.yaml

# Conda-окружение mlagents (miniconda на homebrew)
CONDA_ENV     := /opt/homebrew/Caskroom/miniconda/base/envs/mlagents
PYTHON        := $(CONDA_ENV)/bin/python
MLAGENTS      := $(CONDA_ENV)/bin/mlagents-learn
TENSORBOARD   := $(CONDA_ENV)/bin/tensorboard

# ---------- параметры запуска ----------
RUN_ID   ?= gfsx_$(shell date +%Y%m%d_%H%M%S)
NUM_ENVS ?= 1
EXTRA    ?=

# ==============================================================================
.PHONY: help setup build train resume editor tensorboard clean-build clean-run

help:
	@grep -E '^# make' Makefile | sed 's/^# //'

# ---------- окружение Python ----------
# Активация conda в Makefile не работает (каждая строка — новый shell).
# Вместо этого проверяем, что окружение существует и все бинари на месте.
setup:
	@test -d $(CONDA_ENV) || (echo "Conda env не найден: $(CONDA_ENV)"; \
	  echo "Создай: conda create -n mlagents python=3.10 -y"; exit 1)
	@test -x $(MLAGENTS)  || (echo "mlagents не установлен в env."; \
	  echo "Установи: conda run -n mlagents pip install 'mlagents==1.1.0' torch tensorboard"; exit 1)
	@echo "✓ Conda env готов: $(CONDA_ENV)"
	@$(MLAGENTS) --help > /dev/null && echo "✓ mlagents-learn работает"
# ---------- сборка билда ----------
build:
	@test -x $(UNITY) || (echo "Unity не найден: $(UNITY)"; exit 1)
	@echo "→ Собираю билд: $(BUILD_OUTPUT)"
	@rm -rf $(BUILD_OUTPUT)
	@mkdir -p $(BUILD_DIR)
	@$(UNITY) -batchmode -nographics -quit \
		-projectPath $(PROJECT) \
		-executeMethod BuildScript.PerformBuild \
		-buildOutput $(BUILD_OUTPUT) \
		-logFile $(BUILD_DIR)/build.log || \
		(tail -50 $(BUILD_DIR)/build.log; exit 1)
	@xattr -dr com.apple.quarantine $(BUILD_OUTPUT) 2>/dev/null || true
	@echo "✓ Готово: $(BUILD_OUTPUT)"

# ---------- обучение с билдом ----------
train: check-env check-build
	@echo "→ RUN_ID=$(RUN_ID)  NUM_ENVS=$(NUM_ENVS)"
	@cd $(PROJECT) && $(MLAGENTS) $(CONFIG) \
		--run-id=$(RUN_ID) \
		--env=$(BUILD_OUTPUT) \
		--num-envs=$(NUM_ENVS) \
		--no-graphics \
		$(EXTRA)

# ---------- продолжить последний run ----------
resume: check-env check-build
	@LAST=$$(ls -t $(PROJECT)/results 2>/dev/null | head -1); \
	if [ -z "$$LAST" ]; then echo "Нет ни одного run в results/"; exit 1; fi; \
	echo "→ Продолжаю: $$LAST"; \
	cd $(PROJECT) && $(MLAGENTS) $(CONFIG) \
		--run-id=$$LAST --resume \
		--env=$(BUILD_OUTPUT) --num-envs=$(NUM_ENVS) --no-graphics $(EXTRA)

# ---------- обучение из редактора (медленно, для отладки) ----------
editor: check-env
	@echo "→ Ждёт Play в Unity Editor. RUN_ID=$(RUN_ID)"
	@cd $(PROJECT) && $(MLAGENTS) $(CONFIG) --run-id=$(RUN_ID) $(EXTRA)

# ---------- tensorboard ----------
# Открыв TB, вставь этот regex в "Filter tags (regex)" наверху, чтобы скрыть шум:
#   ^Environment/(Cumulative Reward|Episode Length)$|^Policy/(Entropy|Learning Rate)$|^Losses/|^Custom/
tensorboard: check-env
	@echo "→ http://localhost:6006"
	@echo "→ Фильтр тегов в TB (paste в поле сверху):"
	@echo '   ^Environment/(Cumulative Reward|Episode Length)$$|^Policy/(Entropy|Learning Rate)$$|^Losses/|^Custom/'
	@cd $(PROJECT) && $(TENSORBOARD) --logdir=results

# ---------- утилиты ----------
check-env:
	@test -x $(MLAGENTS) || (echo "mlagents не установлен. Запусти: make setup"; exit 1)
	@test -f $(CONFIG)   || (echo "Не найден $(CONFIG)"; exit 1)

check-build:
	@test -d $(BUILD_OUTPUT) || (echo "Нет билда. Собери: make build"; exit 1)

clean-build:
	@rm -rf $(BUILD_DIR)
	@echo "✓ Билд удалён"

clean-run:
	@echo "Удалить ВСЮ папку results (все обучения)? Ctrl+C для отмены, Enter — продолжить."
	@read _
	@rm -rf $(PROJECT)/results
	@echo "✓ results удалён"

# ==============================================================================
# ☁️  YANDEX CLOUD — обучение на удалённой Linux CPU-машине
# ==============================================================================
# Первый раз:
#   1) укажи CLOUD_USER / CLOUD_IP / CLOUD_KEY (в CLI или в этом файле)
#   2) make cloud-setup      — установка miniconda + mlagents на сервере
#   3) make cloud-run        — build linux + upload + train (одной командой)
#
# Ежедневно:
#   make cloud-run                       — быстро всё сначала (свежий билд)
#   make cloud-resume                    — продолжить последний run
#   make cloud-tb                        — SSH-туннель + tensorboard
#   make cloud-download RUN_ID=xxx       — скачать модель
#   make cloud-ssh                       — открыть SSH-shell
# ==============================================================================

# --- параметры облака (переопределяй в CLI: make cloud-run CLOUD_IP=1.2.3.4) ---
CLOUD_USER        ?= yacamptest
CLOUD_IP          ?= CHANGEME
CLOUD_KEY         ?= $(HOME)/.ssh/yc_key
CLOUD_ENV_BIN     ?= GFSX_Simulator.x86_64
CLOUD_NUM_ENVS    ?= 16
CLOUD_RUN_ID      ?= cloud_$(shell date +%Y%m%d_%H%M%S)
CLOUD_EXTRA       ?=

# --- локальные пути под linux-билд ---
LINUX_BUILD_DIR   := $(CURDIR)/Build_Linux
LINUX_BUILD_BIN   := $(LINUX_BUILD_DIR)/$(CLOUD_ENV_BIN)
LINUX_BUILD_ZIP   := $(CURDIR)/Build_Linux.zip

# --- удалённые пути ---
REMOTE_HOME       := /home/$(CLOUD_USER)
REMOTE_BUILD_DIR  := $(REMOTE_HOME)/Build_Linux
REMOTE_ENV_BIN    := $(REMOTE_BUILD_DIR)/$(CLOUD_ENV_BIN)
REMOTE_CONFIG     := $(REMOTE_HOME)/config.yaml
REMOTE_MLAGENTS   := $(REMOTE_HOME)/miniconda/envs/mlagents/bin/mlagents-learn
REMOTE_TENSORBOARD := $(REMOTE_HOME)/miniconda/envs/mlagents/bin/tensorboard

# --- ssh/scp хелперы ---
SSH := ssh -i $(CLOUD_KEY) -o StrictHostKeyChecking=accept-new $(CLOUD_USER)@$(CLOUD_IP)
SCP := scp -i $(CLOUD_KEY) -o StrictHostKeyChecking=accept-new

.PHONY: cloud-help cloud-check cloud-build-linux cloud-package cloud-upload cloud-upload-config \
        cloud-setup cloud-train cloud-run cloud-resume cloud-status cloud-tail cloud-stop \
        cloud-download cloud-tb cloud-ssh cloud-clean-remote

cloud-help:
	@echo "☁️  Yandex Cloud targets:"
	@echo "   make cloud-setup                          — установить miniconda+mlagents на сервере (один раз)"
	@echo "   make cloud-run                            — build linux + upload + train"
	@echo "   make cloud-resume                         — продолжить последний run"
	@echo "   make cloud-build-linux                    — только собрать Linux Server билд"
	@echo "   make cloud-package                        — упаковать в Build_Linux.zip"
	@echo "   make cloud-upload                         — залить билд + конфиг на сервер"
	@echo "   make cloud-train                          — запустить обучение (не собирая)"
	@echo "   make cloud-status                         — статус обучения на сервере"
	@echo "   make cloud-tail                           — tail лога обучения"
	@echo "   make cloud-stop                           — остановить обучение на сервере"
	@echo "   make cloud-download RUN_ID=xxx            — скачать GFSX_Brain.onnx"
	@echo "   make cloud-tb                             — SSH-туннель + tensorboard (http://localhost:6006)"
	@echo "   make cloud-ssh                            — интерактивный SSH"
	@echo "   make cloud-clean-remote                   — снести Build_Linux на сервере"
	@echo ""
	@echo "Переменные (переопределяй в CLI): CLOUD_USER=$(CLOUD_USER)  CLOUD_IP=$(CLOUD_IP)"

cloud-check:
	@test "$(CLOUD_IP)" != "CHANGEME" || (echo "❌ Задай CLOUD_IP: make cloud-run CLOUD_IP=1.2.3.4"; exit 1)
	@test -f $(CLOUD_KEY) || (echo "❌ Нет ключа $(CLOUD_KEY)"; exit 1)

# --- 1. Сборка Linux Server билда ---
cloud-build-linux:
	@test -x $(UNITY) || (echo "❌ Unity не найден: $(UNITY)"; exit 1)
	@echo "→ Собираю Linux Dedicated Server: $(LINUX_BUILD_BIN)"
	@rm -rf $(LINUX_BUILD_DIR)
	@mkdir -p $(LINUX_BUILD_DIR)
	@$(UNITY) -batchmode -nographics -quit \
		-projectPath $(PROJECT) \
		-executeMethod BuildScript.PerformLinuxServerBuild \
		-buildOutput $(LINUX_BUILD_BIN) \
		-logFile $(LINUX_BUILD_DIR)/build.log || \
		(tail -50 $(LINUX_BUILD_DIR)/build.log; exit 1)
	@echo "✓ Готово: $(LINUX_BUILD_BIN)"

# --- 2. Упаковка в zip для быстрой заливки ---
cloud-package: cloud-build-linux
	@echo "→ Упаковываю в $(LINUX_BUILD_ZIP)"
	@cd $(CURDIR) && rm -f $(LINUX_BUILD_ZIP) && zip -r -q Build_Linux.zip Build_Linux
	@ls -lh $(LINUX_BUILD_ZIP) | awk '{print "  размер: "$$5}'

# --- 3. Заливка на сервер ---
cloud-upload: cloud-check cloud-package cloud-upload-config
	@echo "→ Заливаю $(LINUX_BUILD_ZIP) → $(CLOUD_USER)@$(CLOUD_IP):~/"
	@$(SCP) $(LINUX_BUILD_ZIP) $(CLOUD_USER)@$(CLOUD_IP):~/
	@echo "→ Распаковываю на сервере"
	@$(SSH) 'rm -rf $(REMOTE_BUILD_DIR) && unzip -q ~/Build_Linux.zip -d ~/ && chmod +x $(REMOTE_ENV_BIN)'
	@echo "✓ Билд на сервере готов"

cloud-upload-config: cloud-check
	@test -f $(CONFIG) || (echo "❌ Нет $(CONFIG)"; exit 1)
	@$(SCP) $(CONFIG) $(CLOUD_USER)@$(CLOUD_IP):$(REMOTE_CONFIG)
	@echo "✓ Конфиг залит: $(REMOTE_CONFIG)"

# --- 4. Setup сервера (один раз) ---
cloud-setup: cloud-check
	@echo "→ Устанавливаю окружение на $(CLOUD_USER)@$(CLOUD_IP) (займёт 5-10 мин)"
	@$(SSH) 'bash -s' <<-'EOF'
		set -e
		echo "=== 1/5 apt packages ==="
		sudo apt-get update -qq
		sudo apt-get install -y -qq unzip wget git build-essential
		echo "=== 2/5 miniconda ==="
		if [ ! -d "$$HOME/miniconda" ]; then
		    wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh
		    bash /tmp/miniconda.sh -b -p $$HOME/miniconda
		    rm /tmp/miniconda.sh
		fi
		source $$HOME/miniconda/bin/activate
		conda init bash > /dev/null 2>&1 || true
		echo "=== 3/5 conda ToS ==="
		conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main 2>/dev/null || true
		conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r 2>/dev/null || true
		echo "=== 4/5 python env ==="
		if ! conda env list | grep -q mlagents; then
		    conda create -n mlagents python=3.10.12 -y -q
		fi
		source $$HOME/miniconda/bin/activate mlagents
		pip install -q "torch~=2.2.1" --index-url https://download.pytorch.org/whl/cpu
		conda install -y -q "grpcio=1.48.2" -c conda-forge
		pip install -q "setuptools<70"
		pip install -q "mlagents==1.1.0"
		echo "=== 5/5 верификация ==="
		mlagents-learn --help > /dev/null && echo "✓ mlagents-learn OK"
	EOF
	@echo "✓ Сервер готов"

# --- 5. Запуск обучения (в фоне, чтобы можно было отключиться) ---
cloud-train: cloud-check
	@echo "→ Стартую обучение RUN_ID=$(CLOUD_RUN_ID) на $(CLOUD_NUM_ENVS) сред"
	@$(SSH) 'test -x $(REMOTE_ENV_BIN)' || (echo "❌ Билд не найден. Запусти make cloud-upload"; exit 1)
	@$(SSH) 'test -f $(REMOTE_CONFIG)'  || (echo "❌ Конфиг не найден. Запусти make cloud-upload-config"; exit 1)
	@$(SSH) 'pgrep -f mlagents-learn > /dev/null && echo "⚠️  mlagents-learn уже крутится, останови его через make cloud-stop" && exit 1 || true'
	@$(SSH) 'source ~/miniconda/bin/activate mlagents && \
	         cd ~ && \
	         nohup mlagents-learn $(REMOTE_CONFIG) \
	             --run-id=$(CLOUD_RUN_ID) \
	             --env=$(REMOTE_ENV_BIN) \
	             --num-envs=$(CLOUD_NUM_ENVS) \
	             --no-graphics \
	             $(CLOUD_EXTRA) \
	             --env-args -logFile /dev/null \
	             > ~/train_$(CLOUD_RUN_ID).log 2>&1 &'
	@echo "✓ Обучение запущено в фоне, RUN_ID=$(CLOUD_RUN_ID)"
	@echo "→ Мониторить:  make cloud-tail RUN_ID=$(CLOUD_RUN_ID)"
	@echo "→ TensorBoard: make cloud-tb"

# --- 6. Всё-в-одном: build + upload + train ---
cloud-run: cloud-upload
	@$(MAKE) cloud-train

# --- 7. Продолжить последний run ---
cloud-resume: cloud-check
	@LAST=$$($(SSH) 'ls -t ~/results 2>/dev/null | head -1'); \
	if [ -z "$$LAST" ]; then echo "❌ На сервере нет ни одного run"; exit 1; fi; \
	echo "→ Продолжаю $$LAST"; \
	$(SSH) "source ~/miniconda/bin/activate mlagents && cd ~ && \
	         nohup mlagents-learn $(REMOTE_CONFIG) \
	             --run-id=$$LAST --resume \
	             --env=$(REMOTE_ENV_BIN) --num-envs=$(CLOUD_NUM_ENVS) --no-graphics \
	             --env-args -logFile /dev/null \
	             > ~/train_$$LAST.log 2>&1 &"
	@echo "✓ Resume запущен"

# --- Мониторинг ---
cloud-status: cloud-check
	@echo "→ Процессы:"; $(SSH) 'pgrep -af mlagents-learn || echo "  (не запущено)"'
	@echo "→ Runs:";     $(SSH) 'ls -la ~/results 2>/dev/null || echo "  (нет)"'

cloud-tail: cloud-check
	@LAST=$$(ls -t /tmp 2>/dev/null | head -0 ; \
	    $(SSH) 'ls -t ~/train_*.log 2>/dev/null | head -1'); \
	if [ -z "$$LAST" ]; then echo "❌ Нет train_*.log на сервере"; exit 1; fi; \
	echo "→ Tail $$LAST (Ctrl+C для выхода)"; \
	$(SSH) "tail -f $$LAST"

cloud-stop: cloud-check
	@$(SSH) 'pkill -f mlagents-learn && echo "✓ Остановлено" || echo "⚠️  Не было запущено"'

# --- Скачать модель ---
cloud-download: cloud-check
	@test "$(RUN_ID)" != "" || (echo "❌ Укажи RUN_ID: make cloud-download RUN_ID=cloud_20260715_1200"; exit 1)
	@echo "→ Скачиваю $(RUN_ID)/GFSX_Brain.onnx → $(PROJECT)/Assets/Models/"
	@mkdir -p $(PROJECT)/Assets/Models
	@$(SCP) $(CLOUD_USER)@$(CLOUD_IP):~/results/$(RUN_ID)/GFSX_Brain.onnx $(PROJECT)/Assets/Models/
	@echo "✓ Готово: $(PROJECT)/Assets/Models/GFSX_Brain.onnx"

# --- TensorBoard через SSH-туннель ---
cloud-tb: cloud-check
	@echo "→ Запускаю tensorboard на сервере и пробрасываю порт 6006"
	@echo "→ Открой в браузере: http://localhost:6006"
	@echo "→ Ctrl+C для отключения"
	@$(SSH) -L 6006:127.0.0.1:6006 \
	    "source ~/miniconda/bin/activate mlagents && tensorboard --logdir ~/results --port 6006 --bind_all"

# --- Интерактивный SSH ---
cloud-ssh: cloud-check
	@$(SSH)

# --- Очистка удалённого билда ---
cloud-clean-remote: cloud-check
	@echo "Удалить $(REMOTE_BUILD_DIR) и ~/Build_Linux.zip на сервере? Enter — да, Ctrl+C — нет."
	@read _
	@$(SSH) 'rm -rf $(REMOTE_BUILD_DIR) ~/Build_Linux.zip'
	@echo "✓ Удалено"
