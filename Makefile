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
NUM_ENVS ?= 4
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
tensorboard: check-env
	@echo "→ http://localhost:6006"
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
