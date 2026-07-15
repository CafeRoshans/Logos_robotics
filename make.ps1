# ==============================================================================
# GFSX Robot — ML-Agents workflow (Windows / PowerShell)
# ==============================================================================
# .\make.ps1 help
# .\make.ps1 setup       — проверить conda env и mlagents (один раз)
# .\make.ps1 build       — собрать Unity-билд в Build\GFSX_Simulator.exe
# .\make.ps1 train       — запустить обучение с новым run-id
# .\make.ps1 resume      — продолжить последнее обучение
# .\make.ps1 editor      — запустить обучение из открытого редактора (без билда)
# .\make.ps1 tensorboard — открыть tensorboard
# .\make.ps1 clean-build — удалить билд
# .\make.ps1 clean-run   — удалить папку results (все обучения!)
#
# Параметры (необязательные):
#   -RunId (id)     свой run-id вместо автогенерируемого
#   -NumEnvs (n)    число параллельных сред (по умолчанию 4)
#   -Extra "..."    доп. аргументы, которые прокинутся в mlagents-learn
#
# Пример:
#   .\make.ps1 train -RunId my_run_01 -NumEnvs 8 -Extra "--force"
# ==============================================================================

param(
    [Parameter(Position = 0)]
    [string]$Task = "help",

    [string]$RunId = "gfsx_$(Get-Date -Format 'yyyyMMdd_HHmmss')",
    [int]$NumEnvs = 2,
    [string]$Extra = ""
)

$ErrorActionPreference = "Stop"

# Python по умолчанию буферизирует stdout блоками, если он не подключён к настоящему
# терминалу (как при вызове через & из скрипта) — из-за этого вывод mlagents-learn
# появляется рывками или только в конце. Отключаем буферизацию, чтобы видеть прогресс
# обучения (шаги, награды и т.д.) в реальном времени.
$env:PYTHONUNBUFFERED = "1"

# ---------- пути / версии — ПОДСТАВЬ СВОИ ----------
$UnityVersion = "6000.5.3f1"
$Unity        = "C:\Program Files\Unity\Hub\Editor\$UnityVersion\Editor\Unity.exe"
$ProjectPath  = Join-Path $PSScriptRoot "Unity"
$BuildDir     = Join-Path $PSScriptRoot "Unity\Build"
$BuildOutput  = Join-Path $BuildDir "YandexCamp_Robotics.exe"
$Config       = Join-Path $ProjectPath "config.yaml"

# Conda-окружение mlagents (miniconda/anaconda на Windows)
$CondaEnv    = "$env:USERPROFILE\miniconda3\envs\mlagents"
$MlAgents    = Join-Path $CondaEnv "Scripts\mlagents-learn.exe"
$TensorBoard = Join-Path $CondaEnv "Scripts\tensorboard.exe"

# ==============================================================================
# ---------- утилиты ----------
function Check-Env {
    if (-not (Test-Path $MlAgents)) {
        Write-Host "mlagents не установлен. Запусти: .\make.ps1 setup" -ForegroundColor Red
        exit 1
    }
    if (-not (Test-Path $Config)) {
        Write-Host "Не найден $Config" -ForegroundColor Red
        exit 1
    }
}

function Check-Build {
    if (-not (Test-Path $BuildOutput)) {
        Write-Host "Нет билда. Собери: .\make.ps1 build" -ForegroundColor Red
        exit 1
    }
}

# ==============================================================================
switch ($Task) {

    "help" {
        Write-Host "Доступные команды:"
        Write-Host "  setup build train resume editor tensorboard clean-build clean-run"
        Write-Host ""
        Write-Host "Параметры: -RunId (id)  -NumEnvs (n)  -Extra ""...."""
    }

    # ---------- окружение Python ----------
    "setup" {
        if (-not (Test-Path $CondaEnv)) {
            Write-Host "Conda env не найден: $CondaEnv" -ForegroundColor Red
            Write-Host "Создай: conda create -n mlagents python=3.10 -y"
            exit 1
        }
        if (-not (Test-Path $MlAgents)) {
            Write-Host "mlagents не установлен в env." -ForegroundColor Red
            Write-Host "Установи: conda run -n mlagents pip install mlagents==1.1.0 torch tensorboard"
            exit 1
        }
        Write-Host "OK Conda env готов: $CondaEnv" -ForegroundColor Green
        & $MlAgents --help | Out-Null
        Write-Host "OK mlagents-learn работает" -ForegroundColor Green
    }

    # ---------- сборка билда ----------
    "build" {
        if (-not (Test-Path $Unity)) {
            Write-Host "Unity не найден: $Unity" -ForegroundColor Red
            Write-Host "Проверь версию/путь установки в Unity Hub."
            exit 1
        }
        Write-Host "-> Собираю билд: $BuildOutput"
        Remove-Item $BuildOutput -Force -ErrorAction SilentlyContinue
        New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null

        $logFile = Join-Path $BuildDir "build.log"
        Remove-Item $logFile -Force -ErrorAction SilentlyContinue

        $buildArgs = @(
            "-batchmode",
            "-nographics",
            "-quit",
            "-projectPath", $ProjectPath,
            "-executeMethod", "BuildScript.PerformBuild",
            "-buildOutput", $BuildOutput,
            "-logFile", $logFile
        )

        $proc = Start-Process -FilePath $Unity -ArgumentList $buildArgs -Wait -PassThru -NoNewWindow
        $exitCode = $proc.ExitCode

        if ($exitCode -ne 0 -or -not (Test-Path $BuildOutput)) {
            Write-Host "Сборка не удалась (exit code $exitCode), последние строки лога:" -ForegroundColor Red
            if (Test-Path $logFile) {
                Get-Content $logFile -Tail 80
            } else {
                Write-Host "Файл лога не создан: $logFile" -ForegroundColor Red
            }
            exit 1
        }
        Write-Host "OK Готово: $BuildOutput" -ForegroundColor Green
    }

    # ---------- обучение с билдом ----------
    "train" {
        Check-Env
        Check-Build
        Write-Host "-> RUN_ID=$RunId  NUM_ENVS=$NumEnvs"
        Push-Location $ProjectPath
        try {
            $mlArgs = @(
                $Config,
                "--run-id=$RunId",
                "--env=$BuildOutput",
                "--num-envs=$NumEnvs",
                "--no-graphics"
            )
            if ($Extra) { $mlArgs += $Extra }
            & $MlAgents @mlArgs
        } finally {
            Pop-Location
        }
    }

    # ---------- продолжить последний run ----------
    "resume" {
        Check-Env
        Check-Build
        $resultsDir = Join-Path $ProjectPath "results"
        if (-not (Test-Path $resultsDir)) {
            Write-Host "Нет ни одного run в results/" -ForegroundColor Red
            exit 1
        }
        $last = Get-ChildItem $resultsDir | Sort-Object LastWriteTime -Descending | Select-Object -First 1
        if (-not $last) {
            Write-Host "Нет ни одного run в results/" -ForegroundColor Red
            exit 1
        }
        Write-Host "-> Продолжаю: $($last.Name)"
        Push-Location $ProjectPath
        try {
            $mlArgs = @(
                $Config,
                "--run-id=$($last.Name)",
                "--resume",
                "--env=$BuildOutput",
                "--num-envs=$NumEnvs",
                "--no-graphics"
            )
            if ($Extra) { $mlArgs += $Extra }
            & $MlAgents @mlArgs
        } finally {
            Pop-Location
        }
    }

    # ---------- обучение из редактора (медленно, для отладки) ----------
    "editor" {
        Check-Env
        Write-Host "-> Ждёт Play в Unity Editor. RUN_ID=$RunId"
        Push-Location $ProjectPath
        try {
            $mlArgs = @($Config, "--run-id=$RunId")
            if ($Extra) { $mlArgs += $Extra }
            & $MlAgents @mlArgs
        } finally {
            Pop-Location
        }
    }

    # ---------- tensorboard ----------
    "tensorboard" {
        Check-Env
        Write-Host "-> http://localhost:6006"
        Push-Location $ProjectPath
        try {
            & $TensorBoard --logdir=results
        } finally {
            Pop-Location
        }
    }

    # ---------- утилиты очистки ----------
    "clean-build" {
        Remove-Item $BuildDir -Recurse -Force -ErrorAction SilentlyContinue
        Write-Host "OK Билд удалён" -ForegroundColor Green
    }

    "clean-run" {
        Write-Host "Удалить ВСЮ папку results (все обучения)? [y/N]"
        $confirm = Read-Host
        if ($confirm -eq "y") {
            Remove-Item (Join-Path $ProjectPath "results") -Recurse -Force -ErrorAction SilentlyContinue
            Write-Host "OK results удалён" -ForegroundColor Green
        } else {
            Write-Host "Отменено"
        }
    }

    default {
        Write-Host "Неизвестная команда: $Task" -ForegroundColor Red
        Write-Host "Доступные: setup, build, train, resume, editor, tensorboard, clean-build, clean-run"
    }
}