using UnityEngine;

/// <summary>
/// Автономное управление камерой по ДВУМ осям — сеть НЕ управляет камерой напрямую.
///   1) Пока мяч виден — P(ID)-регулятор держит его в центре кадра по обеим осям
///      одновременно: горизонталь (yaw/поворот) и вертикаль (tilt/наклон).
///   2) Как только мяч теряется — камера сначала "держит взгляд" на последнем известном
///      направлении (holdLastKnownSeconds), а затем начинает поиск: качели по yaw между
///      -maxAngleDeg и +maxAngleDeg (СНАЧАЛА в ту сторону, где мяч видели последний раз),
///      И ОДНОВРЕМЕННО зигзаг по tilt между searchTiltMinDeg и searchTiltMaxDeg — чтобы
///      обшаривать разную высоту, а не только разворот в одной горизонтальной плоскости.
///   3) Движение камеры всегда ДИСКРЕТНО (обе оси) — реальный дешёвый сервопривод физически
///      не может встать в произвольный угол, только в одно из положений с шагом stepQuantumDeg.
///      Величина шага ПРОПОРЦИОНАЛЬНА ошибке (растёт от stepDeg/tiltStepDeg при малой ошибке
///      до maxStepDeg/maxTiltStepDeg при большой, см. QuantizedProportionalStep) — раньше PID
///      использовался только по ЗНАКУ выхода, сама величина (и, соответственно, Kp) вообще
///      ни на что не влияли.
///
/// Работает независимо от decision period ML-Agents — вызывается из RobotBrain.FixedUpdate()
/// каждый физический тик. Сеть получает оба угла камеры (CurrentAngleDeg, CurrentTiltDeg)
/// как наблюдения и учится доворачивать КОРПУС туда же, куда сейчас смотрит камера
/// по горизонтали (см. bodyCameraAlignmentBonus в RobotBrain).
///
/// TODO (НЕ ПОДТВЕРЖДЕНО, на 2026-07-21): физически не проверено, есть ли у реального GFS-X
/// отдельный сервопривод под наклон (tilt) камеры. В Robot/config.py есть второй серво-канал
/// SERVO_CAMERA_LOW=7 (отдельно от SERVO_CAMERA_PAN=8), но в Robot/numpy_brain.py он всего
/// раз выставляется в 90° при инициализации (init_arm()) и дальше НИКОГДА не управляется
/// динамически — то есть похоже на реальный тилт-сервопривод, но подключённый к текущему
/// (устаревшему) циклу управления. Если выяснится, что SERVO_CAMERA_LOW — НЕ наклон камеры
/// (а что-то другое, например фиксированная высота крепления) — весь тилт здесь (PID, поиск
/// по tilt, CurrentTiltDeg как наблюдение в RobotBrain) нужно убирать: по правилу проекта,
/// если реальный робот не может измерить/исполнить параметр — в симуляции его тоже быть
/// не должно (см. обсуждение и удаление heading/delta.x/delta.z по этой же причине).
/// </summary>
public class CameraAimController : MonoBehaviour
{
    [Header("Ссылка на сервопривод")]
    [Tooltip("Transform, вокруг которого физически крутится камера (Y — поворот/yaw, " +
             "X — наклон/tilt) — используется, ТОЛЬКО если panPivot/tiltPivot ниже не заданы " +
             "(старая однотрансформенная схема, оба угла на одном объекте).")]
    public Transform cameraServo;

    [Header("Раздельные pivot'ы (нужны, если на реальном роботе датчики механически " +
            "связаны только с ОДНОЙ из осей камеры — например, УЗ-датчик крутится с yaw, " +
            "но не наклоняется с tilt)")]
    [Tooltip("Внешний pivot — крутится ТОЛЬКО по yaw (горизонталь). Если задан вместе с " +
             "tiltPivot — привяжи к этому объекту точку УЗ-датчика (VirtualSensors.centerPoint), " +
             "если на реальном роботе УЗ мехнически связан с горизонтальным поворотом камеры.")]
    public Transform panPivot;
    [Tooltip("Внутренний pivot (дочерний panPivot) — крутится ТОЛЬКО по tilt (вертикаль). " +
             "Сама Camera должна быть дочерней именно этого объекта.")]
    public Transform tiltPivot;

    [Header("Пределы — ПОВОРОТ (yaw, горизонталь)")]
    [Tooltip("Максимальный угол поворота камеры от корпуса, ± градусы. Синхронизируется " +
             "с RobotBrain.cameraServoMaxAngle автоматически в Initialize() — не меняй " +
             "тут вручную по отдельности, будет рассинхрон с наблюдением сети. " +
             "Реальная камера GFS-X: <45° в каждую сторону.")]
    public float maxAngleDeg = 45f;

    [Header("Пределы — НАКЛОН (tilt, вертикаль)")]
    [Tooltip("Нижний предел наклона (камера смотрит вниз), градусы. Отрицательный — " +
             "0 это уровень горизонта, камера никогда не задирается выше горизонта.")]
    public float minTiltDeg = -10f;
    [Tooltip("Верхний предел наклона, градусы. По умолчанию 0 — камера не поднимается " +
             "выше уровня горизонта (мяч на полу физически не бывает выше камеры).")]
    public float maxTiltDeg = 0f;

    [Header("Дискретный шаг ПРИ СЛЕЖЕНИИ (эмуляция реального сервопривода)")]
    [Tooltip("МИНИМАЛЬНЫЙ шаг ПОВОРОТА при слежении, градусы — применяется, когда ошибка только " +
             "чуть выше centerDeadband. Дальше шаг растёт пропорционально ошибке до maxStepDeg — " +
             "чем мяч дальше от центра, тем сильнее доворот за один тик. Референс реального железа: 1-2°.")]
    [Range(0.5f, 5f)]
    public float stepDeg = 1.5f;
    [Tooltip("МАКСИМАЛЬНЫЙ шаг ПОВОРОТА при слежении, градусы — применяется при большой ошибке " +
             "(мяч у самого края кадра). Между stepDeg и maxStepDeg шаг растёт линейно от величины " +
             "|Kp × error| (и Ki/Kd, если включены) — раньше величина выхода PID вообще ни на " +
             "что не влияла, использовался только её знак, и Kp был бесполезен для управления резкостью.")]
    [Range(0.5f, 15f)]
    public float maxStepDeg = 4f;
    [Tooltip("МИНИМАЛЬНЫЙ шаг НАКЛОНА при слежении, градусы. Диапазон наклона узкий (обычно 10°), " +
             "поэтому шаг обычно чуть меньше, чем у поворота.")]
    [Range(0.25f, 5f)]
    public float tiltStepDeg = 1f;
    [Tooltip("МАКСИМАЛЬНЫЙ шаг НАКЛОНА при слежении, градусы — та же логика, что и maxStepDeg выше.")]
    [Range(0.25f, 8f)]
    public float maxTiltStepDeg = 2.5f;
    [Tooltip("Дискрет реального сервопривода, градусы. Шаг, посчитанный пропорционально ошибке, " +
             "всегда ОКРУГЛЯЕТСЯ ВНИЗ до ближайшего кратного этому значению (но не меньше одного " +
             "дискрета) — реальный серво физически не может встать в произвольный угол, только " +
             "в одно из дискретных положений; округление именно вниз, а не вверх — чтобы шаг " +
             "никогда не перелетал то, что просил регулятор.")]
    [Range(0.1f, 2f)]
    public float stepQuantumDeg = 0.5f;
    [Tooltip("Минимальный интервал между шагами ПРИ СЛЕЖЕНИИ, сек (общий для обеих осей — " +
             "реальный серво-контроллер обычно обновляет оба канала одним тактом). " +
             "При 0.05 сек это ~20 шагов/сек потолок.")]
    [Range(0.01f, 0.5f)]
    public float stepIntervalSeconds = 0.05f;

    [Header("Скорость ПОИСКА при потере мяча (отдельно от слежения)")]
    [Tooltip("Шаг ПОВОРОТА во время поиска (качели), градусы. Больше, чем stepDeg — " +
             "при поиске точность не нужна, важна скорость обзора всего диапазона.")]
    [Range(0.5f, 15f)]
    public float searchStepDeg = 4f;
    [Tooltip("±диапазон ПОИСКА по повороту (yaw), градусы — УЖЕ, чем полный maxAngleDeg " +
             "(реальный физический предел камеры, 30°). УЗ-датчик механически привязан к yaw " +
             "(см. VirtualSensors.centerPoint), поэтому чем дольше камера ищет мяч далеко от " +
             "вперёд, тем дольше УЗ не смотрит по курсу движения. Узкий диапазон поиска держит " +
             "камеру ближе к вперёд большую часть времени, не отнимая у СЛЕЖЕНИЯ полный физический " +
             "диапазон — если мяч реально нашёлся сбоку, слежение всё равно довернёт до maxAngleDeg.")]
    [Range(1f, 45f)]
    public float searchYawMaxDeg = 15f;
    [Tooltip("Шаг НАКЛОНА во время поиска (зигзаг вверх-вниз), градусы. Диапазон поиска " +
             "по наклону (searchTiltMinDeg..searchTiltMaxDeg) узкий, поэтому шаг небольшой — " +
             "зигзаг должен успевать несколько раз качнуться, пока идёт один проход по yaw.")]
    [Range(0.25f, 5f)]
    public float searchTiltStepDeg = 2f;
    [Tooltip("Интервал между шагами ВО ВРЕМЯ ПОИСКА, сек (общий для обеих осей). Меньше, " +
             "чем stepIntervalSeconds — камера шагает чаще, поиск идёт быстрее.")]
    [Range(0.005f, 0.2f)]
    public float searchStepIntervalSeconds = 0.02f;
    [Tooltip("Нижний предел наклона ВО ВРЕМЯ ПОИСКА (уже, чем minTiltDeg слежения — " +
             "поиск специально не задирает камеру в крайние положения, обшаривает " +
             "более узкую, вероятную по высоте зону).")]
    public float searchTiltMinDeg = -5f;
    [Tooltip("Верхний предел наклона ВО ВРЕМЯ ПОИСКА.")]
    public float searchTiltMaxDeg = 0f;

    [Header("Слежение за мячом (PID) — ПОВОРОТ")]
    [Tooltip("Пропорциональный коэффициент поворота. Больше — резче реагирует на смещение мяча от центра.")]
    public float kP = 1.6f;
    public float kI = 0f;
    public float kD = 0f;
    [Tooltip("Мёртвая зона по горизонтальной ошибке (-1..1, 0 = центр). Внутри неё шаг " +
             "не делается — иначе камера будет дрожать туда-сюда ровно в центре из-за шума YOLO.")]
    [Range(0f, 0.3f)]
    public float centerDeadband = 0.03f;

    [Header("Слежение за мячом (PID) — НАКЛОН")]
    [Tooltip("Пропорциональный коэффициент наклона.")]
    public float tiltKp = 1.6f;
    public float tiltKi = 0f;
    public float tiltKd = 0f;
    [Tooltip("Мёртвая зона по вертикальной ошибке (-1..1, 0 = центр).")]
    [Range(0f, 0.3f)]
    public float tiltDeadband = 0.03f;

    [Header("Поиск мяча при потере")]
    [Tooltip("Сколько секунд камера просто ждёт на месте (смотрит в последнее известное " +
             "направление), прежде чем начать активный поиск.")]
    public float holdLastKnownSeconds = 1.0f;
    [Tooltip("Пауза на каждом крайнем положении ПО ПОВОРОТУ (±maxAngleDeg) перед разворотом " +
             "в другую сторону, сек. Наклон паузу не делает — он просто зигзагует непрерывно, " +
             "пока идёт поиск по повороту (по замыслу это как раз и создаёт 'зигзаг').")]
    public float searchPauseAtExtremes = 0.3f;

    /// <summary>Текущий угол ПОВОРОТА камеры относительно корпуса, градусы (0 = прямо вперёд).
    /// Читает RobotBrain для наблюдения сети и для награды bodyCameraAlignmentBonus.</summary>
    public float CurrentAngleDeg { get; private set; }

    /// <summary>Текущий угол НАКЛОНА камеры, градусы (0 = уровень горизонта, отрицательные — вниз).
    /// Читает RobotBrain для наблюдения сети.</summary>
    public float CurrentTiltDeg { get; private set; }

    /// <summary>true = камера потеряла мяч и качается в поиске (см. класс-комментарий, пункт 2),
    /// false = уверенно следит за мячом (или мяч только что появился и это первый тик). Читает
    /// RobotBrain как наблюдение — сеть должна знать РЕЖИМ камеры, а не только её текущий угол:
    /// "камера мечется в поиске" и "камера уверенно ведёт цель" по одному лишь углу неразличимы.</summary>
    public bool IsSearching => _searching;

    PidController _pidYaw;
    PidController _pidTilt;

    float _timeSinceStep;
    float _lostTimer;
    bool  _searching;

    int   _searchDir = 1;             // направление качелей по yaw: +1/-1
    float _searchPauseTimer;
    float _lastKnownAngleSign = 1f;   // в какую сторону по yaw искать сначала

    int   _searchTiltDir = -1;        // направление зигзага по tilt: начинаем движение вниз

    void Awake()
    {
        _pidYaw  = new PidController(kP, kI, kD);
        _pidTilt = new PidController(tiltKp, tiltKi, tiltKd);
    }

    /// <summary>
    /// Вызывается каждый физический тик из RobotBrain.FixedUpdate() — НЕ привязано
    /// к decision period ML-Agents, ровно как и работала бы реальная прошивка серво.
    /// </summary>
    /// <param name="ballVisible">Видит ли робот мяч ПРЯМО СЕЙЧАС (с учётом burst dropout).</param>
    /// <param name="rawHorizontalAngle">"Сырой" угол смещения мяча по горизонтали от центра
    /// кадра, -1 (лево) .. +1 (право).</param>
    /// <param name="rawVerticalAngle">"Сырой" угол смещения мяча по вертикали от центра кадра,
    /// -1 (верх) .. +1 (низ) — соглашение как в SimulatedYoloCamera/RealVision.</param>
    /// <param name="dt">Time.fixedDeltaTime.</param>
    public void Tick(bool ballVisible, float rawHorizontalAngle, float rawVerticalAngle, float dt)
    {
        _timeSinceStep += dt;

        if (ballVisible)
        {
            _lostTimer = 0f;
            _searching = false;
            _searchPauseTimer = 0f;
            _lastKnownAngleSign = rawHorizontalAngle >= 0f ? 1f : -1f;

            float errorH = rawHorizontalAngle;
            float errorV = rawVerticalAngle;

            bool needStepH = Mathf.Abs(errorH) >= centerDeadband;
            bool needStepV = Mathf.Abs(errorV) >= tiltDeadband;

            // Не копим интеграл там, где и так в допуске по этой оси.
            if (!needStepH) _pidYaw.Reset();
            if (!needStepV) _pidTilt.Reset();

            if (!needStepH && !needStepV) return; // обе оси в допуске — делать нечего

            if (_timeSinceStep < stepIntervalSeconds) return; // физический предел частоты шагов
            _timeSinceStep = 0f;

            if (needStepH)
            {
                float pidOutH = _pidYaw.Update(errorH, stepIntervalSeconds);
                float stepH = QuantizedProportionalStep(pidOutH, kP, stepDeg, maxStepDeg);
                ApplyYawStep(Mathf.Sign(pidOutH), stepH);
            }

            if (needStepV)
            {
                float pidOutV = _pidTilt.Update(errorV, stepIntervalSeconds);
                float stepV = QuantizedProportionalStep(pidOutV, tiltKp, tiltStepDeg, maxTiltStepDeg);
                // errorV положительный = мяч НИЖЕ центра кадра (соглашение из SimulatedYoloCamera).
                // ПОДТВЕРЖДЕНО ЭМПИРИЧЕСКИ (на практике): с минусом камера при приближении мяча
                // (мяч уходит вниз кадра) задирала взгляд ВВЕРХ вместо наклона вниз — знак был
                // перепутан. Шаг теперь берётся С ТЕМ ЖЕ знаком, что и pidOutV, без инверсии.
                ApplyTiltStep(Mathf.Sign(pidOutV), stepV);
            }
        }
        else
        {
            _lostTimer += dt;
            if (_lostTimer < holdLastKnownSeconds)
                return; // держим камеру там, где было последнее известное направление

            if (!_searching)
            {
                _searching = true;
                // Сначала ищем в ту сторону, где мяч видели последний раз — не наугад.
                _searchDir = _lastKnownAngleSign >= 0f ? 1 : -1;
                _searchPauseTimer = 0f;
            }

            // Поиск использует СВОИ, более быстрые параметры — при поиске точность не нужна,
            // важна скорость обзора всего диапазона по обеим осям.
            if (_timeSinceStep < searchStepIntervalSeconds) return;
            _timeSinceStep = 0f;

            // --- Поворот: качели с паузой на краях. Поиск использует УЗКИЙ searchYawMaxDeg,
            // а не полный физический maxAngleDeg — держит камеру (и жёстко привязанный к её
            // yaw УЗ-датчик) ближе к вперёд большую часть времени поиска. Слежение за уже
            // НАЙДЕННЫМ мячом (ветка выше) по-прежнему пользуется полным maxAngleDeg. ---
            bool atExtreme = (_searchDir > 0 && CurrentAngleDeg >= searchYawMaxDeg - 0.01f)
                           || (_searchDir < 0 && CurrentAngleDeg <= -searchYawMaxDeg + 0.01f);
            if (atExtreme)
            {
                _searchPauseTimer += searchStepIntervalSeconds;
                if (_searchPauseTimer >= searchPauseAtExtremes)
                {
                    _searchDir *= -1;
                    _searchPauseTimer = 0f;
                }
            }
            else
            {
                ApplyYawStep(_searchDir, searchStepDeg, -searchYawMaxDeg, searchYawMaxDeg);
            }

            // --- Наклон: непрерывный зигзаг между searchTiltMinDeg/MaxDeg, БЕЗ паузы —
            // именно это и создаёт "зигзагом", пока поворот идёт своими качелями отдельно. ---
            bool tiltAtExtreme = (_searchTiltDir > 0 && CurrentTiltDeg >= searchTiltMaxDeg - 0.01f)
                               || (_searchTiltDir < 0 && CurrentTiltDeg <= searchTiltMinDeg + 0.01f);
            if (tiltAtExtreme) _searchTiltDir *= -1;
            ApplyTiltStep(_searchTiltDir, searchTiltStepDeg, searchTiltMinDeg, searchTiltMaxDeg);
        }
    }

    /// <summary>
    /// Величина шага сервопривода ПРОПОРЦИОНАЛЬНА ошибке — |pidOut|/kp даёт нормированную
    /// 0..1 "силу" ошибки (при Ki=Kd=0, как по умолчанию, это ровно |error|), которая линейно
    /// превращается в шаг между minStep и maxStep. Результат округляется ВНИЗ до ближайшего
    /// кратного stepQuantumDeg — реальный серво умеет только дискретные положения, а округление
    /// именно вниз (не вверх) не даёт шагу перелетать то, что попросил регулятор.
    /// </summary>
    float QuantizedProportionalStep(float pidOut, float kp, float minStep, float maxStep)
    {
        float mag = kp > 0.0001f ? Mathf.Clamp01(Mathf.Abs(pidOut) / kp) : 0f;
        float raw = Mathf.Lerp(minStep, maxStep, mag);
        float quantized = Mathf.Floor(raw / stepQuantumDeg) * stepQuantumDeg;
        return Mathf.Max(stepQuantumDeg, quantized);
    }

    void ApplyYawStep(float direction, float stepSizeDeg)
    {
        ApplyYawStep(direction, stepSizeDeg, -maxAngleDeg, maxAngleDeg);
    }

    void ApplyYawStep(float direction, float stepSizeDeg, float clampMin, float clampMax)
    {
        if (direction == 0f) return;
        CurrentAngleDeg = Mathf.Clamp(CurrentAngleDeg + Mathf.Sign(direction) * stepSizeDeg, clampMin, clampMax);
        ApplyRotation();
    }

    void ApplyTiltStep(float direction, float stepSizeDeg)
    {
        ApplyTiltStep(direction, stepSizeDeg, minTiltDeg, maxTiltDeg);
    }

    void ApplyTiltStep(float direction, float stepSizeDeg, float clampMin, float clampMax)
    {
        if (direction == 0f) return;
        CurrentTiltDeg = Mathf.Clamp(CurrentTiltDeg + Mathf.Sign(direction) * stepSizeDeg, clampMin, clampMax);
        ApplyRotation();
    }

    void ApplyRotation()
    {
        if (panPivot != null && tiltPivot != null)
        {
            // Раздельная схема: yaw только на внешнем pivot, tilt только на внутреннем.
            // Всё, что физически прикреплено к panPivot (например, VirtualSensors.centerPoint
            // для УЗ-датчика, если он на реальном роботе крутится вместе с камерой по
            // горизонтали, но не наклоняется) — получит только горизонтальный поворот.
            panPivot.localRotation  = Quaternion.Euler(0f, CurrentAngleDeg, 0f);
            tiltPivot.localRotation = Quaternion.Euler(CurrentTiltDeg, 0f, 0f);
        }
        else if (cameraServo != null)
        {
            // Фолбэк: старая однотрансформенная схема (оба угла на одном объекте) —
            // используется, если раздельные pivot'ы не настроены.
            cameraServo.localRotation = Quaternion.Euler(CurrentTiltDeg, CurrentAngleDeg, 0f);
        }
    }

    /// <summary>Полный сброс состояния — вызывай в OnEpisodeBegin.</summary>
    public void ResetState()
    {
        CurrentAngleDeg = 0f;
        CurrentTiltDeg  = 0f;
        _lostTimer = 0f;
        _searching = false;
        _searchPauseTimer = 0f;
        _timeSinceStep = 0f;
        _searchDir = 1;
        _searchTiltDir = -1;
        _lastKnownAngleSign = 1f;
        _pidYaw?.Reset();
        _pidTilt?.Reset();
        if (panPivot != null) panPivot.localRotation = Quaternion.identity;
        if (tiltPivot != null) tiltPivot.localRotation = Quaternion.identity;
        if (panPivot == null && tiltPivot == null && cameraServo != null)
            cameraServo.localRotation = Quaternion.identity;
    }
}