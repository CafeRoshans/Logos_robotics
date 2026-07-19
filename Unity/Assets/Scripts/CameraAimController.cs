using UnityEngine;

/// <summary>
/// Автономное управление камерой — сеть больше НЕ управляет камерой напрямую. Вместо этого:
///   1) Пока мяч виден — P(ID)-регулятор держит его в центре кадра.
///   2) Как только мяч теряется — камера сначала "держит взгляд" на последнем известном
///      направлении (holdLastKnownSeconds), а затем начинает поиск: качели между
///      -maxAngleDeg и +maxAngleDeg, СНАЧАЛА в ту сторону, где мяч видели последний раз.
///   3) Движение камеры всегда ДИСКРЕТНО — шаг stepDeg (1-2°), не чаще, чем раз в
///      stepIntervalSeconds. Это не костыль поверх PID, а жёсткое физическое ограничение:
///      реальный дешёвый сервопривод физически не может двигаться плавнее и быстрее.
///      Именно поэтому сеть (RL), управляя углом напрямую, физически не могла научиться
///      двигать камеру плавно — плавности взяться неоткуда, если реальное железо на неё
///      не способно. Теперь плавность и не требуется: PID работает НАД дискретным шагом
///      (выбирает, В КАКУЮ сторону шагнуть на этом тике), а не пытается выдать
///      непрерывный угол, который потом придётся квантовать.
///
/// Работает независимо от decision period ML-Agents — вызывается из RobotBrain.FixedUpdate()
/// каждый физический тик, как и положено автономной "прошивке" реального сервопривода.
/// Сеть получает угол камеры (CurrentAngleDeg) как наблюдение и учится доворачивать КОРПУС
/// туда же, куда сейчас смотрит камера (см. bodyCameraAlignmentBonus в RobotBrain) —
/// это единственное, чему сеть теперь учится в связке с камерой.
/// </summary>
public class CameraAimController : MonoBehaviour
{
    [Header("Ссылка на сервопривод")]
    [Tooltip("Transform, вокруг Y которого физически крутится камера. Обычно дочерний " +
             "объект корпуса робота.")]
    public Transform cameraServo;

    [Header("Пределы")]
    [Tooltip("Максимальный угол отклонения камеры от корпуса, ± градусы. Синхронизируется " +
             "с RobotBrain.cameraServoMaxAngle автоматически в Initialize() — не меняй " +
             "тут вручную по отдельности, будет рассинхрон с наблюдением сети. " +
             "Реальная камера GFS-X: <45° в каждую сторону.")]
    public float maxAngleDeg = 45f;

    [Header("Дискретный шаг (эмуляция реального сервопривода)")]
    [Tooltip("Величина одного физического шага камеры ПРИ СЛЕЖЕНИИ за мячом, градусы. " +
             "Референс реального железа: 1-2°. Маленький шаг — нужна точность удержания в центре.")]
    [Range(0.5f, 5f)]
    public float stepDeg = 1.5f;
    [Tooltip("Минимальный интервал между шагами ПРИ СЛЕЖЕНИИ, сек. Реальный сервопривод не может " +
             "переставляться чаще этого — при 0.05 сек это ~20 шагов/сек потолок.")]
    [Range(0.01f, 0.5f)]
    public float stepIntervalSeconds = 0.05f;

    [Header("Скорость ПОИСКА при потере мяча (отдельно от слежения)")]
    [Tooltip("Шаг камеры ВО ВРЕМЯ ПОИСКА (качели), градусы. Больше, чем stepDeg — при поиске " +
             "точность не нужна, важна скорость обзора всего диапазона.")]
    [Range(0.5f, 15f)]
    public float searchStepDeg = 4f;
    [Tooltip("Интервал между шагами ВО ВРЕМЯ ПОИСКА, сек. Меньше, чем stepIntervalSeconds — " +
             "камера шагает чаще, поиск идёт быстрее. Комбинируется с searchStepDeg: " +
             "итоговая угловая скорость поиска = searchStepDeg / searchStepIntervalSeconds.")]
    [Range(0.005f, 0.2f)]
    public float searchStepIntervalSeconds = 0.02f;

    [Header("Слежение за мячом (PID)")]
    [Tooltip("Пропорциональный коэффициент. Больше — резче реагирует на смещение мяча от центра.")]
    public float kP = 1.6f;
    [Tooltip("Интегральный коэффициент. Обычно 0 — для этой задачи P уже достаточно; " +
             "включай, только если камера стабильно не дотягивает мяч точно до центра.")]
    public float kI = 0f;
    [Tooltip("Дифференциальный коэффициент. Обычно 0 — гасит перелёты при быстро " +
             "движущемся мяче, но может усиливать шум детекции.")]
    public float kD = 0f;
    [Tooltip("Мёртвая зона по ошибке (мяч уже в кадре, -1..1, 0 = центр). Внутри неё шаг " +
             "не делается — иначе камера будет дрожать туда-сюда ровно в центре из-за шума YOLO.")]
    [Range(0f, 0.3f)]
    public float centerDeadband = 0.03f;

    [Header("Поиск мяча при потере")]
    [Tooltip("Сколько секунд камера просто ждёт на месте (смотрит в последнее известное " +
             "направление), прежде чем начать активный поиск качелями.")]
    public float holdLastKnownSeconds = 1.0f;
    [Tooltip("Пауза на каждом крайнем положении (±maxAngleDeg) перед разворотом в другую " +
             "сторону, сек. Без паузы качели выглядят рывком на границе диапазона.")]
    public float searchPauseAtExtremes = 0.3f;

    /// <summary>Текущий угол камеры относительно корпуса, градусы. Читает RobotBrain
    /// для наблюдения сети и для награды bodyCameraAlignmentBonus.</summary>
    public float CurrentAngleDeg { get; private set; }

    PidController _pid;
    float _timeSinceStep;
    float _lostTimer;
    bool  _searching;
    int   _searchDir = 1;
    float _searchPauseTimer;
    float _lastKnownAngleSign = 1f;

    void Awake()
    {
        _pid = new PidController(kP, kI, kD);
    }

    /// <summary>
    /// Вызывается каждый физический тик из RobotBrain.FixedUpdate() — НЕ привязано
    /// к decision period ML-Agents, ровно как и работала бы реальная прошивка серво.
    /// </summary>
    /// <param name="ballVisible">Видит ли робот мяч ПРЯМО СЕЙЧАС (с учётом burst dropout —
    /// та же видимость, что использует и RL-наблюдение, для физической согласованности).</param>
    /// <param name="rawHorizontalAngle">"Сырой" (без синтетического шума для RL) угол
    /// смещения мяча от центра кадра, -1 (лево) .. +1 (право) — соглашение как в RealVision.cs.</param>
    /// <param name="dt">Time.fixedDeltaTime.</param>
    public void Tick(bool ballVisible, float rawHorizontalAngle, float dt)
    {
        _timeSinceStep += dt;

        if (ballVisible)
        {
            _lostTimer = 0f;
            _searching = false;
            _searchPauseTimer = 0f;
            _lastKnownAngleSign = rawHorizontalAngle >= 0f ? 1f : -1f;

            float error = rawHorizontalAngle;
            if (Mathf.Abs(error) < centerDeadband)
            {
                _pid.Reset(); // не копим интеграл, пока и так в допуске
                return;
            }

            if (_timeSinceStep < stepIntervalSeconds) return; // физический предел частоты шагов
            _timeSinceStep = 0f;

            float pidOut = _pid.Update(error, stepIntervalSeconds);
            ApplyStep(Mathf.Sign(pidOut), stepDeg);
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

            // Поиск использует СВОИ, более быстрые параметры (searchStepDeg/searchStepIntervalSeconds) —
            // при поиске точность не нужна, важна скорость обзора всего диапазона.
            if (_timeSinceStep < searchStepIntervalSeconds) return;
            _timeSinceStep = 0f;

            bool atExtreme = (_searchDir > 0 && CurrentAngleDeg >= maxAngleDeg - 0.01f)
                           || (_searchDir < 0 && CurrentAngleDeg <= -maxAngleDeg + 0.01f);
            if (atExtreme)
            {
                _searchPauseTimer += searchStepIntervalSeconds;
                if (_searchPauseTimer >= searchPauseAtExtremes)
                {
                    _searchDir *= -1;
                    _searchPauseTimer = 0f;
                }
                return; // стоим на краю (либо ещё копим паузу, либо только что развернулись)
            }

            ApplyStep(_searchDir, searchStepDeg);
        }
    }

    void ApplyStep(float direction, float stepSizeDeg)
    {
        if (direction == 0f) return;
        CurrentAngleDeg = Mathf.Clamp(CurrentAngleDeg + Mathf.Sign(direction) * stepSizeDeg, -maxAngleDeg, maxAngleDeg);
        if (cameraServo != null)
            cameraServo.localRotation = Quaternion.Euler(0f, CurrentAngleDeg, 0f);
    }

    /// <summary>Полный сброс состояния — вызывай в OnEpisodeBegin.</summary>
    public void ResetState()
    {
        CurrentAngleDeg = 0f;
        _lostTimer = 0f;
        _searching = false;
        _searchPauseTimer = 0f;
        _timeSinceStep = 0f;
        _searchDir = 1;
        _lastKnownAngleSign = 1f;
        _pid?.Reset();
        if (cameraServo != null)
            cameraServo.localRotation = Quaternion.identity;
    }
}