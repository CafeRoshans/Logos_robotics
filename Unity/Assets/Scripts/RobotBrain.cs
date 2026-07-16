using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;

/// <summary>
/// ML-Agents агент для робота GFS-X.
/// Настройки Behavior Parameters (задаются в инспекторе):
///   Vector Observation Space Size = 15
///   Stacked Vectors               = 4
///   Continuous Actions            = 3   (leftTrack, rightTrack, camera_servo)
///   Discrete Branches             = 1, размер ветки = 3  (0 = idle, 1 = grab, 2 = release)
///
/// Важно: на TrackController, которым управляет этот агент, useManualInput должен
/// быть выключен (false) — иначе клавиатура (если Behavior Type != Heuristic и кто-то
/// жмёт клавиши) будет затирать команды, приходящие из OnActionReceived.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class RobotBrain : Agent
{
    [Header("Компоненты робота")]
    public Controller tracks;
    public GripperController gripper;
    public VirtualSensors sensors;
    public SimulatedYoloCamera yolo;
    [Tooltip("Transform, вокруг Y которого крутится камера (сервопривод). Может быть родителем самой камеры.")]
    public Transform cameraServo;
    [Tooltip("Мяч, за которым робот охотится")]
    public Transform targetBall;
    [Tooltip("Спавнер препятствий на этой арене. Если задан — на каждый OnEpisodeBegin " +
             "будет вызван Respawn() и препятствия перераскладываются случайно.")]
    public ObstacleSpawner obstacleSpawner;

    [Header("Сервопривод камеры")]
    [Tooltip("Максимальный угол отклонения камеры ± (градусы)")]
    public float cameraServoMaxAngle = 90f;
    [Tooltip("Скорость поворота сервопривода (град/сек) при полном сигнале")]
    public float cameraServoSpeedDegPerSec = 90f;

    [Header("Границы арены (терминал 'вылетел')")]
    [Tooltip("Смещение центра арены ОТ стартовой позиции робота, в локальных единицах. " +
             "Обычно (0,0,0), если робот стартует по центру арены. Так mult-arena работает автоматически.")]
    public Vector3 arenaCenterOffset = Vector3.zero;
    [Tooltip("Половина размера арены (м). Робот считается вылетевшим, если ушёл дальше.")]
    public Vector3 arenaHalfSize = new Vector3(2f, 1f, 2f);

    [Header("Награды и штрафы")]
    [Tooltip("Множитель награды за сближение с мячом (Δd, м). " +
             "0.05 = за подъезд на 1 м агент получает +0.05. За эпизод 500 шагов " +
             "суммарный вклад distance-reward < ±3, что не забивает +5 за успешный захват.")]
    public float distanceRewardScale   = 0.05f;
    [Tooltip("Порог 'близко' — ниже него награда за сближение усиливается")]
    public float closeDistanceThreshold = 0.5f;
    [Tooltip("Множитель усиления награды за сближение вблизи мяча")]
    public float closeDistanceBonusMul  = 1.5f;
    [Tooltip("Штраф за резкое изменение управляющих сигналов между шагами")]
    public float actionRatePenalty      = 0.0005f;
    [Tooltip("Штраф за критически близкую стену (по ИК/УЗ). За 500 шагов у стены = -1 к эпизоду.")]
    public float wallProximityPenalty   = 0.002f;
    [Tooltip("Бонус за то, что мяч в центре кадра")]
    public float centeringBonus         = 0.005f;
    [Tooltip("Терминальный бонус за успешный захват мяча")]
    public float grabSuccessReward      = 5.0f;
    [Tooltip("Терминальный штраф за вылет за пределы арены")]
    public float outOfArenaPenalty      = -2.0f;
    [Tooltip("Небольшой штраф за каждый шаг — стимулирует скорость решения")]
    public float perStepPenalty         = -0.0005f;

    [Header("Случайный спавн робота и мяча")]
    [Tooltip("Спавнить робота в случайной точке из obstacleSpawner.unusedPoints при каждом эпизоде")]
    public bool randomizeSpawnPositions = true;
    [Tooltip("Минимальное расстояние между роботом и мячом при спавне (м). " +
             "Ставь >= размера мяча + захвата, чтобы робот не заспавнился на мяче.")]
    public float minRobotBallDistance = 0.6f;
    [Tooltip("Рандомизировать поворот робота при спавне (0..360°)")]
    public bool randomizeRobotHeading = true;

    [Header("Спавн мяча — фиксированная точка")]
    [Tooltip("Если задан — мяч ВСЕГДА ставится в эту точку (например, середина одного бортика арены). " +
             "Приоритетнее чем случайный спавн. Если null — берётся случайная точка из unusedPoints.")]
    public Transform ballSpawnPoint;

    [Header("Рандомизация массы мяча")]
    [Tooltip("На каждом эпизоде мяч получает случайную массу в этом диапазоне (кг). " +
             "Помогает обучить полиси быть устойчивой к разной инерции мяча.")]
    public bool randomizeBallMass = true;
    public float ballMassMin = 0.01f;
    public float ballMassMax = 0.10f;

    [Header("Рандомизация массы робота")]
    [Tooltip("На каждом эпизоде масса робота выставляется случайно в этом диапазоне (кг). " +
             "Реальные роботы имеют разный вес — АКБ, крепления, датчики. " +
             "Работает только если Rigidbody НЕ kinematic (иначе mass физически не влияет).")]
    public bool randomizeRobotMass = true;
    public float robotMassMin = 0.8f;
    public float robotMassMax = 1.5f;

    [Header("Рандомизация моторов (асимметрия и потолок скорости)")]
    [Tooltip("Разброс множителя скорости для каждой гусеницы в эпизоде. " +
             "0.85..1.15 = левый/правый борт могут отличаться до ±15%. " +
             "Эмулирует разное состояние редукторов.")]
    public bool  randomizeMotors        = true;
    public float motorSpeedMulMin       = 0.85f;
    public float motorSpeedMulMax       = 1.15f;
    [Tooltip("Разброс максимальной скорости робота (maxLinearCmd), м/сек. " +
             "0.6..1.0 = более-менее сильный/слабый АКБ или трение.")]
    public float robotMaxSpeedMin       = 0.6f;
    public float robotMaxSpeedMax       = 1.0f;
    [Tooltip("Разброс сглаживания разгона PWM. Больше — медленнее реакция мотора.")]
    public float motorPwmStepMin        = 10f;
    public float motorPwmStepMax        = 20f;

    // public float randomizeRobotMass = true;
    // public float robotMassMin = 5.0f;
    // public float robotMassMax = 10.0f;

    [Header("YOLO Burst Dropout — потеря мяча при резких поворотах")]
    [Tooltip("Включить симуляцию смаза YOLO при быстром вращении робота")]
    public bool enableYoloBurstDropout = true;
    [Tooltip("Порог угловой скорости робота (град/сек), после которого камера 'смазывается' и YOLO теряет мяч. " +
             "Для реального GFS-X ~90 деg/сек = быстрый разворот.")]
    public float angularSpeedDropoutThreshold = 90f;
    [Tooltip("Мин. длительность burst dropout в решениях (5 при DecisionPeriod=5 ≈ 0.5 сек)")]
    public int burstDropoutMinSteps = 5;
    [Tooltip("Макс. длительность burst dropout в решениях (15 при DecisionPeriod=5 ≈ 1.5 сек)")]
    public int burstDropoutMaxSteps = 15;

    [Header("Периодическое пересоздание препятствий")]
    [Tooltip("Каждые N эпизодов препятствия перераскладываются в новых точках. " +
             "1 = каждый эпизод (доменная рандомизация каждый раз). " +
             "10000 = одно расположение на 10000 эпизодов подряд (медленный curriculum). " +
             "0 = никогда (расположение фиксируется первым Respawn).")]
    public int obstacleRespawnEveryEpisodes = 1;

    [Header("Страховка на случай, если Max Step в инспекторе Agent не спасает")]
    [Tooltip("Жёсткий лимит на количество вызовов OnActionReceived за эпизод. 0 = выключено.")]
    public int hardEpisodeStepLimit = 3000;
    [Tooltip("Штраф за окончание эпизода по таймауту")]
    public float timeoutPenalty = -0.5f;

    // --- служебные ---
    private Rigidbody _rb;
    // _rb.mass = 5.0f; // default mass for the robot, can be randomized if needed
    private Vector3   _startPosition;
    private Quaternion _startRotation;
    private Vector3   _ballStartPosition;

    private float _cameraServoAngle       = 0f;
    private float _prevDistanceToBall     = -1f;
    private float _prevLeft               = 0f;
    private float _prevRight              = 0f;
    private float _timeSinceLastDetection = 0f;
    private float _lastKnownBallDirection = 0f;
    private int   _episodeStepCount       = 0;

    private int _globalStepCount = 0; // для логирования в Debug.Log, не сбрасывается на OnEpisodeBegin
    private int _episodeCount    = 0; // счётчик эпизодов агента, для периодического пересоздания препятствий

    // Burst dropout: сколько шагов подряд ещё нужно "слепить" YOLO
    private int   _dropoutStepsLeft = 0;
    // Предыдущее значение eulerAngles.y для вычисления угловой скорости
    private float _prevHeadingDeg   = 0f;

    // --- Кастомные метрики за эпизод ---
    private float _rewardDistance = 0f;
    private float _rewardWall     = 0f;
    private float _rewardCenter   = 0f;
    private float _rewardAction   = 0f;
    private float _rewardStep     = 0f;
    private float _rewardTerminal = 0f;
    private int   _framesBallVisible = 0;
    private int   _framesTotal       = 0;
    private float _speedAccum        = 0f;
    private int   _dropoutBurstsCount = 0; // сколько burst dropout произошло за эпизод

    private Queue<float[]> actionBuffer = new Queue<float[]>();
    private int currentActionLatency = 5; 


    public override void Initialize()
    {
        _rb = GetComponent<Rigidbody>();
        _startPosition = transform.position;
        _startRotation = transform.rotation;
        if (targetBall != null) _ballStartPosition = targetBall.position;
    }

    public override void OnEpisodeBegin()
    {
        // 1. Препятствия перераскладываем ПЕРВЫМИ (не каждый эпизод, а раз в N).
        //    Robot placement в свободных точках всё равно работает — unusedPoints
        //    заполняется в Respawn и потом остаётся между эпизодами.
        //    Первый эпизод (episodeCount = 0) всегда пересоздаёт, чтобы unusedPoints
        //    было чем-то заполнено.
        bool shouldRespawn = obstacleRespawnEveryEpisodes > 0
            ? (_episodeCount % obstacleRespawnEveryEpisodes == 0)
            : (_episodeCount == 0);
        if (shouldRespawn && obstacleSpawner != null)
        {
            obstacleSpawner.Respawn();
            Debug.Log($"[{name}] Episode #{_episodeCount}: RESPAWN препятствий (every {obstacleRespawnEveryEpisodes}).");
        }
        else if (obstacleRespawnEveryEpisodes > 1 && _episodeCount % 5 == 0)
        {
            // Каждые 5 эпизодов пишем что происходит — иначе непонятно, работает ли счётчик
            Debug.Log($"[{name}] Episode #{_episodeCount}: пропускаю respawn (следующий через {obstacleRespawnEveryEpisodes - (_episodeCount % obstacleRespawnEveryEpisodes)}).");
        }

        _episodeCount++;

        // 2. Разжимаем клешню (если был мяч) — до перемещения позиций
        if (gripper != null && gripper.isHolding) gripper.Release();
        if (gripper != null) gripper.grabCommand = false;

        // 3. Определяем позиции робота и мяча
        Vector3 robotPos = _startPosition;
        Quaternion robotRot = _startRotation;
        Vector3 ballPos = _ballStartPosition;

        // --- Позиция робота: случайная из свободных точек (если включено, каждый эпизод) ---
        if (randomizeSpawnPositions && obstacleSpawner != null
            && obstacleSpawner.unusedPoints.Count >= 1)
        {
            var pts = obstacleSpawner.unusedPoints;
            int robotIdx = Random.Range(0, pts.Count);
            Vector3 candRobotPos = pts[robotIdx].position;
            if (IsValid(candRobotPos)) robotPos = candRobotPos;
            else Debug.LogWarning($"[RobotBrain] Точка '{pts[robotIdx].name}' даёт невалидную позицию {candRobotPos}, использую _startPosition.");

            if (randomizeRobotHeading)
                robotRot = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        }

        // --- Позиция мяча: фиксированная точка (например, середина бортика) ---
        if (ballSpawnPoint != null)
        {
            Vector3 fixedBallPos = ballSpawnPoint.position;
            if (IsValid(fixedBallPos)) ballPos = fixedBallPos;
            else Debug.LogWarning($"[RobotBrain] ballSpawnPoint '{ballSpawnPoint.name}' даёт невалидную позицию {fixedBallPos}.");

            // Если мяч слишком близко к роботу (робот заспавнился рядом с бортиком),
            // отодвинем робота: возьмём другую случайную точку с достаточной дистанцией.
            if (randomizeSpawnPositions && obstacleSpawner != null
                && obstacleSpawner.unusedPoints.Count >= 1
                && Vector3.Distance(robotPos, ballPos) < minRobotBallDistance)
            {
                var pts = obstacleSpawner.unusedPoints;
                for (int tries = 0; tries < 20; tries++)
                {
                    int cand = Random.Range(0, pts.Count);
                    Vector3 cp = pts[cand].position;
                    if (!IsValid(cp)) continue;
                    if (Vector3.Distance(cp, ballPos) < minRobotBallDistance) continue;
                    robotPos = cp;
                    break;
                }
            }
        }

        // Финальная проверка — если что-то дало NaN, используем безопасные значения
        if (!IsValid(robotPos)) robotPos = _startPosition;
        if (!IsValid(ballPos))  ballPos  = _ballStartPosition;

        // 4. Ставим робота
        transform.SetPositionAndRotation(robotPos, robotRot);
        if (_rb != null && !_rb.isKinematic)
        {
            _rb.linearVelocity  = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }

        // рандомим массу робота 
        if(_rb != null && !_rb.isKinematic && randomizeRobotMass)
        {
            float robotMass = Gaussian(2.5f); // Example range for robot mass
            _rb.mass = Mathf.Max(0.001f, robotMass); // Ensure mass is not zero or negative
        }

        if (tracks != null) tracks.SetTrackInputs(0f, 0f);

        // 5. Ставим мяч + рандомизируем массу
        if (targetBall != null)
        {
            targetBall.position = ballPos;
            var brb = targetBall.GetComponent<Rigidbody>();
            if (brb != null)
            {
                if (!brb.isKinematic)
                {
                    brb.linearVelocity  = Vector3.zero;
                    brb.angularVelocity = Vector3.zero;
                }
                // Рандомизация массы — доменная рандомизация для sim-to-real.
                // Работает и на kinematic мяче: масса запомнится к моменту "разжатия" клешни.
                if (randomizeBallMass)
                {
                    float m = Gaussian(0.1f); // Example range for ball mass
                    brb.mass = Mathf.Max(0.001f, m); // страховка от нуля/отрицательного
                }
            }
        }

        // // 5.5. Рандомизация массы робота и параметров моторов
        // //      (доменная рандомизация — те же реальные роботы отличаются АКБ, редукторами)
        // if (randomizeRobotMass && _rb != null && !_rb.isKinematic)
        // {
        //     _rb.mass = Mathf.Max(0.01f, Random.Range(robotMassMin, robotMassMax));
        // }

        if (randomizeMotors && tracks != null)
        {
            // Асимметрия левого/правого борта — эмулирует разный износ редукторов
            tracks.leftSpeedMul  = Random.Range(motorSpeedMulMin, motorSpeedMulMax);
            tracks.rightSpeedMul = Random.Range(motorSpeedMulMin, motorSpeedMulMax);

            // Максимальная скорость робота — эмулирует АКБ или трение
            tracks.maxLinearCmd = Mathf.Max(0.05f, Random.Range(robotMaxSpeedMin, robotMaxSpeedMax));

            // Сглаживание разгона PWM — реакция моторов
            tracks.maxPwmStep = Mathf.Max(1f, Random.Range(motorPwmStepMin, motorPwmStepMax));
        }

        // Сервопривод камеры
        _cameraServoAngle = 0f;
        if (cameraServo != null)
            cameraServo.localRotation = Quaternion.Euler(0f, 0f, 0f);

        // Служебные переменные наград
        _prevDistanceToBall     = DistanceToBall();
        _prevLeft                = 0f;
        _prevRight               = 0f;
        _timeSinceLastDetection = 0f;
        _lastKnownBallDirection = 0f;
        _episodeStepCount       = 0;

        // Сброс кастомных счётчиков
        _rewardDistance = 0f;
        _rewardWall     = 0f;
        _rewardCenter   = 0f;
        _rewardAction   = 0f;
        _rewardStep     = 0f;
        _rewardTerminal = 0f;
        _framesBallVisible = 0;
        _framesTotal       = 0;
        _speedAccum        = 0f;

        // Burst dropout сбрасываем на начало эпизода
        _dropoutStepsLeft  = 0;
        _prevHeadingDeg    = transform.eulerAngles.y;

        // simulatin latenct of real ros2 connection
        currentActionLatency = (int)UnityEngine.Random.Range(8.14f, 12.14f);
        actionBuffer.Clear();

        for (int i =0; i < currentActionLatency; i++)
        {
            actionBuffer.Enqueue(new float[] {0f, 0f, 0f});
        }

    }

    // Гауссов шум через Box-Muller
    static float Gaussian(float stddev) {
        float u1 = 1f - Random.value;
        float u2 = 1f - Random.value;
        float z = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
        return z * stddev;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. УЗ нормализованный
        float uzNoisy = sensors.ultrasonicNormalized + Gaussian(0.1f);
        sensor.AddObservation(Mathf.Clamp01(uzNoisy));
        // 2. Левый ИК препятствия
        sensor.AddObservation(sensors != null ? (float)sensors.leftIR : 0f);
        // 3. Правый ИК препятствия
        sensor.AddObservation(sensors != null ? (float)sensors.rightIR : 0f);
        // 4. ИК клешни
        sensor.AddObservation(sensors != null ? (float)sensors.gripperIR : 0f);

        // 5. Горизонтальный угол до мяча по камере (0, если не виден).
        //    ВАЖНО: SeesBallEffective() учитывает burst dropout — если робот
        //    только что резко повернулся, YOLO "смазан" и мяч не виден.
        bool visible = SeesBallEffective();

        sensor.AddObservation(visible ? yolo.horizontalAngle : 0f);
        // 6. Норм. расстояние по камере (1, если не виден)
        sensor.AddObservation(visible ? yolo.normalizedDistance : 1f);
        // 7. Последнее известное направление на мяч
        sensor.AddObservation(_lastKnownBallDirection);
        // 8. Флаг видимости
        sensor.AddObservation(visible ? 1f : 0f);

        // 9. Угол сервопривода камеры, нормализованный
        sensor.AddObservation(Mathf.Clamp(_cameraServoAngle / Mathf.Max(1f, cameraServoMaxAngle), -1f, 1f));

        // 10. hasBall
        sensor.AddObservation(gripper != null && gripper.isHolding ? 1f : 0f);

        // 11..12. Смещение от старта X/Z (нормализованное)
        Vector3 delta = transform.position - _startPosition;
        float norm = Mathf.Max(0.001f, Mathf.Max(arenaHalfSize.x, arenaHalfSize.z));
        sensor.AddObservation(delta.x / norm);
        sensor.AddObservation(delta.z / norm);

        // 13. Heading (курс) робота, нормализованный -1..1
        float heading = transform.eulerAngles.y;
        if (heading > 180f) heading -= 360f;
        sensor.AddObservation(heading / 180f);

        // 14. Скорость робота (м/с)
        sensor.AddObservation(_rb != null ? _rb.linearVelocity.magnitude : 0f);

        // 15. Время с последней детекции мяча (сек)
        sensor.AddObservation(_timeSinceLastDetection);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
            float dbgLeft  = actions.ContinuousActions[0];
            float dbgRight = actions.ContinuousActions[1];
            float dbgCam   = actions.ContinuousActions[2];
            //Debug.Log($"[RobotBrain] OnActionReceived: left={dbgLeft:F3} right={dbgRight:F3} cam={dbgCam:F3} step={_episodeStepCount}");

            _episodeStepCount++;
            _globalStepCount++;
            UpdateBurstDropout();

        if (hardEpisodeStepLimit > 0 && _episodeStepCount >= hardEpisodeStepLimit)
        {
            AddReward(timeoutPenalty);
            _rewardTerminal += timeoutPenalty;
            LogEpisodeStats(success: false);
            EndEpisode();
            return;
        }

        // Считываем свежие сигналы от сети
        float freshLeft  = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float freshRight = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float freshCam   = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        int   gripAct    = actions.DiscreteActions[0]; // 0 = idle, 1 = grab, 2 = release

        // Пропускаем непрерывные сигналы через FIFO-буфер задержки — эмулируем
        // латентность ROS2/сети/моторов. Кладём свежее в хвост, забираем самое
        // старое из головы. Дискретный gripAct не задерживаем — команды клешне
        // на реальном роботе идут отдельным каналом и обычно быстрее.
        actionBuffer.Enqueue(new float[] { freshLeft, freshRight, freshCam });
        float[] delayed = actionBuffer.Count > 0
            ? actionBuffer.Dequeue()
            : new float[] { 0f, 0f, 0f };

        float leftTrack  = delayed[0];
        float rightTrack = delayed[1];
        float camSignal  = delayed[2];

        // 1. Движение — прокидываем в TrackController напрямую по двум гусеницам
        if (tracks != null)
        {
            tracks.SetTrackInputs(leftTrack, rightTrack);
        }

        // 2. Сервопривод камеры (интегрируем сигнал в угол)
        _cameraServoAngle = Mathf.Clamp(
            _cameraServoAngle + camSignal * cameraServoSpeedDegPerSec * Time.deltaTime,
            -cameraServoMaxAngle, cameraServoMaxAngle);
        if (cameraServo != null)
            cameraServo.localRotation = Quaternion.Euler(0f, _cameraServoAngle, 0f);

        // 3. Клешня
        if (gripper != null)
        {
            if      (gripAct == 1) gripper.grabCommand = true;
            else if (gripAct == 2) gripper.grabCommand = false;
            // gripAct == 0 — не трогаем состояние
        }

        // 4. Обновление служебных переменных детекции — используем эффективную видимость
        if (SeesBallEffective())
        {
            _lastKnownBallDirection = yolo.horizontalAngle;
            _timeSinceLastDetection = 0f;
        }
        else
        {
            _timeSinceLastDetection += Time.deltaTime;
        }

        // 5. Награды
        ComputeRewards(leftTrack, rightTrack);

        _prevLeft  = leftTrack;
        _prevRight = rightTrack;
    }

    void ComputeRewards(float leftTrack, float rightTrack)
    {
        // Кадровые счётчики для метрик — считаем эффективную видимость,
        // чтобы Custom/BallVisibleFraction отражал реальный сигнал, доходящий до модели.
        _framesTotal++;
        bool ballVisible = SeesBallEffective();
        if (ballVisible) _framesBallVisible++;
        if (_rb != null) _speedAccum += _rb.linearVelocity.magnitude;

        // а) Сближение с мячом (delta distance)
        float curDist = DistanceToBall();
        if (_prevDistanceToBall > 0f && curDist > 0f)
        {
            float delta = _prevDistanceToBall - curDist; // + = приблизились
            delta = Mathf.Clamp(delta, -0.5f, 0.5f);
            float mul   = (curDist < closeDistanceThreshold) ? closeDistanceBonusMul : 1f;
            float rDist = delta * distanceRewardScale * mul;
            AddReward(rDist); _rewardDistance += rDist;
        }
        _prevDistanceToBall = curDist;

        // б) Штраф за резкость управления
        float dLeft  = Mathf.Abs(leftTrack  - _prevLeft);
        float dRight = Mathf.Abs(rightTrack - _prevRight);
        float rAct = -(dLeft + dRight) * actionRatePenalty;
        AddReward(rAct); _rewardAction += rAct;

        // в) Бонус за центрирование мяча в кадре
        if (ballVisible)
        {
            float centered = 1f - Mathf.Abs(yolo.horizontalAngle);
            float rCen = centered * centeringBonus;
            AddReward(rCen); _rewardCenter += rCen;
        }

        // г) Штраф за критически близкие стены
        if (sensors != null)
        {
            float rWall = 0f;
            if (sensors.ultrasonicNormalized < 0.15f) rWall -= wallProximityPenalty;
            if (sensors.leftIR  == 1)                 rWall -= wallProximityPenalty;
            if (sensors.rightIR == 1)                 rWall -= wallProximityPenalty;
            if (rWall != 0f) { AddReward(rWall); _rewardWall += rWall; }
        }

        // д) Мелкий постоянный штраф
        AddReward(perStepPenalty); _rewardStep += perStepPenalty;

        // е) Терминал: успешный захват
        if (gripper != null && gripper.isHolding)
        {
            AddReward(grabSuccessReward);
            _rewardTerminal += grabSuccessReward;
            LogEpisodeStats(success: true);
            EndEpisode();
            return;
        }

        // ж) Терминал: вылет за арену
        Vector3 arenaCenter = _startPosition + arenaCenterOffset;
        Vector3 p = transform.position - arenaCenter;
        if (Mathf.Abs(p.x) > arenaHalfSize.x ||
            Mathf.Abs(p.z) > arenaHalfSize.z ||
            p.y < -arenaHalfSize.y || p.y > arenaHalfSize.y)
        {
            AddReward(outOfArenaPenalty);
            _rewardTerminal += outOfArenaPenalty;
            LogEpisodeStats(success: false);
            EndEpisode();
        }
    }

    /// <summary>
    /// Логирует пер-эпизодные кастомные метрики в Custom/... секцию TensorBoard.
    /// Вызывается перед EndEpisode(). Таймаут-терминал логирует отдельно из OnActionReceived.
    /// </summary>
    void LogEpisodeStats(bool success)
    {
        var s = Academy.Instance.StatsRecorder;

        // Успех эпизода — усредняется во время summary, даёт success rate ∈ [0..1]
        s.Add("Custom/SuccessRate", success ? 1f : 0f);

        // Разбивка накопленной награды за эпизод
        s.Add("Custom/Reward/Distance", _rewardDistance);
        s.Add("Custom/Reward/Wall",     _rewardWall);
        s.Add("Custom/Reward/Center",   _rewardCenter);
        s.Add("Custom/Reward/Action",   _rewardAction);
        s.Add("Custom/Reward/Step",     _rewardStep);
        s.Add("Custom/Reward/Terminal", _rewardTerminal);

        // Доля кадров, когда робот видит мяч (за эпизод)
        if (_framesTotal > 0)
            s.Add("Custom/BallVisibleFraction", (float)_framesBallVisible / _framesTotal);

        // Средняя скорость робота за эпизод
        if (_framesTotal > 0)
            s.Add("Custom/AvgSpeed", _speedAccum / _framesTotal);

        // Heat map: позиция робота относительно центра арены
        Vector3 rel = transform.position - (_startPosition + arenaCenterOffset);
        s.Add("Custom/Position/RelX", rel.x);
        s.Add("Custom/Position/RelZ", rel.z);

        // Сколько раз YOLO "смазался" за эпизод (burst dropout при резких поворотах)
        s.Add("Custom/YoloBurstsPerEpisode", _dropoutBurstsCount);
        _dropoutBurstsCount = 0;

        // Параметры моторов и масс за этот эпизод — видно распределение доменной рандомизации
        if (tracks != null)
        {
            s.Add("Custom/Motors/LeftSpeedMul",  tracks.leftSpeedMul);
            s.Add("Custom/Motors/RightSpeedMul", tracks.rightSpeedMul);
            s.Add("Custom/Motors/Asymmetry",    Mathf.Abs(tracks.leftSpeedMul - tracks.rightSpeedMul));
            s.Add("Custom/Motors/MaxLinearCmd", tracks.maxLinearCmd);
            s.Add("Custom/Motors/MaxPwmStep",   tracks.maxPwmStep);
        }
        if (_rb != null) s.Add("Custom/Mass/Robot", _rb.mass);
        if (targetBall != null)
        {
            var brb = targetBall.GetComponent<Rigidbody>();
            if (brb != null) s.Add("Custom/Mass/Ball", brb.mass);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        var disc = actionsOut.DiscreteActions;

        float left = 0f, right = 0f, c = 0f;
        int   g = 0;

        var kb = Keyboard.current;
        if (kb != null)
        {
            // Те же клавиши, что и дефолт TrackController: W/S — левая, E/D — правая
            left  = kb.wKey.ReadValue() - kb.sKey.ReadValue();
            right = kb.eKey.ReadValue() - kb.dKey.ReadValue();
            // Камера перенесена на I/K, чтобы не конфликтовать с E/D (правая гусеница)
            c = kb.iKey.ReadValue() - kb.kKey.ReadValue();
            if      (kb.spaceKey.isPressed) g = 1;             // grab
            else if (kb.xKey.isPressed)     g = 2;             // release
        }

        cont[0] = left;
        cont[1] = right;
        cont[2] = c;
        disc[0] = g;
    }

    float DistanceToBall()
    {
        if (targetBall == null) return -1f;
        return Vector3.Distance(transform.position, targetBall.position);
    }

    static bool IsValid(Vector3 v)
    {
        return !(float.IsNaN(v.x) || float.IsInfinity(v.x) ||
                 float.IsNaN(v.y) || float.IsInfinity(v.y) ||
                 float.IsNaN(v.z) || float.IsInfinity(v.z));
    }

    /// <summary>
    /// «Реальная» видимость мяча с учётом burst dropout от смаза камеры при вращении.
    /// Используется вместо прямого yolo.isVisible в наблюдениях и наградах.
    /// </summary>
    bool SeesBallEffective()
    {
        if (yolo == null || !yolo.isVisible) return false;
        if (_dropoutStepsLeft > 0) return false;
        return true;
    }

    /// <summary>
    /// Проверяет угловую скорость робота и, если она выше порога, стартует burst dropout.
    /// Вызывается раз за OnActionReceived.
    /// </summary>
    void UpdateBurstDropout()
    {
        // Считаем угловую скорость через дельту heading — Rigidbody kinematic
        // не даёт корректный angularVelocity, поэтому берём вручную.
        float curHeading = transform.eulerAngles.y;
        float deltaDeg = Mathf.DeltaAngle(_prevHeadingDeg, curHeading);
        _prevHeadingDeg = curHeading;

        // Скорость за один decision step, переводим в град/сек:
        // Time.deltaTime × DecisionPeriod — но проще считать по абсолютному значению delta:
        // порог в град/сек = deltaDeg / decisionDeltaTime.
        float decisionDeltaTime = Mathf.Max(0.001f, Time.deltaTime);
        float angularSpeedDegPerSec = Mathf.Abs(deltaDeg) / decisionDeltaTime;

        if (enableYoloBurstDropout && angularSpeedDegPerSec > angularSpeedDropoutThreshold && _dropoutStepsLeft <= 0)
        {
            _dropoutStepsLeft = Random.Range(burstDropoutMinSteps, burstDropoutMaxSteps + 1);
            _dropoutBurstsCount++;
        }

        if (_dropoutStepsLeft > 0) _dropoutStepsLeft--;
    }

    void OnDrawGizmosSelected()
    {
        // Рисуем реальные границы арены — относительно текущего положения робота
        // (в редакторе _startPosition ещё не задан, поэтому берём transform.position).
        Vector3 center = Application.isPlaying
            ? (_startPosition + arenaCenterOffset)
            : (transform.position + arenaCenterOffset);
        Gizmos.color = Color.magenta;
        Gizmos.DrawWireCube(center, arenaHalfSize * 2f);
    }
}