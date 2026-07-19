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
///   Continuous Actions            = 3   (gas [-1..1], steering [-1..1], camera_yaw_target [-1..1])
///   Discrete Branches             = 1, размер ветки = 3  (0 = idle, 1 = grab, 2 = release)
///
/// Важно: на TrackController, которым управляет этот агент, useManualInput должен
/// быть выключен (false) — иначе клавиатура (если Behavior Type != Heuristic и кто-то
/// жмёт клавиши) будет затирать команды, приходящие из OnActionReceived.
///
/// ВАЖНО про скорость: Controller двигает Rigidbody кинематически (MovePosition/MoveRotation),
/// поэтому rb.linearVelocity НЕ обновляется автоматически физическим движком и всегда остаётся
/// (0,0,0). Скорость корпуса здесь считается вручную — по фактической дельте позиции между
/// последовательными вызовами OnActionReceived — и используется и для наблюдения №14,
/// и для штрафа за движение назад (backwardMovementPenalty).
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class RobotBrain : Agent
{
    [Header("Компоненты робота")]
    public Controller tracks;
    public GripperController gripper;
    public VirtualSensors sensors;
    [Tooltip("Симуляция камеры — используется ВО ВРЕМЯ ТРЕНИРОВКИ. Должен быть назначен в инспекторе для каждой арены.")]
    public SimulatedYoloCamera simCam;
    [Tooltip("UDP-приёмник от реальной YOLO — используется ТОЛЬКО когда useRealRobot=true.")]
    public RealVision yolo;
    [Tooltip("Transform, вокруг Y которого крутится камера (сервопривод). Может быть родителем самой камеры.")]
    public Transform cameraServo;
    [Tooltip("Мяч, за которым робот охотится")]
    public Transform targetBall;
    [Tooltip("Спавнер препятствий на этой арене. Если задан — на каждый OnEpisodeBegin " +
             "будет вызван Respawn() и препятствия перераскладываются случайно.")]
    public ObstacleSpawner obstacleSpawner;

    [Header("Реальный робот (ROS)")]
    [Tooltip("Если true — читаем видение из yolo (UDP от реального робота) и шлём команды через rosBridge. " +
             "Если false — читаем из simCam и rosBridge игнорируется (режим тренировки).")]
    public bool useRealRobot = false;
    public ROSBridge rosBridge;

    [Header("Сервопривод камеры")]
    [Tooltip("Максимальный угол отклонения камеры ± (градусы). Камера свободно осматривается в " +
             "широком диапазоне, но награду за центрирование получает только когда мяч видно И корпус " +
             "довёрнут — не даёт стоять и просто крутить камеру.")]
    public float cameraServoMaxAngle = 90f;

    [Tooltip("МАКСИМАЛЬНАЯ скорость сервомотора (град за одно решение). Уменьшение = более " +
             "медленная и плавная камера, не рыскает. 15° — референс. 5-8° — очень плавно, но " +
             "камера долго доходит до цели. Пример: при DecisionPeriod=5 (Time.fixedDeltaTime=0.02): " +
             "15°/decision × 10 decisions/sec = 150°/сек — реалистично для дешёвого сервопривода.")]
    [Range(1f, 30f)]
    public float cameraMaxStepDeg = 8f;

    [Tooltip("EMA-сглаживание target'а от сети. 0.3 = быстрое реагирование, 0.1 = сильное " +
             "сглаживание (шум сети почти не влияет). Меньше = стабильнее камера.")]
    [Range(0.05f, 1f)]
    public float cameraTargetEmaAlpha = 0.15f;

    [Tooltip("Deadband: если сглаженный target находится в этом диапазоне от текущего угла — " +
             "камера НЕ двигается. Больше = ленивее, стабильнее. 0.05 = 4.5° мёртвая зона.")]
    [Range(0f, 0.2f)]
    public float cameraTargetDeadband = 0.05f;

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
    [Tooltip("Коэф. α в exp(α×(1-dist)): α=2 → ×7.4 у мяча, ×1.0 на 1м. Усиливает сигнал у цели без сингулярности.")]
    public float distanceRewardAlpha   = 2f;
    [Tooltip("Радиус переключения Phase1→Phase2 (м). Дальше — delta-reward за сближение. Ближе — slow-approach.")]
    public float closeRadius           = 0.35f;
    [Tooltip("Штраф за резкое изменение gas/steer между шагами. Сглаживает езду.")]
    public float driveRatePenalty  = 0.001f;
    [Tooltip("Штраф за резкое изменение camTarget между шагами. Важнее езды — камера дёргает в реальном мире. ")]
    public float cameraRatePenalty = 0.005f;
    [Tooltip("Штраф за критически близкую стену (по ИК/УЗ). Каждый сенсор, который срабатывает = -этот штраф/шаг.")]
    public float wallProximityPenalty   = 0.01f;

    [Tooltip("Штраф за окончание эпизода по таймауту")]
    public float timeoutPenalty = 1.0f;

    [Header("Столкновение с препятствием")]
    [Tooltip("Тег, который должен быть выставлен на prefab препятствия в Unity Editor. " +
             "Без правильного тега штраф не сработает.")]
    public string obstacleTag              = "Obstacle";
    [Tooltip("Разовый штраф при физическом касании препятствия.")]
    public float  obstacleCollisionPenalty = 1.0f;
    [Tooltip("Завершать эпизод при столкновении с препятствием (сильный сигнал, но короче эпизоды). " +
             "false = только штраф, обучение продолжается.")]
    public bool   endEpisodeOnObstacleHit  = false;

    // wobbleAsymmetryPenalty и steerFlipPenalty удалены — они были нужны, чтобы модель,
    // управляющая напрямую left/right, не ездила зигзагом. С Gas/Steering раскладкой в
    // Controller.Move() симметрия обеспечивается автоматически: gas=1, steer=0 → left=right,
    // а зигзаг стал невозможен на уровне action space.
    [Tooltip("Терминальный бонус за успешный захват своего мяча.")]
    public float grabSuccessReward      = 5.0f;


    [Header("Blind approach — движение вперёд когда мяч НЕ виден")]
    [Tooltip("Бонус за каждый шаг, когда робот едет вперёд, но мяч ещё не виден. " +
             "Стимулирует активный поиск, а не стояние на месте при потере мяча.")]
    public float blindApproachBonus = 0.007f; // изменил с 3 до 7, пока робот имеет малую энтропию мб поможет
    [Tooltip("Минимальная реальная скорость вперёд (м/с) для срабатывания blindApproachBonus")]
    public float blindApproachMinForwardSpeed = 0.05f;

    [Header("Шум наблюдений (доменная рандомизация)")]
    [Tooltip("Амплитуда uniform-шума на УЗ (±). Значение читается из yaml, если useYamlEnvParams=true.")]
    public float ultrasonicNoise       = 0.05f;
    [Tooltip("Амплитуда uniform-шума на vision angle (±)")]
    public float visionAngleNoise      = 0.03f;
    [Tooltip("Амплитуда uniform-шума на vision distance (±). Обычно в 3× больше angle-шума — дистанция шумнее.")]
    public float visionDistanceNoise   = 0.09f;

    [Header("Задержка сенсоров (симуляция ROS2 pipeline)")]
    [Tooltip("На сколько decisions задерживаются показания датчиков. 3 при DecisionPeriod=5 ≈ 300мс. " +
             "Отдельно от actionBuffer, потому что sensor pipeline имеет свою задержку. Читается из yaml.")]
    public int sensorLatencySteps = 3;

    [Header("Yaml environment_parameters")]
    [Tooltip("Читать ball_mass, ball_scale, ultrasonic_noise, vision_noise, sensor_latency, action_latency из config.yaml " +
             "→ environment_parameters. Позволяет менять физику без пересборки билда + поддерживает curriculum.")]
    public bool useYamlEnvParams = true;

    [Header("Automatic gripper (из референса)")]
    [Tooltip("Клешня хватает автоматически по IR-датчику, сеть НЕ управляет grab/release. " +
             "Убирает дискретное action → упрощает action space → быстрее обучение. " +
             "После включения обязательно поставь Behavior Parameters → Discrete Branches = 0 в инспекторе.")]
    public bool useAutomaticGripper = true;

    [Header("Hard-stop после захвата")]
    [Tooltip("После успешного захвата мяча принудительно обнулять команды моторов. " +
             "Иначе робот продолжает крутиться и может уронить мяч.")]
    public bool hardStopOnHold = true;

    [Header("Per-step и арена")]
    [Tooltip("Небольшой штраф за каждый шаг — стимулирует скорость решения.")]
    public float perStepPenalty         = 0.0005f;
    [Tooltip("Терминальный штраф за вылет за пределы арены.")]
    public float outOfArenaPenalty      = 2.0f;

    [Header("Phase 2 — точный подъезд (dist < closeRadius)")]
    [Tooltip("Порог линейной скорости (м/с): ниже = 'медленно' для gentlePlacementBonus.")]
    public float gentleLinearSpeedThresh  = 0.08f;
    [Tooltip("Порог нормированной угловой команды steer (0..1): ниже = 'почти прямо' для gentlePlacementBonus.")]
    public float gentleAngularSpeedThresh = 0.15f;
    [Tooltip("Бонус каждый шаг в Phase2, когда linear+angular ниже порогов (тихий подъезд).")]
    public float gentlePlacementBonus    = 0.008f;
    [Tooltip("Штраф каждый шаг в Phase2, когда скорость превышает порог (пролёт мимо).")]
    public float gentleOverspeedPenalty  = 0.015f;

    [Header("Штраф за движение назад")]
    [Tooltip("Штраф за движение назад (когда корпус реально смещается против transform.forward)")]
    public float backwardMovementPenalty = 0.01f;
    [Tooltip("Мёртвая зона по скорости (м/с), ниже которой направление не штрафуем (шум/стояние на месте)")]
    public float backwardMovementDeadzone = 0.01f;

    [Header("Streak-бонус за непрерывное удержание мяча в кадре")]
    [Tooltip("Максимальный бонус за центрирование (достигается при streak ≥ streakCap). " +
             "При streak=0 бонус = 0, растёт линейно до этого значения.")]
    public float centeringBonusMax  = 0.012f;
    [Tooltip("Сколько шагов подряд нужно удерживать мяч в центре, чтобы выйти на полный бонус. " +
             "30 при DecisionPeriod=5 ≈ 3 секунды непрерывного трекинга.")]
    public int   centeringStreakCap = 30;
    [Tooltip("Порог |horizontalAngle| (0..1), ниже которого мяч считается 'в центре' для streak. " +
             "0.25 = мяч в средней четверти кадра.")]
    public float centeringAngleThreshold = 0.25f;
    [Tooltip("Минимальная скорость вперёд (м/с) или поворот (норм.), при которой streak продолжается. " +
             "Если робот стоит — streak сбрасывается: нельзя заработать бонус, просто стоя и глядя.")]
    public float centeringMinMovement = 0.03f;

    [Header("Совмещение корпуса и камеры (основной стимул разворота корпуса)")]
    [Tooltip("Награда, когда корпус развёрнут туда же, куда смотрит камера (угол сервопривода " +
             "относительно корпуса близок к 0), И в этот момент мяч виден. Раньше агент получал " +
             "награду просто за то, что камера навела мяч в центр кадра — и находил дешёвый способ " +
             "стоять на месте, крутя только камерой. Теперь награда требует, чтобы КОРПУС сам довернулся " +
             "туда, куда смотрит камера: камера может свободно искать мяч в широком диапазоне, но пока " +
             "корпус не подстроится под её направление — награды не будет. Вес больше, чем у centeringBonus — " +
             "это главный стимул именно доворачивать корпус и потом ехать, а не просто смотреть.")]
    public float bodyCameraAlignmentBonus = 0.007f;
    [Tooltip("Допуск (градусы) между углом камеры (относительно корпуса) и 0, при котором " +
             "считаем корпус и камеру 'совмещёнными'. По ТЗ — 0..3°.")]
    public float bodyCameraAlignmentToleranceDeg = 3f;


    [Tooltip("Порог ИК клешни, выше которого считаем, что мяч действительно рядом")]
    public float grabProximityIRThreshold = 0.5f;

    [Header("Случайный спавн робота и мяча")]
    [Tooltip("Спавнить робота в случайной точке из obstacleSpawner.unusedPoints при каждом эпизоде")]
    public bool randomizeSpawnPositions = true;
    [Tooltip("Минимальное расстояние между роботом и мячом при спавне (м). " +
             "Ставь >= размера мяча + захвата, чтобы робот не заспавнился на мяче.")]
    public float minRobotBallDistance = 0.6f;
    [Tooltip("Рандомизировать поворот робота при спавне (0..360°)")]
    public bool randomizeRobotHeading = true;

    [Header("Спавн мяча — фиксированные точки")]
    [Tooltip("Массив точек-кандидатов для мяча (например, середины 4 бортиков арены). " +
             "На каждом эпизоде случайно выбирается ОДНА. Если массив пуст — берётся точка " +
             "из obstacleSpawner.unusedPoints (старое поведение). Приоритетнее чем unusedPoints.")]
    public Transform[] ballSpawnPoints;

    [Header("Рандомизация массы мяча")]
    [Tooltip("На каждом эпизоде мяч получает случайную массу (Gaussian). " +
             "Помогает обучить полиси быть устойчивой к разной инерции мяча.")]
    public bool randomizeBallMass = true;

    [Header("Рандомизация массы робота")]
    [Tooltip("На каждом эпизоде масса робота выставляется случайно (Gaussian). " +
             "Работает только если Rigidbody НЕ kinematic.")]
    public bool randomizeRobotMass = true;

    [Header("Рандомизация моторов (реалистичная модель: общий фактор + per-side джиттер)")]
    [Tooltip("Включить рандомизацию моторов")]
    public bool  randomizeMotors        = true;
    [Tooltip("ОБЩИЙ множитель обеих гусениц за эпизод. Эмулирует уровень заряда АКБ / общий износ. " +
             "Оба мотора получают ОДНО значение из этого диапазона (не независимо). " +
             "0.85..1.15 = заряд от почти-разряженного до свежего.")]
    public float motorCommonMulMin      = 0.85f;
    public float motorCommonMulMax      = 1.15f;
    [Tooltip("Малый ПЕР-СТОРОННИЙ джиттер, добавляемый к каждой гусенице отдельно. " +
             "Эмулирует производственный допуск и небольшой износ редукторов. " +
             "±0.02 = максимум 2% разницы между L и R. Реалистично.")]
    [Range(0f, 0.10f)]
    public float motorPerSideJitter     = 0.02f;
    [Tooltip("Разброс максимальной скорости робота (maxLinearCmd), м/сек. " +
             "0.6..1.0 = более-менее сильный/слабый АКБ или трение.")]
    public float robotMaxSpeedMin       = 0.6f;
    public float robotMaxSpeedMax       = 1.0f;
    [Tooltip("Разброс сглаживания разгона PWM. Больше — медленнее реакция мотора.")]
    public float motorPwmStepMin        = 10f;
    public float motorPwmStepMax        = 20f;

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


    // --- служебные ---
    private Rigidbody _rb;
    private Vector3   _startPosition;
    private Quaternion _startRotation;
    private Vector3   _ballStartPosition;

    private float _cameraServoAngle       = 0f;
    private float _prevDistanceToBall     = -1f;
    private float _prevGas                = 0f;
    private float _prevSteer              = 0f;
    private float _prevCam                = 0f;
    private float _currentCameraYaw       = 0f;  // -1..1, абсолютное состояние камеры с rate-limit
    private float _smoothedCamTarget      = 0f;  // EMA-сглаженный target от сети — глушит шум
    private float _lastObstacleHitTime    = -999f; // время последнего штрафа за столкновение (cooldown)
    private float _timeSinceLastDetection = 0f;
    private float _lastKnownBallDirection = 0f;

    // Скорость корпуса, вычисленная вручную по дельте позиции (см. комментарий к классу).
    private Vector3 _prevPosition;
    private Vector3 _lastVelocity;

    private int _episodeCount    = 0; // счётчик эпизодов агента, для периодического пересоздания препятствий

    // Burst dropout: сколько шагов подряд ещё нужно "слепить" YOLO
    private int   _dropoutStepsLeft = 0;
    // Предыдущее значение eulerAngles.y для вычисления угловой скорости
    private float _prevHeadingDeg   = 0f;

    // --- Кастомные метрики за эпизод ---
    private float _rewardDistance  = 0f;
    private float _rewardWall      = 0f;
    private float _rewardObstacle  = 0f;  // штрафы за физические столкновения с obstacle
    private float _rewardCenter    = 0f;
    private int   _centeringStreak = 0;   // шагов подряд мяч в центре + робот движется
    private float _rewardDriveRate = 0f;  // штраф за резкость gas/steer
    private float _rewardCamRate   = 0f;  // штраф за резкость camTarget (отдельно — важнее)
    private float _rewardStep      = 0f;
    private float _rewardTerminal  = 0f;
    private int   _phase2Steps     = 0;   // сколько шагов за эпизод робот провёл в Phase 2
    private int   _framesBallVisible = 0;
    private int   _framesTotal       = 0;
    private float _speedAccum        = 0f;
    private int   _dropoutBurstsCount = 0; // сколько burst dropout произошло за эпизод

    private Queue<float[]> actionBuffer = new Queue<float[]>();

    // Отдельный FIFO для задержки сенсоров (UZ, LIR, RIR, GripperIR).
    // Реальный ROS pipeline: sensor → serial → node → topic → Unity. Латентность есть.
    private Queue<float[]> sensorBuffer = new Queue<float[]>();
    private float[] _delayedSensors = new float[4] { 1f, 0f, 0f, 0f };
    private int currentActionLatency = 5;

    // ---- Helpers: абстрагируют источник видения (sim vs реальный робот) ----
    bool VisionIsVisible()
    {
        if (useRealRobot) return yolo != null && yolo.isVisible;
        return simCam != null && simCam.isVisible;
    }
    float VisionHorizontalAngle()
    {
        if (useRealRobot) return yolo != null ? yolo.horizontalAngle : 0f;
        return simCam != null ? simCam.horizontalAngle : 0f;
    }
    float VisionNormalizedDistance()
    {
        if (useRealRobot) return yolo != null ? yolo.normalizedDistance : 1f;
        return simCam != null ? simCam.normalizedDistance : 1f;
    }

    public override void Initialize()
    {
        _rb = GetComponent<Rigidbody>();
        _startPosition = transform.position;
        _startRotation = transform.rotation;
        _prevPosition = _startPosition;
        _lastVelocity = Vector3.zero;
        if (targetBall != null) _ballStartPosition = targetBall.position;

        // Проверка что ссылки идут на объекты внутри той же иерархии верхнего родителя
        // (т.е. на объекты своей арены, а не на чужие). Если ссылка share'ится между
        // инстансами префабов — Unity об этом молчит, но обучение будет считать
        // ложные success rate и общие state. Ловим на этапе Initialize.
        Transform root = transform.root;
        void CheckSameArena(Object obj, string label)
        {
            if (obj == null) return;
            Component comp = obj as Component;
            if (comp == null) return;
            if (comp.transform.root != root)
            {
                Debug.LogError($"[{name}] Поле '{label}' ссылается на объект '{comp.name}' " +
                               $"ВНЕ этой арены (его root = '{comp.transform.root.name}', мой root = '{root.name}'). " +
                               $"Это баг мульти-арены — пересобери префаб.", this);
            }
        }
        CheckSameArena(tracks,           "tracks");
        CheckSameArena(gripper,          "gripper");
        CheckSameArena(sensors,          "sensors");
        CheckSameArena(simCam,           "simCam");   // simCam — в арене; yolo (RealVision) — глобальный
        CheckSameArena(obstacleSpawner,  "obstacleSpawner");
        if (cameraServo    != null && cameraServo.root    != root) Debug.LogError($"[{name}] cameraServo вне арены",    this);
        if (targetBall     != null && targetBall.root     != root) Debug.LogError($"[{name}] targetBall вне арены",     this);
        if (ballSpawnPoints != null)
        {
            for (int i = 0; i < ballSpawnPoints.Length; i++)
            {
                var p = ballSpawnPoints[i];
                if (p != null && p.root != root)
                    Debug.LogError($"[{name}] ballSpawnPoints[{i}] ('{p.name}') вне арены", this);
            }
        }

        // Гарантируем что буферы задержки готовы до первого CollectObservations,
        // который ML-Agents может вызвать (через NotifyAgentDone/Bootstrapping)
        // ещё ДО первого OnEpisodeBegin.
        InitSensorBuffer();
        InitActionBuffer();
    }

    void InitSensorBuffer()
    {
        if (sensorBuffer == null) sensorBuffer = new Queue<float[]>();
        sensorBuffer.Clear();
        _delayedSensors = new float[] { 1f, 0f, 0f, 0f };
        for (int i = 0; i < sensorLatencySteps; i++)
            sensorBuffer.Enqueue(new float[] { 1f, 0f, 0f, 0f });
    }

    void InitActionBuffer()
    {
        if (actionBuffer == null) actionBuffer = new Queue<float[]>();
        actionBuffer.Clear();
        int lat = Mathf.Max(1, currentActionLatency > 0 ? currentActionLatency : 10);
        for (int i = 0; i < lat; i++)
            actionBuffer.Enqueue(new float[] { 0f, 0f, 0f });
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

        // --- Позиция мяча: случайная из ballSpawnPoints (например, 4 бортика арены) ---
        if (ballSpawnPoints != null && ballSpawnPoints.Length > 0)
        {
            // Собираем валидные точки (не null, не NaN-позиция)
            int validCount = 0;
            for (int i = 0; i < ballSpawnPoints.Length; i++)
                if (ballSpawnPoints[i] != null && IsValid(ballSpawnPoints[i].position))
                    validCount++;

            if (validCount > 0)
            {
                // Выбираем случайную по индексу среди валидных
                int pick = Random.Range(0, validCount);
                int idx = 0;
                for (int i = 0; i < ballSpawnPoints.Length; i++)
                {
                    if (ballSpawnPoints[i] == null || !IsValid(ballSpawnPoints[i].position)) continue;
                    if (idx == pick) { ballPos = ballSpawnPoints[i].position; break; }
                    idx++;
                }
            }
            else
            {
                Debug.LogWarning($"[{name}] ballSpawnPoints не содержит валидных точек, использую _ballStartPosition.");
            }

            // Если мяч оказался слишком близко к роботу (робот заспавнился рядом
            // с выбранным бортиком), отодвигаем робота в другую свободную точку.
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

        if (tracks != null) tracks.Move(0f, 0f);

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

        if (randomizeMotors && tracks != null)
        {
            // Реалистичная модель мотора: общий фактор (АКБ) + малый per-side джиттер (допуск).
            // Оба мотора питаются от одного источника — их скорости коррелированы.
            // Раньше независимая рандомизация давала асимметрию до ±30%, из-за чего робот
            // при спавне сразу тянуло сильно вбок — источник дрыганья на прямой езде.
            float commonMul   = Random.Range(motorCommonMulMin, motorCommonMulMax);
            float leftJitter  = Random.Range(-motorPerSideJitter, motorPerSideJitter);
            float rightJitter = Random.Range(-motorPerSideJitter, motorPerSideJitter);
            tracks.leftSpeedMul  = commonMul * (1f + leftJitter);
            tracks.rightSpeedMul = commonMul * (1f + rightJitter);

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
        _prevGas                = 0f;
        _prevSteer              = 0f;
        _prevCam                = 0f;
        _currentCameraYaw       = 0f;
        _smoothedCamTarget      = 0f;
        _lastObstacleHitTime    = -999f;
        _timeSinceLastDetection = 0f;
        _lastKnownBallDirection = 0f;

        // Сброс вручную-считаемой скорости — иначе первый шаг нового эпизода
        // засчитает "прыжок" из старой позиции конца прошлого эпизода в стартовую как движение.
        _prevPosition = transform.position;
        _lastVelocity = Vector3.zero;

        // Сброс кастомных счётчиков
        _rewardDistance  = 0f;
        _rewardWall      = 0f;
        _rewardObstacle  = 0f;
        _rewardCenter    = 0f;
        _centeringStreak = 0;
        _rewardDriveRate = 0f;
        _rewardCamRate   = 0f;
        _rewardStep      = 0f;
        _rewardTerminal  = 0f;
        _phase2Steps     = 0;
        _framesBallVisible = 0;
        _framesTotal       = 0;
        _speedAccum        = 0f;

        // Burst dropout сбрасываем на начало эпизода
        _dropoutStepsLeft  = 0;
        _prevHeadingDeg    = transform.eulerAngles.y;

        // Симуляция латентности реального ROS2 pipeline
        currentActionLatency = (int)UnityEngine.Random.Range(8.14f, 12.14f);
        InitActionBuffer();

        // Sensor buffer — задержка датчиков (независимо от action latency)
        InitSensorBuffer();

        // Читаем yaml environment_parameters — обновляет ball_mass/scale, шумы, latency.
        // Вызывается ПОСЛЕ базовой рандомизации, чтобы yaml мог её переопределить.
        ReadYamlEnvParams();
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
        // === СЕНСОРЫ (наблюдения 1-4) с шумом + задержкой ===
        // Шум добавляется к УЗ. ИК — бинарные, шум применён как случайные инверсии.
        // Задержка — FIFO из sensorBuffer (симулирует ROS/serial latency).
        float rawUS   = sensors != null ? sensors.ultrasonicNormalized : 1f;
        float noisyUS = Mathf.Clamp01(rawUS + Gaussian(ultrasonicNoise));
        float rawLIR  = sensors != null ? sensors.leftIR   : 0;
        float rawRIR  = sensors != null ? sensors.rightIR  : 0;
        float rawGIR  = sensors != null ? sensors.gripperIR : 0;

        // Пропускаем через sensor buffer (если задержка > 0)
        if (sensorLatencySteps > 0)
        {
            if (sensorBuffer == null) InitSensorBuffer();
            sensorBuffer.Enqueue(new float[] { noisyUS, rawLIR, rawRIR, rawGIR });
            _delayedSensors = sensorBuffer.Dequeue();
            sensor.AddObservation(_delayedSensors[0]);
            sensor.AddObservation(_delayedSensors[1]);
            sensor.AddObservation(_delayedSensors[2]);
            sensor.AddObservation(_delayedSensors[3]);
        }
        else
        {
            sensor.AddObservation(noisyUS);
            sensor.AddObservation(rawLIR);
            sensor.AddObservation(rawRIR);
            sensor.AddObservation(rawGIR);
        }

        // === VISION (наблюдения 5-8) с шумом ===
        bool visible = SeesBallEffective();
        float angleObs = visible
            ? Mathf.Clamp(VisionHorizontalAngle() + Random.Range(-visionAngleNoise, visionAngleNoise), -1f, 1f)
            : 0f;
        float distObs = visible
            ? Mathf.Clamp01(VisionNormalizedDistance() + Random.Range(-visionDistanceNoise, visionDistanceNoise))
            : 1f;
        sensor.AddObservation(angleObs);
        sensor.AddObservation(distObs);
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

        // 14. Скорость робота (м/с) — вручную посчитанная по дельте позиции,
        // а не rb.linearVelocity (для кинематического Rigidbody она всегда 0).
        sensor.AddObservation(_lastVelocity.magnitude);

        // 15. Время с последней детекции мяча (сек)
        sensor.AddObservation(_timeSinceLastDetection);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
            UpdateBurstDropout();

        // Таймаут — используем встроенный StepCount/MaxStep агента (задаётся в Inspector).
        // Применяем штраф за шаг до того, как ML-Agents сам вызовет EndEpisode().
        if (MaxStep > 0 && StepCount >= MaxStep - 1)
        {
            AddReward(-timeoutPenalty);
            _rewardTerminal -= timeoutPenalty;
            LogEpisodeStats(success: false);
            return;
        }

        // Считываем свежие сигналы от сети.
        //   ContinuousActions[0] = gas             ∈ [-1..1]  (вперёд / назад)
        //   ContinuousActions[1] = steering        ∈ [-1..1]  (влево / вправо)
        //   ContinuousActions[2] = camera yaw TARGET ∈ [-1..1] (нормализованный угол камеры)
        float freshGas   = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float freshSteer = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float freshCam   = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        // Discrete action: если useAutomaticGripper — сеть НЕ управляет клешнёй,
        // читаем только для обратной совместимости (когда в Behavior Parameters
        // ещё Discrete Branches = 1). При useAutomaticGripper клешня хватает
        // автоматически по IR (задаётся через GripperController.autoGrabOnSensor).
        int gripAct = 0;
        if (!useAutomaticGripper && actions.DiscreteActions.Length > 0)
        {
            gripAct = actions.DiscreteActions[0]; // 0 = idle, 1 = grab, 2 = release
        }

        // Пропускаем непрерывные сигналы через FIFO-буфер задержки — эмулируем
        // латентность ROS2/сети/моторов. Дискретный gripAct не задерживаем.
        actionBuffer.Enqueue(new float[] { freshGas, freshSteer, freshCam });
        float[] delayed = actionBuffer.Count > 0
            ? actionBuffer.Dequeue()
            : new float[] { 0f, 0f, 0f };

        float gas     = delayed[0];
        float steer   = delayed[1];
        float camTarget = delayed[2];

        // 1. Движение — Gas + Steering, TrackController раскладывает в L/R по turnK.
        //    Совместимо с ROS /cmd_vel (linear.x = gas, angular.z = steer).
        //    HARD-STOP после захвата: пока держим мяч, силой обнуляем моторы, чтобы
        //    не крутиться и не уронить (как в референсе).
        if (tracks != null)
        {
            bool holdingBall = hardStopOnHold && gripper != null && gripper.isHolding;
            if (holdingBall) tracks.Move(0f, 0f);
            else             tracks.Move(gas, steer);
        }

        // 2. Сервопривод камеры — абсолютный target с ТРЕМЯ уровнями сглаживания:
        //    (a) EMA-фильтр самого target'а — глушит высокочастотный шум сети;
        //    (b) deadband на дельту — не двигаемся, если target "почти на месте";
        //    (c) rate-limit — физический предел скорости сервомотора.
        //    Все три параметра НАСТРАИВАЮТСЯ в инспекторе:
        //    cameraTargetEmaAlpha, cameraTargetDeadband, cameraMaxStepDeg.

        // Страховка от NaN
        if (float.IsNaN(camTarget) || float.IsInfinity(camTarget)) camTarget = _currentCameraYaw;

        // Переводим max-step из градусов в нормализованные [-1..1] единицы
        float maxStepNormalized = cameraMaxStepDeg / Mathf.Max(1f, cameraServoMaxAngle);

        // (a) EMA
        _smoothedCamTarget = (1f - cameraTargetEmaAlpha) * _smoothedCamTarget
                             + cameraTargetEmaAlpha * camTarget;

        // (b) Deadband
        float camDelta = _smoothedCamTarget - _currentCameraYaw;
        if (Mathf.Abs(camDelta) < cameraTargetDeadband) camDelta = 0f;

        // (c) Rate-limit
        camDelta = Mathf.Clamp(camDelta, -maxStepNormalized, maxStepNormalized);

        _currentCameraYaw = Mathf.Clamp(_currentCameraYaw + camDelta, -1f, 1f);
        _cameraServoAngle = _currentCameraYaw * cameraServoMaxAngle;
        if (cameraServo != null)
            cameraServo.localRotation = Quaternion.Euler(0f, _cameraServoAngle, 0f);

        // 3. Клешня
        if (gripper != null)
        {
            if (useAutomaticGripper)
            {
                // Автоматический режим (как в референсе): grabCommand всегда true.
                // GripperController.CanGrab() сам решит хватать по IR-датчику клешни.
                gripper.grabCommand = true;
                if (useRealRobot && rosBridge != null && gripper.isHolding)
                    rosBridge.PublishGripperCmd(2); // закрыть клешню на реальном роботе
            }
            else
            {
                // Legacy режим: сеть управляет через дискретный action.
                if      (gripAct == 1) gripper.grabCommand = true;
                else if (gripAct == 2) gripper.grabCommand = false;
            }
        }

        // 4. Обновление служебных переменных детекции — используем эффективную видимость
        if (SeesBallEffective())
        {
            _lastKnownBallDirection = VisionHorizontalAngle();
            _timeSinceLastDetection = 0f;
        }
        else
        {
            _timeSinceLastDetection += Time.deltaTime;
        }

        // 4b. Пересчёт скорости корпуса по дельте позиции — ДО ComputeRewards.
        Vector3 currentPosition = transform.position;
        float dt = Mathf.Max(Time.deltaTime, 0.0001f);
        _lastVelocity = (currentPosition - _prevPosition) / dt;
        _prevPosition = currentPosition;

        // 4c. ROS: отправляем команды на реальный робот (только в режиме useRealRobot)
        if (useRealRobot && rosBridge != null)
        {
            rosBridge.PublishCommand(gas, steer);
            rosBridge.PublishCameraCmd(_cameraServoAngle);
        }

        // 5. Награды
        ComputeRewards(gas, steer, camTarget, gripAct);

        _prevGas   = gas;
        _prevSteer = steer;
        _prevCam   = camTarget;
    }

    void ComputeRewards(float gas, float steer, float camTarget, int gripAct)
    {
        // Кадровые счётчики для метрик — считаем эффективную видимость,
        // чтобы Custom/BallVisibleFraction отражал реальный сигнал, доходящий до модели.
        _framesTotal++;
        bool ballVisible = SeesBallEffective();
        if (ballVisible) _framesBallVisible++;
        _speedAccum += _lastVelocity.magnitude;

        // а) Двухфазная логика сближения с мячом.
        //    Phase 1 (dist ≥ closeRadius): delta × scale × exp(α×(1-dist)).
        //      exp-множитель без сингулярности: α=2 → ×7.4 у границы closeRadius, ×1.0 на 1м.
        //    Phase 2 (dist < closeRadius): delta-reward отключён — работает slow-placement (ниже).
        float curDist = DistanceToBall();
        float forwardSpeed = Vector3.Dot(_lastVelocity, transform.forward);
        if (_prevDistanceToBall > 0f && curDist > 0f && curDist >= closeRadius)
        {
            float delta = Mathf.Clamp(_prevDistanceToBall - curDist, -0.5f, 0.5f);
            float expMul = Mathf.Exp(distanceRewardAlpha * (1f - Mathf.Clamp01(curDist)));
            float rDist = delta * distanceRewardScale * expMul;
            AddReward(rDist); _rewardDistance += rDist;
        }
        _prevDistanceToBall = curDist;

        // б) Раздельные штрафы за резкость управления.
        //    Камера штрафуется сильнее: EMA сглаживает физическое движение, но не учит сеть
        //    выдавать плавные команды — без отдельного penalty сеть осциллирует, создавая артефакт.
        float dGas   = gas       - _prevGas;
        float dSteer = steer     - _prevSteer;
        float dCam   = camTarget - _prevCam;
        float rDrive = -driveRatePenalty  * (dGas * dGas + dSteer * dSteer);
        float rCam   = -cameraRatePenalty * (dCam * dCam);
        AddReward(rDrive + rCam);
        _rewardDriveRate += rDrive;
        _rewardCamRate   += rCam;

        // в) Streak-бонус за непрерывное удержание мяча в кадре + bodyCameraAlignment.
        //
        //    STREAK-МЕХАНИКА (в.1):
        //    Проблема плоского centeringBonus = const: агент выучивает «surveillance mode» —
        //    подъезжает к мячу, останавливается, вращает серво так чтобы мяч оставался в центре,
        //    и собирает бонус каждый шаг не двигаясь к цели. Простое уменьшение бонуса лишь
        //    ослабляет сигнал, не убирая эксплойт.
        //
        //    Решение — streak + условие движения:
        //      _centeringStreak++ если мяч виден, |angle| < threshold, И робот движется.
        //      _centeringStreak = 0 если любое условие нарушено.
        //    Награда = (streak / streakCap) × centeringBonusMax × (1 - |angle|).
        //      streak=0  → бонус 0   (стоять и смотреть = ничего не зарабатывает)
        //      streak=15 → бонус ×0.5 (полминуты непрерывного трекинга при движении)
        //      streak≥30 → бонус max  (плато — экспоненты нет, чтобы не взорвать градиент)
        //    Рост линейный до cap — не экспоненциальный: истинная exp(k×streak) взрывается
        //    на длинных streak'ах и вытесняет все остальные награды из градиента PPO.
        //
        //    bodyCameraAlignmentBonus (в.2) — отдельная награда: корпус развёрнут туда, куда
        //    смотрит камера (угол серво ≤ toleranceDeg). Основной стимул разворачивать КОРПУС,
        //    а не только наводить серво. Streak и alignment независимы: можно иметь alignment
        //    без streak (тело повёрнуто, но мяч только что появился в кадре) и наоборот.
        bool ballCentered = VisionIsVisible() && Mathf.Abs(VisionHorizontalAngle()) < centeringAngleThreshold;
        bool robotMoving  = Mathf.Abs(forwardSpeed) > centeringMinMovement
                            || Mathf.Abs(steer)      > centeringMinMovement;

        if (ballCentered && robotMoving)
            _centeringStreak = Mathf.Min(_centeringStreak + 1, centeringStreakCap);
        else
            _centeringStreak = 0;

        if (_centeringStreak > 0)
        {
            float streakFactor = (float)_centeringStreak / centeringStreakCap;           // 0..1
            float centered     = 1f - Mathf.Abs(VisionHorizontalAngle());                // точность
            float rCenter      = streakFactor * centeringBonusMax * centered;
            AddReward(rCenter); _rewardCenter += rCenter;
        }

        if (VisionIsVisible())
        {
            float absServoAngle = Mathf.Abs(_cameraServoAngle);
            if (absServoAngle <= bodyCameraAlignmentToleranceDeg)
            {
                float alignment = 1f - (absServoAngle / Mathf.Max(0.0001f, bodyCameraAlignmentToleranceDeg));
                AddReward(alignment * bodyCameraAlignmentBonus);
            }
        }

        // г) Штраф за критически близкие стены
        if (sensors != null)
        {
            float wallPen = 0f;
            if (sensors.ultrasonicNormalized < 0.05f) wallPen += wallProximityPenalty;
            if (sensors.leftIR  == 1)                 wallPen += wallProximityPenalty;
            if (sensors.rightIR == 1)                 wallPen += wallProximityPenalty;
            if (wallPen > 0f) { AddReward(-wallPen); _rewardWall -= wallPen; }
        }

        // д) Мелкий постоянный штраф — не стоять
        AddReward(-perStepPenalty);
        _rewardStep -= perStepPenalty;

        // е) Штраф за движение назад — по проекции РЕАЛЬНОЙ (вручную посчитанной по дельте
        // позиции) скорости корпуса на forward. Так штраф не срабатывает при развороте на месте
        // (там продольная составляющая ≈ 0) и корректно учитывает фактическое перемещение,
        // а не сигналы гусениц напрямую.
        if (forwardSpeed < -backwardMovementDeadzone)
        {
            AddReward(-Mathf.Abs(forwardSpeed) * backwardMovementPenalty);
        }

        // е.1) BLIND APPROACH — бонус за движение вперёд когда мяч НЕ виден.
        //      Без этого робот на старте (пока не научился крутиться и искать) просто стоит
        //      и копит per-step penalty. Тут стимулируем ехать вперёд в поиске.
        if (!ballVisible && forwardSpeed > blindApproachMinForwardSpeed)
        {
            AddReward(blindApproachBonus);
            _rewardCenter += blindApproachBonus;  // логируем как part of "center/search" cluster
        }

        // е.2) PHASE 2 — медленный точный подъезд (dist < closeRadius).
        //      Delta-reward здесь отключён (см. блок «а»). Вместо него:
        //      бонус если linear И angular команда ниже порогов (робот вкатывается тихо),
        //      штраф если едет с газом — пролетит мимо, не поместит мяч в клешню.
        if (curDist > 0f && curDist < closeRadius)
        {
            _phase2Steps++;
            float linSpeed = Mathf.Abs(forwardSpeed);
            float angCmd   = Mathf.Abs(steer);
            if (linSpeed < gentleLinearSpeedThresh && angCmd < gentleAngularSpeedThresh)
            {
                AddReward(gentlePlacementBonus);
                _rewardDistance += gentlePlacementBonus;
            }
            else
            {
                AddReward(-gentleOverspeedPenalty);
                _rewardDistance -= gentleOverspeedPenalty;
            }
        }

        // к) Терминал при захвате своего мяча.
        //    Верифицируем что схвачен ИМЕННО свой мяч (защита от мульти-арены).
        if (gripper != null && gripper.isHolding)
        {
            bool ownBall = true;
            if (targetBall != null && gripper.HeldRigidbody != null)
                ownBall = (gripper.HeldRigidbody.transform == targetBall);

            if (ownBall)
            {
                AddReward(grabSuccessReward);
                _rewardTerminal += grabSuccessReward;
                LogEpisodeStats(success: true);
                EndEpisode();
                return;
            }
            else
            {
                // Захвачен ЧУЖОЙ мяч (мульти-арена баг) — отпускаем, штрафуем, не терминал.
                Debug.LogWarning($"[{name}] Захвачен ЧУЖОЙ мяч '{gripper.HeldRigidbody.name}' " +
                                 $"(мой targetBall = '{targetBall.name}'). Отпускаю.");
                gripper.Release();
                gripper.grabCommand = false;
            }
        }

        // л) Терминал: вылет за арену.
        // Границы отсчитываются от стартовой позиции + локальный offset арены,
        // а не от глобального (0,0,0) — иначе робот, спавнящийся не в нуле,
        // сразу считается вылетевшим.
        Vector3 arenaCenter = _startPosition + arenaCenterOffset;
        Vector3 p = transform.position - arenaCenter;
        if (Mathf.Abs(p.x) > arenaHalfSize.x ||
            Mathf.Abs(p.z) > arenaHalfSize.z ||
            p.y < -arenaHalfSize.y || p.y > arenaHalfSize.y)
        {
            AddReward(-outOfArenaPenalty);
            _rewardTerminal -= outOfArenaPenalty;
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

        // Диагностика: полезно видеть какой конкретно робот успел / провалил
        if (success)
            Debug.Log($"[{name}] SUCCESS в эпизоде #{_episodeCount}, шаг {StepCount}. " +
                      $"Схваченный мяч: {(gripper != null && gripper.HeldRigidbody != null ? gripper.HeldRigidbody.name : "null")}");

        // Успех эпизода — усредняется во время summary, даёт success rate ∈ [0..1]
        s.Add("Custom/SuccessRate", success ? 1f : 0f);

        // Разбивка накопленной награды за эпизод
        s.Add("Custom/Reward/Distance",  _rewardDistance);
        s.Add("Custom/Reward/Wall",      _rewardWall);
        s.Add("Custom/Reward/Obstacle",  _rewardObstacle);
        s.Add("Custom/Reward/Center",    _rewardCenter);
        s.Add("Custom/Reward/DriveRate", _rewardDriveRate);
        s.Add("Custom/Reward/CamRate",   _rewardCamRate);
        s.Add("Custom/Reward/Step",      _rewardStep);
        s.Add("Custom/Reward/Terminal",  _rewardTerminal);

        // Сколько шагов за эпизод робот провёл в Phase 2 (близко к мячу)
        s.Add("Custom/Phase2Steps", _phase2Steps);

        // Максимальный streak центрирования за эпизод — показывает качество трекинга
        s.Add("Custom/CenteringStreakMax", _centeringStreak);

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

        float gas = 0f, steer = 0f, camTarget = 0f;
        int   g = 0;

        var kb = Keyboard.current;
        if (kb != null)
        {
            // WASD — драйверские команды (как в референсе):
            //   W/S — газ (вперёд/назад)
            //   A/D — руль (влево/вправо)
            //   I/K — камера yaw target: вверх/вниз в [-1..1]
            //   Space — grab, X — release
            gas   = kb.wKey.ReadValue() - kb.sKey.ReadValue();
            steer = kb.dKey.ReadValue() - kb.aKey.ReadValue();
            camTarget = kb.iKey.ReadValue() - kb.kKey.ReadValue();
            if      (kb.spaceKey.isPressed) g = 1;
            else if (kb.xKey.isPressed)     g = 2;
        }

        cont[0] = gas;
        cont[1] = steer;
        cont[2] = camTarget;
        // disc[0] пишем только если Discrete Branches в Behavior Parameters ещё стоит.
        // При useAutomaticGripper Discrete Branches = 0, disc.Length = 0.
        if (disc.Length > 0) disc[0] = g;
    }

    float DistanceToBall()
    {
        if (targetBall == null) return -1f;
        return Vector3.Distance(transform.position, targetBall.position);
    }

    /// <summary>
    /// Читает environment_parameters из config.yaml (Academy) и обновляет соответствующие поля.
    /// Вызывается в OnEpisodeBegin — параметры могут меняться между эпизодами (curriculum).
    /// Если yaml не задал параметр, оставляем текущее значение из инспектора.
    /// </summary>
    void ReadYamlEnvParams()
    {
        if (!useYamlEnvParams) return;
        var env = Academy.Instance.EnvironmentParameters;

        // Ball: mass, scale
        if (targetBall != null)
        {
            var brb = targetBall.GetComponent<Rigidbody>();
            if (brb != null && !brb.isKinematic)
            {
                float yamlBallMass = env.GetWithDefault("ball_mass", -1f);
                if (yamlBallMass > 0f) brb.mass = yamlBallMass;
            }
            float yamlBallScale = env.GetWithDefault("ball_scale", -1f);
            if (yamlBallScale > 0.001f) targetBall.localScale = Vector3.one * yamlBallScale;
        }

        // Noise
        float yamlVisionNoise = env.GetWithDefault("vision_noise", -1f);
        if (yamlVisionNoise >= 0f)
        {
            visionAngleNoise    = yamlVisionNoise;
            visionDistanceNoise = yamlVisionNoise * 3f;
        }
        float yamlUsNoise = env.GetWithDefault("ultrasonic_noise", -1f);
        if (yamlUsNoise >= 0f) ultrasonicNoise = yamlUsNoise;

        // Latency
        float yamlSensorLat = env.GetWithDefault("sensor_latency", -1f);
        if (yamlSensorLat >= 0f) sensorLatencySteps = Mathf.Max(0, (int)yamlSensorLat);
        float yamlActionLat = env.GetWithDefault("action_latency", -1f);
        if (yamlActionLat >= 0f) currentActionLatency = Mathf.Max(0, (int)yamlActionLat);

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
        if (!VisionIsVisible()) return false;
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

    // Штраф за физическое столкновение с препятствием.
    // Cooldown 0.5с — чтобы один удар давал ровно один штраф, а не спам каждый FixedUpdate.
    void OnCollisionEnter(Collision col)
    {
        if (!col.gameObject.CompareTag(obstacleTag)) return;
        if (Time.time - _lastObstacleHitTime < 0.5f) return;

        _lastObstacleHitTime  = Time.time;
        AddReward(-obstacleCollisionPenalty);
        _rewardObstacle -= obstacleCollisionPenalty;

        if (endEpisodeOnObstacleHit)
        {
            LogEpisodeStats(success: false);
            EndEpisode();
        }
    }
}