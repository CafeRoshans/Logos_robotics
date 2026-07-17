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
    public SimulatedYoloCamera yolo;
    [Tooltip("Transform, вокруг Y которого крутится камера (сервопривод). Может быть родителем самой камеры.")]
    public Transform cameraServo;
    [Tooltip("Мяч, за которым робот охотится")]
    public Transform targetBall;
    [Tooltip("Спавнер препятствий на этой арене. Если задан — на каждый OnEpisodeBegin " +
             "будет вызван Respawn() и препятствия перераскладываются случайно.")]
    public ObstacleSpawner obstacleSpawner;

    [Header("Сервопривод камеры")]
    [Tooltip("Максимальный угол отклонения камеры ± (градусы). Специально НЕ ограничиваем узко — " +
             "камера должна свободно 'осматриваться' и искать мяч в широком диапазоне. Награда за " +
             "центрирование мяча в кадре есть (centeringBonus), но основной и более весомый стимул — " +
             "довернуть КОРПУС туда же, куда смотрит камера (bodyCameraAlignmentBonus), так что широкий " +
             "диапазон камеры не создаёт лазейки 'стою и просто верчу камерой'.")]
    public float cameraServoMaxAngle = 90f;
    [Tooltip("Скорость поворота сервопривода (град/сек) при полном сигнале")]
    [System.Obsolete("Не используется с action space Gas/Steering. Скорость камеры теперь задаётся через MAX_CAMERA_STEP_NORMALIZED в OnActionReceived.")]
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

    // wobbleAsymmetryPenalty и steerFlipPenalty удалены — они были нужны, чтобы модель,
    // управляющая напрямую left/right, не ездила зигзагом. С Gas/Steering раскладкой в
    // Controller.Move() симметрия обеспечивается автоматически: gas=1, steer=0 → left=right,
    // а зигзаг стал невозможен на уровне action space.
    [Tooltip("Терминальный бонус за успешный захват мяча — начисляется когда мяч удержан holdStepsRequired шагов подряд")]
    public float grabSuccessReward      = 5.0f;

    [Header("Hold-to-succeed — надо ПРОДЕРЖАТЬ мяч N шагов")]
    [Tooltip("Сколько decisions подряд мяч должен быть в клешне до срабатывания терминала +grabSuccessReward. " +
             "50 при DecisionPeriod=5 ≈ 1 секунда симуляции. Учит крепкому захвату, не 'брифовому касанию'.")]
    public int holdStepsRequired = 50;
    [Tooltip("Непрерывная награда за каждый шаг удержания мяча (до срабатывания терминала). " +
             "0.02 × 50 = +1.0 суммарно к моменту победы, плюс +5 терминал.")]
    public float holdStepReward = 0.02f;

    [Header("Blind approach — движение вперёд когда мяч НЕ виден")]
    [Tooltip("Бонус за каждый шаг, когда робот едет вперёд, но мяч ещё не виден. " +
             "Стимулирует активный поиск, а не стояние на месте при потере мяча.")]
    public float blindApproachBonus = 0.003f;
    [Tooltip("Минимальная реальная скорость вперёд (м/с) для срабатывания blindApproachBonus")]
    public float blindApproachMinForwardSpeed = 0.05f;

    [Header("Точность подъезда к мячу (slow-down + speed penalty near ball)")]
    [Tooltip("Дистанция до мяча (м), ниже которой начинают действовать награды/штрафы за скорость")]
    public float precisionApproachDistance = 0.5f;
    [Tooltip("Порог 'медленно' (м/с). Если скорость ниже — начисляется slowdownBonus.")]
    public float precisionSlowdownSpeed = 0.15f;
    [Tooltip("Бонус за замедление возле мяча (плавный точный подъезд)")]
    public float precisionSlowdownBonus = 0.005f;
    [Tooltip("Штраф за быструю езду возле мяча (пролетает мимо, не может ухватить)")]
    public float precisionOverspeedPenalty = 0.01f;
    [Tooltip("Терминальный штраф за вылет за пределы арены")]
    // [Tooltip("Модуль штрафа за вылет за арену. Минус в коде: AddReward(-outOfArenaPenalty).")]
    public float outOfArenaPenalty      = 2.0f;
    [Tooltip("Небольшой штраф за каждый шаг — стимулирует скорость решения")]
    // [Tooltip("Модуль per-step штрафа. Минус в коде: AddReward(-perStepPenalty).")]
    public float perStepPenalty         = 0.0005f;

    [Header("Штраф за движение назад")]
    [Tooltip("Штраф за движение назад (когда корпус реально смещается против transform.forward)")]
    public float backwardMovementPenalty = 0.01f;
    [Tooltip("Мёртвая зона по скорости (м/с), ниже которой направление не штрафуем (шум/стояние на месте)")]
    public float backwardMovementDeadzone = 0.01f;

    [Header("Центрирование мяча в кадре (для точного прицеливания клешнёй)")]
    [Tooltip("Награда за то, что мяч близко к центру кадра камеры (по yolo.horizontalAngle), " +
             "независимо от угла камеры относительно корпуса. Нужна отдельно от bodyCameraAlignmentBonus: " +
             "1) агент должен понимать, когда камера/мяч выставлены настолько точно, что можно хватать " +
             "клешнёй; 2) чем точнее камера центрирует мяч, тем точнее потом можно довернуть корпус " +
             "на меньший угол, а не грубо 'в сторону мяча'. Вес меньше, чем у bodyCameraAlignmentBonus — " +
             "это вспомогательный сигнал, а не основной драйвер поведения.")]
    public float centeringBonus = 0.005f;

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


    [Header("Штраф за ложный захват")]
    [Tooltip("Штраф за команду grab, когда мяч не рядом с клешнёй")]
    public float falseGrabPenalty = 0.01f;
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
    // [Tooltip("Модуль штрафа за окончание эпизода по таймауту. Минус в коде: AddReward(-timeoutPenalty).")]
    public float timeoutPenalty = 0.5f;

    // --- служебные ---
    private Rigidbody _rb;
    // _rb.mass = 5.0f; // default mass for the robot, can be randomized if needed
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
    private int   _gripHoldSteps          = 0;   // сколько decisions подряд робот держит свой мяч
    private float _timeSinceLastDetection = 0f;
    private float _lastKnownBallDirection = 0f;
    private int   _episodeStepCount       = 0;

    // Скорость корпуса, вычисленная вручную по дельте позиции (см. комментарий к классу).
    private Vector3 _prevPosition;
    private Vector3 _lastVelocity;

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
        CheckSameArena(yolo,             "yolo");
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

        // // 5.5. Рандомизация массы робота и параметров моторов
        // //      (доменная рандомизация — те же реальные роботы отличаются АКБ, редукторами)
        // if (randomizeRobotMass && _rb != null && !_rb.isKinematic)
        // {
        //     _rb.mass = Mathf.Max(0.01f, Random.Range(robotMassMin, robotMassMax));
        // }

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
        _gripHoldSteps          = 0;
        _timeSinceLastDetection = 0f;
        _lastKnownBallDirection = 0f;
        _episodeStepCount       = 0;

        // Сброс вручную-считаемой скорости — иначе первый шаг нового эпизода
        // засчитает "прыжок" из старой позиции конца прошлого эпизода в стартовую как движение.
        _prevPosition = transform.position;
        _lastVelocity = Vector3.zero;

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

        // 14. Скорость робота (м/с) — вручную посчитанная по дельте позиции,
        // а не rb.linearVelocity (для кинематического Rigidbody она всегда 0).
        sensor.AddObservation(_lastVelocity.magnitude);

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
            AddReward(-timeoutPenalty);
            _rewardTerminal -= timeoutPenalty;
            LogEpisodeStats(success: false);
            EndEpisode();
            return;
        }

        // Считываем свежие сигналы от сети.
        //   ContinuousActions[0] = gas             ∈ [-1..1]  (вперёд / назад)
        //   ContinuousActions[1] = steering        ∈ [-1..1]  (влево / вправо)
        //   ContinuousActions[2] = camera yaw TARGET ∈ [-1..1] (нормализованный угол камеры)
        float freshGas   = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float freshSteer = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float freshCam   = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        int   gripAct    = actions.DiscreteActions[0]; // 0 = idle, 1 = grab, 2 = release

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
        if (tracks != null)
        {
            tracks.Move(gas, steer);
        }

        // 2. Сервопривод камеры — абсолютный target с ТРЕМЯ уровнями сглаживания:
        //    (a) EMA-фильтр самого target'а — глушит высокочастотный шум сети;
        //    (b) deadband на дельту — не двигаемся к target'у если он "почти на месте";
        //    (c) rate-limit MAX_CAMERA_STEP_NORMALIZED — физический предел сервомотора.
        //    Без (a) и (b) камера дрожит: любой шумный target=±0.05 → камера двигается
        //    на 15° туда-сюда каждое решение.
        const float MAX_CAMERA_STEP_NORMALIZED = 15f / 90f;   // ≈ 0.167
        const float CAM_TARGET_EMA_ALPHA       = 0.3f;         // 0.3 = сильное сглаживание
        const float CAM_TARGET_DEADBAND        = 0.03f;        // ~2.7°: игнорируем малые изменения

        // Страховка от NaN — если что-то дало Infinity, не портим состояние
        if (float.IsNaN(camTarget) || float.IsInfinity(camTarget)) camTarget = _currentCameraYaw;

        // (a) EMA сглаживание — свежий target смешивается со старым
        _smoothedCamTarget = (1f - CAM_TARGET_EMA_ALPHA) * _smoothedCamTarget
                             + CAM_TARGET_EMA_ALPHA * camTarget;

        // (b) Deadband: если сглаженный target почти совпадает с текущим положением — стоим
        float camDelta = _smoothedCamTarget - _currentCameraYaw;
        if (Mathf.Abs(camDelta) < CAM_TARGET_DEADBAND) camDelta = 0f;

        // (c) Rate-limit: физический предел скорости сервомотора
        camDelta = Mathf.Clamp(camDelta, -MAX_CAMERA_STEP_NORMALIZED, MAX_CAMERA_STEP_NORMALIZED);

        _currentCameraYaw = Mathf.Clamp(_currentCameraYaw + camDelta, -1f, 1f);
        _cameraServoAngle = _currentCameraYaw * cameraServoMaxAngle;
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

        // 4b. Пересчёт скорости корпуса по дельте позиции — ДО ComputeRewards,
        // чтобы штраф за движение назад мог её использовать. Rigidbody здесь кинематический
        // (Controller двигает через MovePosition/MoveRotation), поэтому rb.linearVelocity
        // не отражает реальное перемещение и использовать её нельзя.
        Vector3 currentPosition = transform.position;
        float dt = Mathf.Max(Time.deltaTime, 0.0001f); // защита от деления на 0
        _lastVelocity = (currentPosition - _prevPosition) / dt;
        _prevPosition = currentPosition;

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

        // б) Единый квадратичный штраф за резкость управления (как в референсе).
        //    Формула: -actionRatePenalty × (Δgas² + Δsteer² + Δcam²).
        //    Заменяет прежние три штрафа (rate + wobble + flip) — с action space
        //    (gas, steering) зигзаг физически невозможен, отдельные штрафы избыточны.
        float dGas   = gas       - _prevGas;
        float dSteer = steer     - _prevSteer;
        float dCam   = camTarget - _prevCam;
        float actionRateSq = dGas * dGas + dSteer * dSteer + dCam * dCam;
        float rAct = -actionRatePenalty * actionRateSq;
        AddReward(rAct); _rewardAction += rAct;

        // в) Две отдельные, но связанные награды за работу с камерой/корпусом:
        //
        //   в.1) centeringBonus — мяч близко к центру кадра камеры (по yolo.horizontalAngle).
        //        Не зависит от угла камеры относительно корпуса. Даёт агенту точный сигнал,
        //        когда камера действительно "прицелена" на мяч (полезно для решения о захвате
        //        клешнёй и для точного финального доворота корпуса на малый угол).
        //
        //   в.2) bodyCameraAlignmentBonus — корпус развёрнут туда же, куда смотрит камера
        //        (угол сервопривода относительно корпуса close to 0), И мяч при этом виден.
        //        Вес выше, чем у centeringBonus — это основной стимул именно крутить гусеницы
        //        и доворачивать корпус, а не просто наводить камеру и стоять на месте.
        //        _cameraServoAngle — угол ОТНОСИТЕЛЬНО корпуса, камера не ограничена в диапазоне
        //        (cameraServoMaxAngle = 90°), пусть свободно "осматривается" в поиске мяча.
        if (yolo != null && yolo.isVisible)
        {
            float centered = 1f - Mathf.Abs(yolo.horizontalAngle); // 1 в центре кадра, 0 на краю
            AddReward(centered * centeringBonus);

            float absServoAngle = Mathf.Abs(_cameraServoAngle);
            if (absServoAngle <= bodyCameraAlignmentToleranceDeg)
            {
                // Плавный градиент внутри допуска: чем ближе камера к 0° относительно корпуса — тем больше награда.
                float alignment = 1f - (absServoAngle / Mathf.Max(0.0001f, bodyCameraAlignmentToleranceDeg));
                AddReward(alignment * bodyCameraAlignmentBonus);
            }
        }

        // г) Штраф за критически близкие стены
        if (sensors != null)
        {
            if (sensors.ultrasonicNormalized < 0.05f) AddReward(-wallProximityPenalty);
            if (sensors.leftIR  == 1)                 AddReward(-wallProximityPenalty);
            if (sensors.rightIR == 1)                 AddReward(-wallProximityPenalty);
        }

        // д) Мелкий постоянный штраф — не стоять
        AddReward(-perStepPenalty);
        _rewardStep -= perStepPenalty;

        // е) Штраф за движение назад — по проекции РЕАЛЬНОЙ (вручную посчитанной по дельте
        // позиции) скорости корпуса на forward. Так штраф не срабатывает при развороте на месте
        // (там продольная составляющая ≈ 0) и корректно учитывает фактическое перемещение,
        // а не сигналы гусениц напрямую.
        float forwardSpeed = Vector3.Dot(_lastVelocity, transform.forward);
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

        // е.2) PRECISION APPROACH — награды/штрафы за скорость при близости к мячу.
        //      Далеко от мяча — скорость не важна (даже нужна для эффективного подъезда).
        //      Близко — надо замедлиться, иначе робот пролетает мимо, не может ухватить.
        //      Симметричная пара: замедлился рядом = бонус, летит рядом = штраф.
        if (curDist > 0f && curDist < precisionApproachDistance)
        {
            float speedMag = Mathf.Abs(forwardSpeed);
            if (speedMag < precisionSlowdownSpeed)
            {
                AddReward(precisionSlowdownBonus);
                _rewardDistance += precisionSlowdownBonus;  // группируем к distance-семье
            }
            else
            {
                AddReward(-precisionOverspeedPenalty);
                _rewardDistance -= precisionOverspeedPenalty;
            }
        }

        // з) Штраф за ложный захват — команда grab, когда мяч не подтверждён рядом с клешнёй.
        // Без этого агент может научиться спамить grab "на всякий случай".
        if (gripAct == 1 && (gripper == null || !gripper.isHolding))
        {
            bool ballNear = sensors != null && (float)sensors.gripperIR > grabProximityIRThreshold;
            if (!ballNear)
            {
                AddReward(-falseGrabPenalty);
            }
        }

        // к) Hold-to-succeed: удержание своего мяча N шагов подряд → терминал +grabSuccessReward.
        //    Пока держит — капает holdStepReward. Отпустил / потерял — счётчик обнуляется.
        //    Верифицируем что схвачен ИМЕННО свой мяч (см. защиту от мульти-арены).
        if (gripper != null && gripper.isHolding)
        {
            bool ownBall = true;
            if (targetBall != null && gripper.HeldRigidbody != null)
                ownBall = (gripper.HeldRigidbody.transform == targetBall);

            if (ownBall)
            {
                _gripHoldSteps++;

                // Continuous hold reward — стимулирует держать, а не хватать-бросать
                AddReward(holdStepReward);
                _rewardTerminal += holdStepReward;

                // Терминал только после N шагов удержания
                if (_gripHoldSteps >= holdStepsRequired)
                {
                    AddReward(grabSuccessReward);
                    _rewardTerminal += grabSuccessReward;
                    LogEpisodeStats(success: true);
                    EndEpisode();
                    return;
                }
            }
            else
            {
                // Захвачен ЧУЖОЙ мяч (мульти-арена баг) — отпускаем, штрафуем, не терминал.
                Debug.LogWarning($"[{name}] Захвачен ЧУЖОЙ мяч '{gripper.HeldRigidbody.name}' " +
                                 $"(мой targetBall = '{targetBall.name}'). Отпускаю.");
                gripper.Release();
                gripper.grabCommand = false;
                AddReward(-falseGrabPenalty);
                _gripHoldSteps = 0;
            }
        }
        else
        {
            // Не держит — сброс счётчика удержания
            _gripHoldSteps = 0;
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
            Debug.Log($"[{name}] SUCCESS в эпизоде #{_episodeCount}, шаг {_episodeStepCount}. " +
                      $"Схваченный мяч: {(gripper != null && gripper.HeldRigidbody != null ? gripper.HeldRigidbody.name : "null")}");

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