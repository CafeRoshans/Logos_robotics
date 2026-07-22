using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;

/// <summary>
/// ML-Agents агент для робота GFS-X.
/// Настройки Behavior Parameters (задаются в инспекторе):
///   Vector Observation Space Size = 17
///   Stacked Vectors               = 4
///   Continuous Actions            = 2   (gas [-1..1], steering [-1..1])
///   Discrete Branches             = 1 (legacy) или 0 (useAutomaticGripper=true, дефолт)
///
/// ВАЖНО: камера больше НЕ управляется сетью. Раньше сеть напрямую выдавала угол камеры
/// (camTarget action) и сильно "дёргала" им — реальный сервопривод физически не способен
/// на плавное движение, и сколько бы сеть ни обучалась, плавности взяться было неоткуда.
/// Теперь камера — автономный контур (CameraAimController): PID держит мяч в центре кадра
/// дискретными шагами (1-2°, не чаще фиксированного интервала — как настоящий серво), а при
/// потере мяча сама ищет качелями в сторону последнего известного направления. Сеть получает
/// угол камеры как наблюдение (для CollectObservations и bodyCameraAlignmentBonus) и учится
/// только ДОВОРАЧИВАТЬ КОРПУС туда же, куда сейчас смотрит камера — камерой не управляет.
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
    [Tooltip("Автономный контроллер камеры (PID + поиск). Сеть камерой больше не управляет — " +
             "см. комментарий к классу выше. Должен быть на этом же объекте или дочернем, " +
             "с cameraAim.cameraServo, указывающим на тот же Transform, что и поле выше.")]
    public CameraAimController cameraAim;
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
    [Tooltip("Максимальный угол отклонения камеры ± (градусы). Единственный источник правды — " +
             "дублируется в cameraAim.maxAngleDeg автоматически в Initialize(), чтобы наблюдение " +
             "сети (нормировка по этому значению) и реальный физический предел камеры не разъезжались. " +
             "Не меняй cameraAim.maxAngleDeg отдельно в инспекторе — перезапишется отсюда. " +
             "Реальная камера GFS-X: <45° в каждую сторону — подставь точное значение своего робота.")]
    public float cameraServoMaxAngle = 45f;

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
    public float distanceRewardScale = 1.0f;
    [Tooltip("Коэф. α в exp(α×(1-dist)): α=2 → ×7.4 у мяча, ×1.0 на 1м. Усиливает сигнал у цели без сингулярности.")]
    public float distanceRewardAlpha = 2f;
    [Tooltip("Радиус переключения Phase1→Phase2 (м). Дальше — delta-reward за сближение. Ближе — slow-approach.")]
    public float closeRadius = 0.35f;
    [Tooltip("Штраф за резкое изменение gas/steer между шагами. Сглаживает езду.")]
    public float driveRatePenalty  = 0.001f;
    [Tooltip("Штраф за критически близкую стену — ТОЛЬКО по ИК (leftIR/rightIR, жёстко " +
             "закреплены на корпусе). УЗ сюда не входит: он механически привязан к горизонтальному " +
             "повороту камеры (CameraAimController), поэтому его направление не всегда совпадает " +
             "с направлением движения — штрафовать за его показания было бы наказанием за то, " +
             "куда автономно смотрит камера, а не за реальную опасность. Каждый ИК, который " +
             "срабатывает = -этот штраф/шаг. НАМЕРЕННО маленький: сенсор сам по себе ничего плохого " +
             "не делает — он просто информирует о мире, это не повод для наказания. Наказывать нужно " +
             "за реальные последствия (см. obstacleCollisionPenalty ниже, который должен быть " +
             "на порядок больше).")]
    public float wallProximityPenalty   = 0.01f;

    [Tooltip("Штраф за окончание эпизода по таймауту")]
    public float timeoutPenalty = 1.0f;

    [Header("Столкновение с препятствием / стеной")]
    [Tooltip("Тег, который должен быть выставлен на prefab препятствия в Unity Editor. " +
             "Без правильного тега штраф не сработает.")]
    public string obstacleTag              = "Obstacle";
    [Tooltip("Тег стен арены. РАНЬШЕ штраф проверял только obstacleTag — столкновения со " +
             "стенами вообще не штрафовались (стены и препятствия — разные объекты с разными " +
             "тегами), робот мог биться о стены сколько угодно без последствий. Поставь сюда " +
             "точный тег, который реально стоит на стенах арены в сцене/префабе.")]
    public string wallTag                  = "Wall";
    [Tooltip("Разовый штраф при физическом касании препятствия ИЛИ стены хитбоксом робота. " +
             "Должен быть ощутимо больше, чем wallProximityPenalty (штраф за срабатывание " +
             "дальномера/ИК) — сенсор сам по себе ничего плохого не делает, это просто " +
             "информация о мире, наказывать за её наличие нелогично. А вот реальное физическое " +
             "столкновение — это то, чего мы на самом деле хотим избежать, поэтому и вес здесь " +
             "должен доминировать: robot should learn to react to sensors specifically BECAUSE " +
             "collision is expensive, not because proximity itself is punished.")]
    public float  obstacleCollisionPenalty = 4.0f;
    [Tooltip("Завершать эпизод при столкновении со СПАВНЯЩИМСЯ ПРЕПЯТСТВИЕМ (obstacleTag). " +
             "false = только штраф, обучение продолжается (это нормально — препятствия расставлены " +
             "случайно, лёгкое касание блока не обязательно должно быть фатальным).")]
    public bool   endEpisodeOnObstacleHit  = false;
    [Tooltip("Завершать эпизод при столкновении со СТЕНОЙ АРЕНЫ (wallTag) — НЕЗАВИСИМО от " +
             "endEpisodeOnObstacleHit выше. Врезаться в границу арены — более серьёзный провал, " +
             "чем задеть случайный блок, поэтому по умолчанию true: любое касание стены сразу " +
             "обрывает эпизод, даже если столкновения с препятствиями штрафуются мягче.")]
    public bool   endEpisodeOnWallHit      = true;

    [Tooltip("Терминальный бонус за успешный захват своего мяча.")]
    public float grabSuccessReward      = 8.0f;


    [Header("Blind approach — движение вперёд когда мяч НЕ виден")]
    [Tooltip("Бонус за каждый шаг, когда робот реально СМЕЩАЕТСЯ (по модулю скорости, не " +
             "только строго вперёд — поворот во время движения тоже считается), а мяч ещё " +
             "не виден. Стимулирует активное исследование пространства, а не стояние/кружение " +
             "на месте при потере мяча. БЫЛО слишком маленьким (0.003) и считалось только по " +
             "чисто прямой скорости — сеть находила, что дешевле и безопаснее подождать, пока " +
             "автономный поиск камеры (CameraAimController) сам наткнётся на мяч, чем рисковать " +
             "слепым проездом почти без выигрыша в награде.")]
    public float blindApproachBonus = 0.015f;
    [Tooltip("Минимальная реальная скорость (м/с, по модулю смещения — не только вперёд) для " +
             "срабатывания blindApproachBonus / searchIdlePenalty ниже.")]
    public float blindApproachMinForwardSpeed = 0.05f;
    [Tooltip("Штраф за шаг, когда мяч НЕ виден, а робот почти НЕ смещается (скорость ниже " +
             "blindApproachMinForwardSpeed) — то есть просто стоит или крутится вокруг своей " +
             "оси без реального перемещения по арене. Явно отучает от пассивного ожидания.")]
    public float searchIdlePenalty = 0.004f;

    [Header("Шум наблюдений (доменная рандомизация)")]
    [Tooltip("Амплитуда uniform-шума на УЗ (±). Значение читается из yaml, если useYamlEnvParams=true.")]
    public float ultrasonicNoise = 0.05f;
    [Tooltip("Амплитуда uniform-шума на vision angle (±)")]
    public float visionAngleNoise = 0.03f;
    [Tooltip("Амплитуда uniform-шума на vision distance (±). Обычно в 3× больше angle-шума — дистанция шумнее.")]
    public float visionDistanceNoise = 0.09f;

    [Header("Задержка сенсоров (симуляция ROS2 pipeline)")]
    [Tooltip("На сколько decisions задерживаются показания датчиков. 3 при DecisionPeriod=5 ≈ 300мс. " +
             "Отдельно от actionBuffer, потому что sensor pipeline имеет свою задержку. Читается из yaml.")]
    public int sensorLatencySteps = 3;

    [Header("Yaml environment_parameters")]
    [Tooltip("Читать ball_mass, ball_scale, vision_noise, ultrasonic_noise, sensor_latency, action_latency, " +
             "obstacle_count, ball_spawn_count, obstacle_respawn_every из config.yaml → environment_parameters. " +
             "Позволяет менять физику/обстановку без пересборки билда + поддерживает curriculum (усложнение). " +
             "НЕ включает награды/штрафы — те по прямому запросу больше НЕ читаются из yaml вообще, только " +
             "из инспектора/prefab (см. конец ReadYamlEnvParams()).")]
    public bool useYamlEnvParams = true;

    [Header("Hard-stop после захвата")]
    [Tooltip("После успешного захвата мяча принудительно обнулять команды моторов. " +
             "Иначе робот продолжает крутиться и может уронить мяч.")]
    public bool hardStopOnHold = true;

    [Header("Per-step и арена")]
    [Tooltip("Небольшой штраф за каждый шаг — стимулирует скорость решения.")]
    public float perStepPenalty = 0.0005f;
    [Tooltip("Терминальный штраф за вылет за пределы арены.")]
    public float outOfArenaPenalty      = 3.5f;

    [Header("Phase 2 — точный подъезд (dist < closeRadius)")]
    [Tooltip("Порог линейной скорости (м/с): ниже = 'медленно' для gentlePlacementBonus.")]
    public float gentleLinearSpeedThresh = 0.08f;
    [Tooltip("Порог нормированной угловой команды steer (0..1): ниже = 'почти прямо' для gentlePlacementBonus.")]
    public float gentleAngularSpeedThresh = 0.15f;
    [Tooltip("Бонус каждый шаг в Phase2, когда linear+angular ниже порогов (тихий подъезд).")]
    public float gentlePlacementBonus = 0.008f;
    [Tooltip("Штраф каждый шаг в Phase2, когда скорость превышает порог (пролёт мимо).")]
    public float gentleOverspeedPenalty = 0.015f;

    [Header("Штраф за движение назад")]
    [Tooltip("Штраф за движение назад (когда корпус реально смещается против transform.forward)")]
    public float backwardMovementPenalty = 0.01f;
    [Tooltip("Мёртвая зона по скорости (м/с), ниже которой направление не штрафуем (шум/стояние на месте)")]
    public float backwardMovementDeadzone = 0.01f;

    [Header("Streak-бонус за непрерывное удержание мяча в кадре")]
    [Tooltip("Максимальный бонус за центрирование (достигается при streak ≥ streakCap). " +
             "При streak=0 бонус = 0, растёт линейно до этого значения.")]
    public float centeringBonusMax = 0.012f;
    [Tooltip("Сколько шагов подряд нужно удерживать мяч в центре, чтобы выйти на полный бонус. " +
             "30 при DecisionPeriod=5 ≈ 3 секунды непрерывного трекинга.")]
    public int centeringStreakCap = 30;
    [Tooltip("Порог |horizontalAngle| (0..1), ниже которого мяч считается 'в центре' для streak. " +
             "0.25 = мяч в средней четверти кадра.")]
    public float centeringAngleThreshold = 0.25f;
    [Tooltip("Минимальная скорость вперёд (м/с) или поворот (норм.), при которой streak продолжается. " +
             "Если робот стоит — streak сбрасывается: нельзя заработать бонус, просто стоя и глядя.")]
    public float centeringMinMovement = 0.03f;

    [Header("Совмещение корпуса и камеры (основной стимул разворота корпуса)")]
    [Tooltip("МАКСИМАЛЬНЫЙ бонус за совмещение корпуса с камерой (при streak ≥ bodyAlignmentStreakCap) — " +
             "теперь тоже усиливается во времени, а не флэт-награда за шаг. Раньше агент получал " +
             "флэт-награду просто за то, что корпус развёрнут туда же, куда смотрит камера — и мог " +
             "один раз довернуться и стоять, собирая награду бесконечно (тот же эксплойт, что и с " +
             "centeringBonus). Теперь награда растёт, только пока робот И держит выравнивание, И " +
             "реально движется. Вес больше, чем у centeringBonus — это главный стимул именно " +
             "доворачивать корпус и потом ехать, а не просто смотреть.")]
    public float bodyCameraAlignmentBonus = 0.007f;
    [Tooltip("Допуск (градусы) между углом камеры (относительно корпуса) и 0, при котором " +
             "считаем корпус и камеру 'совмещёнными'. По ТЗ — 0..3°.")]
    public float bodyCameraAlignmentToleranceDeg = 3f;
    [Tooltip("Сколько шагов подряд нужно удерживать выравнивание корпуса с камерой (и двигаться), " +
             "чтобы выйти на полный бонус — та же логика, что и centeringStreakCap.")]
    public int   bodyAlignmentStreakCap = 30;


    [Header("Клешня")]
    [Tooltip("true (дефолт) = клешня хватает автоматически по ИК-датчику (GripperController.CanGrab), " +
             "сеть НЕ решает 'когда хватать' — Discrete Branches в Behavior Parameters должен быть 0. " +
             "false = legacy-режим, сеть сама выбирает момент через discrete action (idle/grab/release) — " +
             "нужен Discrete Branches = 1. Поле вызывалось в OnActionReceived, но нигде не было " +
             "объявлено (CS0103) — видимо, потерялось при переносе клешни на автоматический режим.")]
    public bool useAutomaticGripper = true;
    [Tooltip("Порог ИК клешни, выше которого считаем, что мяч действительно рядом")]
    public float grabProximityIRThreshold = 0.5f;

    [Header("Случайный спавн робота и мяча")]
    [Tooltip("Спавнить робота в случайной точке из obstacleSpawner.unusedPoints при каждом эпизоде")]
    public bool randomizeSpawnPositions = true;
    [Tooltip("Минимальное расстояние между роботом и мячом при спавне (м). " +
             "Ставь >= размера мяча + захвата, чтобы робот не заспавнился на мяче.")]
    public float minRobotBallDistance = 0.6f;
    [Tooltip("Небольшой случайный разброс (±град) вокруг направления 'на центр арены' при " +
             "спавне — чтобы сеть не запоминала идеально одно и то же направление на каждой " +
             "точке. БЫЛО багом: поле объявлялось, но нигде не читалось — поворот при спавне " +
             "всегда оставался _startRotation (фиксированный, из редактора) независимо от того, " +
             "какая точка выбрана, из-за чего на части точек робот оказывался повёрнут прямо " +
             "в стену/препятствие на границе арены.")]
    public bool randomizeRobotHeading = true;
    [Tooltip("Амплитуда случайного отклонения (±град) от направления на центр арены, если " +
             "randomizeRobotHeading=true.")]
    public float headingJitterDeg = 30f;

    [Header("Спавн мяча — фиксированные точки")]
    [Tooltip("Массив точек-кандидатов для мяча (например, середины 4 бортиков арены). " +
             "На каждом эпизоде случайно выбирается ОДНА. Если массив пуст — берётся точка " +
             "из obstacleSpawner.unusedPoints (старое поведение). Приоритетнее чем unusedPoints.")]
    public Transform[] ballSpawnPoints;

    [Header("Спавн точки робота")]
    [Tooltip("Массив точек кандидатов чтобы поставить робота")]
    public Transform[] robotSpawnPoints;

    [Tooltip("Сколько первых точек из ballSpawnPoints использовать. -1 = все. " +
             "Читается из yaml (ball_spawn_count) для curriculum: сначала 1 фикс. точка, потом все 9.")]
    public int ballSpawnCount = -1;

    [Header("Рандомизация массы мяча")]
    [Tooltip("На каждом эпизоде мяч получает случайную массу (Gaussian вокруг номинала). " +
             "Помогает обучить полиси быть устойчивой к разной инерции мяча.")]
    public bool randomizeBallMass = true;
    [Tooltip("Номинальная масса мяча, кг — среднее распределения Gaussian.")]
    public float ballMassMean   = 0.08f;
    [Tooltip("Стандартное отклонение массы мяча, кг.")]
    public float ballMassStddev = 0.02f;

    [Header("Рандомизация массы робота")]
    [Tooltip("На каждом эпизоде масса робота выставляется случайно (Gaussian вокруг номинала). " +
             "Работает только если Rigidbody НЕ kinematic.")]
    public bool randomizeRobotMass = true;
    [Tooltip("Номинальная масса робота, кг — среднее распределения Gaussian.")]
    public float robotMassMean   = 2.5f;
    [Tooltip("Стандартное отклонение массы робота, кг.")]
    public float robotMassStddev = 0.3f;

    [Header("Рандомизация моторов (реалистичная модель: общий фактор + per-side джиттер)")]
    [Tooltip("Включить рандомизацию моторов")]
    public bool randomizeMotors = true;
    [Tooltip("ОБЩИЙ множитель обеих гусениц за эпизод. Эмулирует уровень заряда АКБ / общий износ. " +
             "Оба мотора получают ОДНО значение из этого диапазона (не независимо). " +
             "0.85..1.15 = заряд от почти-разряженного до свежего.")]
    public float motorCommonMulMin = 0.85f;
    public float motorCommonMulMax = 1.15f;
    [Tooltip("Малый ПЕР-СТОРОННИЙ джиттер, добавляемый к каждой гусенице отдельно. " +
             "Эмулирует производственный допуск и небольшой износ редукторов. " +
             "±0.02 = максимум 2% разницы между L и R. Реалистично.")]
    [Range(0f, 0.10f)]
    public float motorPerSideJitter = 0.02f;
    [Tooltip("Разброс максимальной скорости робота — ДОЛЯ (0..1) от nominalMaxLinearCmd, " +
             "а НЕ абсолютная скорость в м/с. 0.6..1.0 = 60-100% от номинала (почти севшая / " +
             "свежая АКБ или трение). БЫЛО багом: раньше эти числа брались как абсолютные м/с, " +
             "и при nominalMaxLinearCmd=0.25 это давало tracks.maxLinearCmd = 0.6..1.0 м/с — в " +
             "2.4-4× БЫСТРЕЕ номинала (и быстрее реального робота, ~0.57 м/с). См. wearRatio ниже.")]
    public float robotMaxSpeedMin = 0.6f;
    public float robotMaxSpeedMax = 1.0f;
    [Tooltip("НОМИНАЛЬНАЯ (не изношенная) максимальная линейная скорость робота, м/с — " +
             "точка отсчёта, от которой считается коэффициент общего износа моторов для " +
             "maxAngularSpeedDeg. Должна совпадать с дефолтным Controller.maxLinearCmd.")]
    public float nominalMaxLinearCmd    = 0.25f;
    [Tooltip("НОМИНАЛЬНЫЙ (не изношенный) максимальный угол разворота, град/с — точка отсчёта " +
             "для maxAngularSpeedDeg. Должна совпадать с дефолтным Controller.maxAngularSpeedDeg.")]
    public float nominalMaxAngularSpeedDeg = 75f;
    [Tooltip("Разброс сглаживания разгона PWM. Больше — медленнее реакция мотора.")]
    public float motorPwmStepMin = 10f;
    public float motorPwmStepMax = 20f;

    [Header("YOLO Burst Dropout — потеря мяча при резких поворотах")]
    [Tooltip("Включить симуляцию смаза YOLO при быстром вращении робота")]
    public bool enableYoloBurstDropout = true;
    [Tooltip("Реальный средний FPS камеры на GFS-X (сейчас ~45). Смаз — это честная физика " +
             "'сколько робот повернулся за время ОДНОГО кадра', а не произвольная константа " +
             "град/сек: порог = cameraFps × maxBlurDegPerFrame (см. ниже).")]
    public float cameraFps = 45f;
    [Tooltip("Сколько градусов поворота ЗА ОДИН КАДР камера ещё 'прощает' без потери детекции — " +
             "грубая оценка, ПРОВЕРЬ ЭМПИРИЧЕСКИ на реальном YOLO: покрути реальный робот с " +
             "контролируемой угловой скоростью, найди скорость, на которой детекция реально " +
             "начинает пропадать, раздели на cameraFps и подставь сюда.")]
    public float maxBlurDegPerFrame = 4f;
    [Tooltip("Абсолютный фолбэк-порог (град/сек) — используется, только если cameraFps <= 0 " +
             "(FPS-модель отключена).")]
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
    private Vector3 _startPosition;
    private Quaternion _startRotation;
    private Vector3 _ballStartPosition;

    private float _cameraServoAngle = 0f;
    private float _prevDistanceToBall = -1f;
    private float _prevGas = 0f;
    private float _prevSteer = 0f;
    private float _prevCam = 0f;
    private float _currentCameraYaw = 0f;  // -1..1, абсолютное состояние камеры с rate-limit
    private float _smoothedCamTarget = 0f;  // EMA-сглаженный target от сети — глушит шум
    private float _lastObstacleHitTime = -999f; // время последнего штрафа за столкновение (cooldown)
    private float _timeSinceLastDetection = 0f;
    private float _lastKnownBallDirection = 0f;

    // Скорость корпуса, вычисленная вручную по дельте позиции (см. комментарий к классу).
    private Vector3 _prevPosition;
    private Vector3 _lastVelocity;

    private int _episodeCount = 0; // счётчик эпизодов агента, для периодического пересоздания препятствий

    // Burst dropout: сколько шагов подряд ещё нужно "слепить" YOLO
    private int _dropoutStepsLeft = 0;
    // Предыдущее значение eulerAngles.y для вычисления угловой скорости
    private float _prevHeadingDeg = 0f;

    // --- Кастомные метрики за эпизод ---
    private float _rewardDistance = 0f;
    private float _rewardWall = 0f;
    private float _rewardObstacle = 0f;  // штрафы за физические столкновения с obstacle

    private float _rewardCenter = 0f;  // бонус за центрирование мяча в кадре (streak)
    private int _centeringStreak = 0;   // шагов подряд мяч в центре + робот движется
    private int   _bodyAlignmentStreak = 0; // шагов подряд корпус совмещён с камерой + робот движется
    private float _rewardDriveRate = 0f;  // штраф за резкость gas/steer
    private float _rewardStep      = 0f;
    private float _rewardTerminal  = 0f;
    private int   _phase2Steps     = 0;   // сколько шагов за эпизод робот провёл в Phase 2
    private int   _framesBallVisible = 0;
    private int   _framesTotal       = 0;
    private float _speedAccum        = 0f;

    // --- Welford online std между эпизодами (сбрасывается в Initialize, не OnEpisodeBegin) ---
    // Для каждого компонента хранятся mean и M2; std = sqrt(M2 / (n-1))
    // Камера сюда не входит: она автономна (CameraAimController), не RL-action, и
    // соответствующих полей cameraRatePenalty/_rewardCamRate в проекте больше нет.
    private int   _wN    = 0;
    private float _wM_dist,  _wM2_dist;
    private float _wM_wall,  _wM2_wall;
    private float _wM_obs,   _wM2_obs;
    private float _wM_ctr,   _wM2_ctr;
    private float _wM_drv,   _wM2_drv;
    private float _wM_step,  _wM2_step;
    private float _wM_term,  _wM2_term;
    private int   _dropoutBurstsCount = 0; // сколько burst dropout произошло за эпизод
    private int   _wrongBallGrabbedCount = 0; // сколько раз схватили чужой мяч (мульти-арена)

    private Queue<float[]> actionBuffer = new Queue<float[]>();

    // Отдельный FIFO для задержки сенсоров (UZ, LIR, RIR, GripperIR)
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
    float VisionVerticalAngle()
    {
        if (useRealRobot) return yolo != null ? yolo.verticalAngle : 0f;
        return simCam != null ? simCam.verticalAngle : 0f;
    }
    float VisionNormalizedDistance()
    {
        if (useRealRobot) return yolo != null ? yolo.normalizedDistance : 1f;
        return simCam != null ? simCam.normalizedDistance : 1f;
    }
    float VisionConfidence()
    {
        if (useRealRobot) return yolo != null ? yolo.confidence : 0f;
        return simCam != null ? simCam.confidence : 0f;
    }

    public override void Initialize()
    {
        _rb = GetComponent<Rigidbody>();
        _startPosition = transform.position;
        _startRotation = transform.rotation;
        _prevPosition = _startPosition;
        _lastVelocity = Vector3.zero;
        if (targetBall != null) _ballStartPosition = targetBall.position;
        if (cameraAim != null)
            cameraAim.maxAngleDeg = cameraServoMaxAngle;
        else
            Debug.LogWarning($"[{name}] cameraAim не назначен — камера не будет работать вообще.");

        // Здесь раннее была проверка на принадлежность объектов к данной арене, но DEPRECATED

        // Гарантируем что буферы задержки готовы до первого CollectObservations,
        // который ML-Agents может вызвать (через NotifyAgentDone/Bootstrapping)
        // ещё ДО первого OnEpisodeBegin.
        InitSensorBuffer();
        InitActionBuffer();

        // Welford-статистика межэпизодная — сбрасываем только при полном рестарте агента
        _wN = 0;
        _wM_dist = _wM2_dist = 0f;
        _wM_wall = _wM2_wall = 0f;
        _wM_obs  = _wM2_obs  = 0f;
        _wM_ctr  = _wM2_ctr  = 0f;
        _wM_drv  = _wM2_drv  = 0f;
        _wM_step = _wM2_step = 0f;
        _wM_term = _wM2_term = 0f;
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
            // Debug каждые 5 эпизодов
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

        // Выбираем одну из четырех свободных точек
        if (randomizeSpawnPositions && obstacleSpawner != null && robotSpawnPoints != null)
        {
            int i = Random.Range(0, robotSpawnPoints.Length);
            robotPos = robotSpawnPoints[i].position;

            // Ориентация — на центр арены (т.е. на противоположную стену от точки спавна),
            // а не фиксированный _startRotation. Точки спавна стоят у границы арены с
            // identity-поворотом (без своего направления) — раньше робот мог оказаться
            // развёрнут прямо в стену/препятствие на границе. randomizeRobotHeading добавляет
            // небольшой разброс (±headingJitterDeg), чтобы не запоминалось одно и то же
            // направление на каждой точке.
            Vector3 dirToCenter = (_startPosition + arenaCenterOffset) - robotPos;
            dirToCenter.y = 0f;
            if (dirToCenter.sqrMagnitude > 0.0001f)
            {
                robotRot = Quaternion.LookRotation(dirToCenter.normalized, Vector3.up);
                if (randomizeRobotHeading)
                    robotRot = Quaternion.Euler(0f, Random.Range(-headingJitterDeg, headingJitterDeg), 0f) * robotRot;
            }
        }

        // --- Позиция мяча: случайная из ballSpawnPoints ---
        if (ballSpawnPoints != null && ballSpawnPoints.Length > 0)
        {
            // ball_spawn_count ограничивает сколько первых точек активно (curriculum).
            // -1 или 0 = все точки; иначе берём первые N.
            int limit = (ballSpawnCount > 0) ? Mathf.Min(ballSpawnCount, ballSpawnPoints.Length)
                                             : ballSpawnPoints.Length;

            // Собираем валидные точки среди первых `limit` (не null, не NaN-позиция)
            int validCount = 0;
            for (int i = 0; i < limit; i++)
                if (ballSpawnPoints[i] != null && IsValid(ballSpawnPoints[i].position))
                    validCount++;

            if (validCount > 0)
            {
                // Выбираем случайную по индексу среди валидных
                int pick = Random.Range(0, validCount);
                int idx = 0;
                for (int i = 0; i < limit; i++)
                {
                    if (ballSpawnPoints[i] == null || !IsValid(ballSpawnPoints[i].position)) continue;
                    if (idx == pick) { ballPos = ballSpawnPoints[i].position; break; }
                    idx++;
                }
            }
            else
            {
                Debug.LogWarning($"[{name}] ballSpawnPoints не содержит валидных точек (limit={limit}), использую _ballStartPosition.");
            }

            // DEPRECATED
            // Если мяч оказался слишком близко к роботу (робот заспавнился рядом
            // с выбранным бортиком), отодвигаем робота в другую свободную точку.
        }

        // Финальная проверка — если что-то дало NaN, используем безопасные значения
        if (!IsValid(robotPos)) robotPos = _startPosition;
        if (!IsValid(ballPos)) ballPos = _ballStartPosition;

        // 4. Ставим робота + рандом масса робота
        transform.SetPositionAndRotation(robotPos, robotRot);
        if (_rb != null && !_rb.isKinematic)
        {
            _rb.linearVelocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }

        // рандомим массу робота 
        if (_rb != null && !_rb.isKinematic && randomizeRobotMass)
        {
            // БЫЛО: Gaussian(2.5f) — среднее этой функции ВСЕГДА 0 (Box-Muller без сдвига),
            // то есть ~половина эпизодов давала отрицательную "массу", которая клампилась
            // в 0.001 кг (робот-пушинка). Теперь среднее и разброс — явные отдельные поля.
            float robotMass = robotMassMean + Gaussian(robotMassStddev);
            _rb.mass = Mathf.Max(0.001f, robotMass);
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
                    brb.linearVelocity = Vector3.zero;
                    brb.angularVelocity = Vector3.zero;
                }
                // Рандомизация массы — доменная рандомизация для sim-to-real.
                if (randomizeBallMass)
                {
                    // БЫЛО: Gaussian(0.1f) — тот же баг нулевого среднего, что и у робота.
                    float m = ballMassMean + Gaussian(ballMassStddev);
                    brb.mass = Mathf.Max(0.001f, m); // страховка от нуля/отрицательного
                }
            }
        }

        if (randomizeMotors && tracks != null)
        {
            // Реалистичная модель мотора: общий фактор (АКБ) + малый per-side джиттер (допуск).
            // Раннее была ошибка с большим разбросом двух моторов
            float commonMul = Random.Range(motorCommonMulMin, motorCommonMulMax);
            float leftJitter = Random.Range(-motorPerSideJitter, motorPerSideJitter);
            float rightJitter = Random.Range(-motorPerSideJitter, motorPerSideJitter);
            tracks.leftSpeedMul = commonMul * (1f + leftJitter);
            tracks.rightSpeedMul = commonMul * (1f + rightJitter);

            // Максимальная скорость робота — эмулирует АКБ/трение. robotMaxSpeedMin/Max — ДОЛЯ
            // от nominalMaxLinearCmd (см. тултип поля), не абсолютные м/с — раньше здесь было
            // Random.Range(robotMaxSpeedMin, robotMaxSpeedMax) НАПРЯМУЮ как м/с, что при
            // nominalMaxLinearCmd=0.25 разгоняло робота до 0.6-1.0 м/с (быстрее номинала И
            // реального робота) и через wearRatio ниже раздувало maxAngularSpeedDeg до сотен
            // градусов/сек — робот почти всегда пересекал angularSpeedDropoutThreshold на любом
            // повороте, и YOLO "слепла" в разы чаще, чем задумано.
            float speedFraction = Mathf.Clamp(Random.Range(robotMaxSpeedMin, robotMaxSpeedMax), 0.01f, 2f);
            tracks.maxLinearCmd = Mathf.Max(0.02f, nominalMaxLinearCmd * speedFraction);

            // Угловой предел — те же самые моторы/редукторы, что и линейный, поэтому износ
            // должен ослаблять оба предела ПРОПОРЦИОНАЛЬНО, а не оставлять maxAngularSpeedDeg
            // фиксированной константой независимо от того, насколько "сел" линейный предел.
            float wearRatio = nominalMaxLinearCmd > 0.0001f
                ? tracks.maxLinearCmd / nominalMaxLinearCmd
                : 1f;
            tracks.maxAngularSpeedDeg = Mathf.Max(1f, nominalMaxAngularSpeedDeg * wearRatio);

            // Сглаживание разгона PWM — реакция моторов
            tracks.maxPwmStep = Mathf.Max(1f, Random.Range(motorPwmStepMin, motorPwmStepMax));
        }

        // Сервопривод камеры — сброс всего состояния (угол, PID, поиск) в автономном контроллере
        if (cameraAim != null) cameraAim.ResetState();

        // Служебные переменные наград
        _prevDistanceToBall     = DistanceToBall();
        _prevGas                = 0f;
        _prevSteer              = 0f;
        _lastObstacleHitTime    = -999f;
        _timeSinceLastDetection = 0f;
        _lastKnownBallDirection = 0f;

        // Сброс вручную-считаемой скорости — иначе первый шаг нового эпизода
        _prevPosition = transform.position;
        _lastVelocity = Vector3.zero;

        // Сброс кастомных счётчиков
        _rewardDistance = 0f;
        _rewardWall = 0f;
        _rewardObstacle = 0f;
        _rewardCenter = 0f;
        _centeringStreak = 0;
        _bodyAlignmentStreak = 0;
        _rewardDriveRate = 0f;
        _rewardStep      = 0f;
        _rewardTerminal  = 0f;
        _phase2Steps     = 0;
        _framesBallVisible = 0;
        _framesTotal = 0;
        _speedAccum = 0f;

        // Burst dropout сбрасываем на начало эпизода
        _dropoutStepsLeft = 0;
        _prevHeadingDeg = transform.eulerAngles.y;

        // Читаем yaml environment_parameters — награды, физика, шумы, latency.
        // Вызывается ДО инициализации буферов — yaml может переопределить latency,
        // и буферы нужно заполнять уже с актуальными значениями.
        currentActionLatency = (int)UnityEngine.Random.Range(2f, 5f);
        ReadYamlEnvParams();

        // Буферы инициализируем ПОСЛЕ yaml — используют актуальные latency-значения.
        InitActionBuffer();
        InitSensorBuffer();
    }

    static float Gaussian(float stddev)
    {
        float u1 = 1f - Random.value;
        float u2 = 1f - Random.value;
        float z = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
        return z * stddev;
    }

    // Online mean/variance между эпизодами (алгоритм Уэлфорда) — вызывались из LogEpisodeStats,
    // но сами методы отсутствовали в проекте (CS0103 после мержа). mean обновляется по месту
    // (ref), m2 — накопленная сумма квадратов отклонений, из которой WelfordStd достаёт std.
    static void WelfordUpdate(float newValue, ref float mean, ref float m2, int n)
    {
        float delta = newValue - mean;
        mean += delta / n;
        float delta2 = newValue - mean;
        m2 += delta * delta2;
    }

    static float WelfordStd(float m2, int n)
    {
        return n >= 2 ? Mathf.Sqrt(m2 / (n - 1)) : 0f;
    }

    public override void CollectObservations(VectorSensor sensor)
    {

        // Шумы на датчики + задержки через
        float rawUS = sensors != null ? sensors.ultrasonicNormalized : 1f;
        float noisyUS = Mathf.Clamp01(rawUS + Gaussian(ultrasonicNoise));
        float rawLIR = sensors != null ? sensors.leftIR : 0;
        float rawRIR = sensors != null ? sensors.rightIR : 0;
        float rawGIR = sensors != null ? sensors.gripperIR : 0;

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

        // наблюдения 5-8 vision с шумом
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

        // 9. Угол ПОВОРОТА камеры (yaw) относительно корпуса, нормализованный — состояние
        // автономного CameraAimController, а не что-то, что задаёт сеть.
        float camAngleDeg = cameraAim != null ? cameraAim.CurrentAngleDeg : 0f;
        sensor.AddObservation(Mathf.Clamp(camAngleDeg / Mathf.Max(1f, cameraServoMaxAngle), -1f, 1f));

        // 10. Угол НАКЛОНА камеры (tilt) относительно корпуса, нормализованный [-1..1] по
        // диапазону [cameraAim.minTiltDeg .. cameraAim.maxTiltDeg]. Помогает сети точнее
        // судить о геометрии — например, отличать "мяч близко и камера сильно опущена"
        // от "мяч далеко, камера почти горизонтально".
        // TODO (НЕ ПОДТВЕРЖДЕНО, на 2026-07-21): физически не проверено, есть ли на реальном
        // GFS-X отдельный сервопривод под наклон камеры (см. подробный TODO в
        // CameraAimController.cs про Robot/config.py SERVO_CAMERA_LOW). Если выяснится, что
        // тилта физически нет — это наблюдение нужно убрать по правилу "нет на реальном
        // роботе → не должно быть и в симуляции" (та же причина, по которой убрали heading
        // и delta.x/delta.z).
        float camTiltDeg = cameraAim != null ? cameraAim.CurrentTiltDeg : 0f;
        float tiltRange = cameraAim != null
            ? Mathf.Max(0.001f, cameraAim.maxTiltDeg - cameraAim.minTiltDeg)
            : 1f;
        float tiltMid = cameraAim != null ? (cameraAim.maxTiltDeg + cameraAim.minTiltDeg) * 0.5f : 0f;
        sensor.AddObservation(Mathf.Clamp((camTiltDeg - tiltMid) / (tiltRange * 0.5f), -1f, 1f));

        // 11. hasBall
        sensor.AddObservation(gripper != null && gripper.isHolding ? 1f : 0f);

        // heading (курс робота) убран — на реальном GFS-X нет компаса/IMU, физически
        // измерить нечем. Было подтверждено тем, что numpy_brain.py на Pi зануляет этот же
        // слот заглушкой (obs[12]=0.0) — заглушка была честнее, чем то, что было в симуляции.

        // 12. Скорость робота (м/с) — вручную посчитанная по дельте позиции,
        // а не rb.linearVelocity (для кинематического Rigidbody она всегда 0).
        sensor.AddObservation(_lastVelocity.magnitude);

        // 13. Время с последней детекции мяча (сек)
        sensor.AddObservation(_timeSinceLastDetection);

        // 14. Уверенность детекции (0..1) — на реальном роботе это packet.conf от YOLO
        // (реально приходит по UDP, раньше просто выбрасывался), в симуляции — синтетический
        // аналог, падающий с дистанцией (см. SimulatedYoloCamera.confidence). Даёт сети сигнал
        // "насколько верить" текущему наблюдению вместо жёсткого бинарного visible.
        sensor.AddObservation(visible ? VisionConfidence() : 0f);

        // 15. Камера в режиме ПОИСКА (мяч потерян, качается) vs СЛЕЖЕНИЯ. Без этого сеть не
        // отличит "камера уверенно ведёт цель" от "камера мечется в поиске" по одному лишь углу.
        sensor.AddObservation(cameraAim != null && cameraAim.IsSearching ? 1f : 0f);

        // 16..17. Предыдущее действие (gas/steer с прошлого решения) — реальный робот всегда
        // знает, что сам последний раз скомандовал, это не "нечестная" информация.
        sensor.AddObservation(_prevGas);
        sensor.AddObservation(_prevSteer);
    }

    /// <summary>
    /// Камера — автономный контур, физически не привязанный к decision period ML-Agents
    /// (как и настоящая прошивка сервопривода). Поэтому двигаем её здесь, в обычном
    /// Unity FixedUpdate, а не внутри OnActionReceived (который вызывается только на
    /// decision-шагах и был бы слишком редким для плавного слежения/поиска).
    ///
    /// Видимость (ballVisible) берём через SeesBallEffective() — то же самое, что видит
    /// сеть в наблюдениях (с учётом YOLO burst dropout при резком повороте): камера
    /// физически не может отследить то, чего "не видит" в данный момент реальная YOLO.
    /// Угол ошибки — RAW (VisionHorizontalAngle(), БЕЗ синтетического шума для RL) —
    /// шум добавляется только к тому, что видит сеть в CollectObservations, а не к
    /// сигналу, которым реально управляет физическое железо.
    /// </summary>
    void FixedUpdate()
    {
        if (cameraAim == null) return;
        bool visible = SeesBallEffective();
        float angleH = VisionHorizontalAngle();
        float angleV = VisionVerticalAngle();
        cameraAim.Tick(visible, angleH, angleV, Time.fixedDeltaTime);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        UpdateBurstDropout();

        // Штрафуем за долгую езду
        if (MaxStep > 0 && StepCount >= MaxStep - 1)
        {
            AddReward(-timeoutPenalty);
            _rewardTerminal -= timeoutPenalty;
            LogEpisodeStats(success: false);
            return;
        }

        // Считываем свежие сигналы от сети.
        //   ContinuousActions[0] = gas      ∈ [-1..1]  (вперёд / назад)
        //   ContinuousActions[1] = steering ∈ [-1..1]  (влево / вправо)
        // Камера больше не action — управляется автономно в FixedUpdate() (CameraAimController).
        float freshGas   = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float freshSteer = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
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
        actionBuffer.Enqueue(new float[] { freshGas, freshSteer });
        float[] delayed = actionBuffer.Count > 0
            ? actionBuffer.Dequeue()
            : new float[] { 0f, 0f };

        float gas   = delayed[0];
        float steer = delayed[1];

        // 1. Движение — Gas + Steering, Controller раскладывает в L/R честной инверсной
        //    кинематикой по реальной колее (trackWidth). Совместимо с ROS /cmd_vel
        //    (linear.x = gas, angular.z = steer). HARD-STOP после захвата: пока держим
        //    мяч, силой обнуляем моторы, чтобы не крутиться и не уронить.
        if (tracks != null)
        {
            bool holdingBall = hardStopOnHold && gripper != null && gripper.isHolding;
            if (holdingBall) tracks.Move(0f, 0f);
            else tracks.Move(gas, steer);
        }

        // 2. Клешня
        if (gripper != null)
        {
            // Автоматический режим: grabCommand всегда true.
            // GripperController.CanGrab() сам решит хватать по IR-датчику клешни.
            gripper.grabCommand = true;
            if (useRealRobot && rosBridge != null && gripper.isHolding)
                rosBridge.PublishGripperCmd(2); // закрыть клешню на реальном роботе

        }

        // 3. Обновление служебных переменных детекции — используем эффективную видимость
        if (SeesBallEffective())
        {
            _lastKnownBallDirection = VisionHorizontalAngle();
            _timeSinceLastDetection = 0f;
        }
        else
        {
            _timeSinceLastDetection += Time.deltaTime;
        }

        // 3b. Пересчёт скорости корпуса по дельте позиции — ДО ComputeRewards.
        Vector3 currentPosition = transform.position;
        float dt = Mathf.Max(Time.deltaTime, 0.0001f);
        _lastVelocity = (currentPosition - _prevPosition) / dt;
        _prevPosition = currentPosition;

        // 3c. ROS: отправляем команды на реальный робот (только в режиме useRealRobot).
        // Угол камеры публикуем из cameraAim — она же им реально управляет в симуляции.
        // NB: если у реального робота своя автономная прошивка камеры (собственно ради
        // чего и делался CameraAimController — как аналог такой прошивки), эта строка
        // с PublishCameraCmd может быть не нужна на реальном роботе — тогда просто убери её.
        if (useRealRobot && rosBridge != null)
        {
            rosBridge.PublishCommand(gas, steer);
            rosBridge.PublishCameraCmd(cameraAim != null ? cameraAim.CurrentAngleDeg : 0f);
        }

        // 4. Награды
        ComputeRewards(gas, steer, gripAct);

        _prevGas = gas;
        _prevSteer = steer;
    }

    void ComputeRewards(float gas, float steer, int gripAct)
    {
        // Кадровые счётчики для метрик — считаем эффективную видимость,
        // чтобы Custom/BallVisibleFraction отражал реальный сигнал, доходящий до модели.
        _framesTotal++;
        bool ballVisible = SeesBallEffective();
        if (ballVisible) _framesBallVisible++;
        _speedAccum += _lastVelocity.magnitude;

        // а) Двухфазная логика сближения с мячом.
        //    Phase 1 (dist ≥ closeRadius): delta × scale × exp(α×(1-dist)).
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

        // б) Штраф за резкость управления (только gas/steer — камера больше не action,
        //    её "резкость" физически ограничена в CameraAimController дискретным шагом
        //    и минимальным интервалом между шагами, штрафовать сеть за это больше не нужно).
        float dGas   = gas   - _prevGas;
        float dSteer = steer - _prevSteer;
        float rDrive = -driveRatePenalty * (dGas * dGas + dSteer * dSteer);
        AddReward(rDrive);
        _rewardDriveRate += rDrive;

        // в) Streak-бонусы за непрерывное удержание хорошего состояния — ОБА теперь
        //    усиливаются во времени, а не флэт-награда за факт состояния на этом шаге.
        //
        //    STREAK-МЕХАНИКА (в.1) — мяч в центре кадра:
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
        //    STREAK-МЕХАНИКА (в.2) — bodyCameraAlignment, ПО ТОЙ ЖЕ ЛОГИКЕ:
        //    Раньше была флэт-награда за каждый шаг, где корпус развёрнут туда же, куда
        //    смотрит камера (alignment × bodyCameraAlignmentBonus) — тот же самый эксплойт:
        //    один раз довернуться и стоять, собирая награду бесконечно. Теперь тоже streak
        //    с тем же условием движения — награда растёт, только пока робот И держит
        //    выравнивание, И реально движется, а не просто стоит выровненным.
        //    Streak и centering НЕЗАВИСИМЫ друг от друга (свои счётчики, свои cap'ы) —
        //    можно иметь alignment без centering (тело повёрнуто, мяч только появился
        //    в кадре, ещё не успел "зацентроваться") и наоборот.
        bool ballCentered = VisionIsVisible() && Mathf.Abs(VisionHorizontalAngle()) < centeringAngleThreshold;
        bool robotMoving = Mathf.Abs(forwardSpeed) > centeringMinMovement
                            || Mathf.Abs(steer) > centeringMinMovement;

        if (ballCentered && robotMoving)
            _centeringStreak = Mathf.Min(_centeringStreak + 1, centeringStreakCap);
        else
            _centeringStreak = 0;

        if (_centeringStreak > 0)
        {
            float streakFactor = (float)_centeringStreak / centeringStreakCap;           // 0..1
            float centered = 1f - Mathf.Abs(VisionHorizontalAngle());                // точность
            float rCenter = streakFactor * centeringBonusMax * centered;
            AddReward(rCenter); _rewardCenter += rCenter;
        }

        float absServoAngle = Mathf.Abs(cameraAim != null ? cameraAim.CurrentAngleDeg : 0f);
        bool  bodyAligned    = VisionIsVisible() && absServoAngle <= bodyCameraAlignmentToleranceDeg;

        if (bodyAligned && robotMoving)
            _bodyAlignmentStreak = Mathf.Min(_bodyAlignmentStreak + 1, bodyAlignmentStreakCap);
        else
            _bodyAlignmentStreak = 0;

        if (_bodyAlignmentStreak > 0)
        {
            float alignStreakFactor = (float)_bodyAlignmentStreak / bodyAlignmentStreakCap; // 0..1
            float alignment = 1f - (absServoAngle / Mathf.Max(0.0001f, bodyCameraAlignmentToleranceDeg));
            float rAlign    = alignStreakFactor * bodyCameraAlignmentBonus * alignment;
            AddReward(rAlign); _rewardCenter += rAlign;
        }

        // г) Штраф за критически близкие стены — ТОЛЬКО по ИК (leftIR/rightIR).
        //    УЗ (ultrasonicNormalized) сюда сознательно НЕ включён: датчик механически
        //    привязан к горизонтальному повороту камеры (см. CameraAimController), а не
        //    к корпусу. Его направление во время поиска мяча может смотреть куда угодно,
        //    не совпадая с направлением движения — штрафовать за "видит близко" в этом
        //    случае означало бы наказывать робота за то, куда автономно смотрит камера,
        //    а не за реальную опасность на пути движения. УЗ остаётся информационным
        //    наблюдением для сети (см. CollectObservations), просто без отдельной награды.
        //    ИК же жёстко закреплены на корпусе — их направление стабильно и осмысленно.
        if (sensors != null)
        {
            float wallPen = 0f;
            if (sensors.ultrasonicNormalized < 0.05f) wallPen += wallProximityPenalty;
            if (sensors.leftIR == 1) wallPen += wallProximityPenalty;
            if (sensors.rightIR == 1) wallPen += wallProximityPenalty;
            if (wallPen > 0f) { AddReward(-wallPen); _rewardWall -= wallPen; }
        }

        // д) Мелкий постоянный штраф — не стоять
        AddReward(-perStepPenalty);
        _rewardStep -= perStepPenalty;

        // е) Штраф за движение назад
        if (forwardSpeed < -backwardMovementDeadzone)
        {
            AddReward(-Mathf.Abs(forwardSpeed) * backwardMovementPenalty);
        }

        // е.1) BLIND SEARCH — бонус за реальное перемещение по арене (по модулю скорости,
        // не только строго вперёд — поворот во время движения тоже считается) когда мяч НЕ
        // виден, и симметричный штраф, если робот почти не смещается (стоит или крутится на
        // месте без толку). Раньше считалось только по forwardSpeed строго по прямой и без
        // штрафа за пассивность — сеть научилась просто ждать, пока автономный поиск камеры
        // сам наткнётся на мяч, вместо активного исследования телом (а тело двигать НАДО,
        // когда мяч вне сектора поиска камеры — сама камера сектор не расширит).
        if (!ballVisible)
        {
            float searchSpeed = _lastVelocity.magnitude;
            if (searchSpeed > blindApproachMinForwardSpeed)
            {
                AddReward(blindApproachBonus);
                _rewardCenter += blindApproachBonus;  // логируем как part of "center/search" cluster
            }
            else
            {
                AddReward(-searchIdlePenalty);
                _rewardCenter -= searchIdlePenalty;
            }
        }

        // е.2) PHASE 2 — медленный точный подъезд (dist < closeRadius).
        //      Delta-reward здесь отключён (см. блок «а»). Вместо него:
        //      бонус если linear И angular команда ниже порогов (робот вкатывается тихо),
        //      штраф если едет с газом — пролетит мимо, не поместит мяч в клешню.
        if (curDist > 0f && curDist < closeRadius)
        {
            _phase2Steps++;
            float linSpeed = Mathf.Abs(forwardSpeed);
            float angCmd = Mathf.Abs(steer);
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

            // Вообще можно убрать так как требовалось на этапе дебага
            // Но пока оставляем в коде
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
                _wrongBallGrabbedCount++;
                Debug.LogWarning($"[{name}] Захвачен ЧУЖОЙ мяч '{gripper.HeldRigidbody.name}' " +
                                 $"(мой targetBall = '{targetBall.name}'). Отпускаю.");
                gripper.Release();
                gripper.grabCommand = false;
            }
        }

        // л) Терминал: вылет за арену.
        // Границы отсчитываются от стартовой позиции + локальный offset арены,
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
        s.Add("Custom/Reward/Distance", _rewardDistance);
        s.Add("Custom/Reward/Wall", _rewardWall);
        s.Add("Custom/Reward/Obstacle", _rewardObstacle);
        s.Add("Custom/Reward/Center", _rewardCenter);
        s.Add("Custom/Reward/DriveRate", _rewardDriveRate);
        s.Add("Custom/Reward/Step",      _rewardStep);
        s.Add("Custom/Reward/Terminal",  _rewardTerminal);

        // Стандартное отклонение между эпизодами (алгоритм Уэлфорда).
        // Показывает стабильность: падающий std = агент ведёт себя всё предсказуемее.
        _wN++;
        WelfordUpdate(_rewardDistance,  ref _wM_dist, ref _wM2_dist, _wN);
        WelfordUpdate(_rewardWall,      ref _wM_wall, ref _wM2_wall, _wN);
        WelfordUpdate(_rewardObstacle,  ref _wM_obs,  ref _wM2_obs,  _wN);
        WelfordUpdate(_rewardCenter,    ref _wM_ctr,  ref _wM2_ctr,  _wN);
        WelfordUpdate(_rewardDriveRate, ref _wM_drv,  ref _wM2_drv,  _wN);
        WelfordUpdate(_rewardStep,      ref _wM_step, ref _wM2_step, _wN);
        WelfordUpdate(_rewardTerminal,  ref _wM_term, ref _wM2_term, _wN);
        if (_wN >= 2)
        {
            s.Add("Custom/Std/Distance",  WelfordStd(_wM2_dist, _wN));
            s.Add("Custom/Std/Wall",      WelfordStd(_wM2_wall, _wN));
            s.Add("Custom/Std/Obstacle",  WelfordStd(_wM2_obs,  _wN));
            s.Add("Custom/Std/Center",    WelfordStd(_wM2_ctr,  _wN));
            s.Add("Custom/Std/DriveRate", WelfordStd(_wM2_drv,  _wN));
            s.Add("Custom/Std/Step",      WelfordStd(_wM2_step, _wN));
            s.Add("Custom/Std/Terminal",  WelfordStd(_wM2_term, _wN));
        }

        // Сколько шагов за эпизод робот провёл в Phase 2 (близко к мячу)
        s.Add("Custom/Phase2Steps", _phase2Steps);

        // Максимальный streak центрирования за эпизод — показывает качество трекинга
        s.Add("Custom/CenteringStreakMax", _centeringStreak);
        s.Add("Custom/BodyAlignmentStreakMax", _bodyAlignmentStreak);

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

        // Захват мяча: чужой мяч (мульти-арена) + способ захвата (ИК-датчик vs резервная сфера)
        s.Add("Custom/Grab/WrongBallGrabbed", _wrongBallGrabbedCount);
        if (gripper != null)
        {
            s.Add("Custom/Grab/ViaSensor",   gripper.grabsViaSensorThisEpisode);
            s.Add("Custom/Grab/ViaFallback", gripper.grabsViaFallbackThisEpisode);
        }

        // Параметры моторов и масс за этот эпизод — видно распределение доменной рандомизации
        if (tracks != null)
        {
            s.Add("Custom/Motors/LeftSpeedMul", tracks.leftSpeedMul);
            s.Add("Custom/Motors/RightSpeedMul", tracks.rightSpeedMul);
            s.Add("Custom/Motors/Asymmetry", Mathf.Abs(tracks.leftSpeedMul - tracks.rightSpeedMul));
            s.Add("Custom/Motors/MaxLinearCmd", tracks.maxLinearCmd);
            s.Add("Custom/Motors/MaxAngularSpeedDeg", tracks.maxAngularSpeedDeg);
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

        float gas = 0f, steer = 0f;
        int   g = 0;

        var kb = Keyboard.current;
        if (kb != null)
        {
            // WASD — драйверские команды:
            //   W/S — газ (вперёд/назад)
            //   A/D — руль (влево/вправо)
            //   Space — grab, X — release
            // Камера больше не читается с клавиатуры — она автономна (CameraAimController)
            // и работает сама в FixedUpdate() независимо от Behavior Type (Heuristic/Default).
            gas   = kb.wKey.ReadValue() - kb.sKey.ReadValue();
            steer = kb.dKey.ReadValue() - kb.aKey.ReadValue();
            if      (kb.spaceKey.isPressed) g = 1;
            else if (kb.xKey.isPressed)     g = 2;
        }

        cont[0] = gas;
        cont[1] = steer;
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
    /// Читает environment_parameters из config.yaml (Academy) и обновляет соответствующие поля —
    /// ТОЛЬКО физику/обстановку/куррикулум (мяч, шум, латентность, препятствия, точки спавна).
    /// Вызывается в OnEpisodeBegin — параметры могут меняться между эпизодами (curriculum).
    /// Если yaml не задал параметр, оставляем текущее значение из инспектора.
    /// Награды/штрафы СОЗНАТЕЛЬНО не читаются отсюда — см. комментарий в конце метода.
    /// </summary>
    void ReadYamlEnvParams()
    {
        if (!useYamlEnvParams) return;
        var env = Academy.Instance.EnvironmentParameters;

        // ── Мяч: масса, размер ──────────────────────────────────────────────
        if (targetBall != null)
        {
            var brb = targetBall.GetComponent<Rigidbody>();
            if (brb != null && !brb.isKinematic)
            {
                float v = env.GetWithDefault("ball_mass", -1f);
                if (v > 0f) brb.mass = v;
            }
            float s = env.GetWithDefault("ball_scale", -1f);
            if (s > 0.001f) targetBall.localScale = Vector3.one * s;
        }

        // ── Шум наблюдений ──────────────────────────────────────────────────
        {
            float v = env.GetWithDefault("vision_noise", -1f);
            if (v >= 0f) { visionAngleNoise = v; visionDistanceNoise = v * 3f; }
        }
        {
            float v = env.GetWithDefault("ultrasonic_noise", -1f);
            if (v >= 0f) ultrasonicNoise = v;
        }

        // ── Латентность ─────────────────────────────────────────────────────
        {
            float v = env.GetWithDefault("sensor_latency", -1f);
            if (v >= 0f) sensorLatencySteps = Mathf.Max(0, (int)v);
        }
        {
            float v = env.GetWithDefault("action_latency", -1f);
            if (v >= 0f) currentActionLatency = Mathf.Max(0, (int)v);
        }

        // ── Препятствия (curriculum) ─────────────────────────────────────────
        {
            float v = env.GetWithDefault("obstacle_count", -1f);
            if (v >= 0f && obstacleSpawner != null) obstacleSpawner.spawnCount = (int)v;
        }

        // ── Точки спавна мяча (curriculum) ───────────────────────────────────
        // 1 = только первая точка (простой этап), 9 = все точки (полная рандомизация)
        {
            float v = env.GetWithDefault("ball_spawn_count", -1f);
            if (v >= 0f) ballSpawnCount = (int)v;
        }

        // ── Respawn препятствий ───────────────────────────────────────────────
        // 1 = каждый эпизод, N = каждые N эпизодов, 0 = никогда
        {
            float v = env.GetWithDefault("obstacle_respawn_every", -1f);
            if (v >= 0f) obstacleRespawnEveryEpisodes = (int)v;
        }

        // Награды/штрафы (distance_reward_*, *_penalty, *_bonus и т.д.) СОЗНАТЕЛЬНО НЕ
        // читаются из yaml — по прямому запросу: куррикулум (усложнение обстановки — шум,
        // латентность, препятствия, точки спавна выше) должен работать, а тюнинг наград —
        // только через инспектор/prefab, чтобы его нельзя было незаметно перезаписать при
        // запуске через mlagents-learn. Раньше тут был блок на ~25 полей, дублирующий ровно
        // то, что уже задано в ArenaPoint.prefab — соответствующие ключи убраны и из
        // config.yaml (см. комментарий там). Если понадобится вернуть — это была секция
        // "КОНСТАНТЫ: награды и штрафы" в config.yaml, git history её помнит.
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

        // Смаз = поворот за время экспозиции ОДНОГО кадра реальной камеры (1/cameraFps сек),
        // а не абсолютная константа град/сек — так порог остаётся физически осмысленным
        // независимо от того, как рандомизируется мотор в конкретном эпизоде.
        float effectiveDropoutThreshold = cameraFps > 0.01f
            ? cameraFps * maxBlurDegPerFrame
            : angularSpeedDropoutThreshold;

        if (enableYoloBurstDropout && angularSpeedDegPerSec > effectiveDropoutThreshold && _dropoutStepsLeft <= 0)
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

    // Штраф за физическое столкновение с препятствием ИЛИ стеной арены.
    // Cooldown 0.5с — чтобы один удар давал ровно один штраф, а не спам каждый FixedUpdate,
    // пока хитбокс продолжает контактировать (OnCollisionStay тоже вызывал бы этот метод,
    // если бы был подписан — сейчас подписан только OnCollisionEnter, что уже покрывает
    // "удар", а не "трение вдоль стены", это осознанно).
    void OnCollisionEnter(Collision col)
    {
        ReportHitboxCollision(col);
    }

    /// <summary>
    /// Публичный метод-обработчик столкновения — вызывается либо напрямую из
    /// OnCollisionEnter здесь (если хитбокс висит на этом же GameObject, где Rigidbody),
    /// либо из вспомогательного компонента-ретранслятора на ДОЧЕРНЕМ объекте с отдельным
    /// хитбоксом (см. HitboxCollisionRelay.cs) — если у робота несколько отдельных
    /// коллайдеров на разных дочерних объектах, повесь этот компонент на каждый из них
    /// и укажи ссылку на этот RobotBrain, чтобы столкновения обоих хитбоксов гарантированно
    /// доходили сюда одним и тем же путём с общим cooldown (не по два штрафа за один удар).
    /// </summary>
    public void ReportHitboxCollision(Collision col)
    {
        bool isObstacle = col.gameObject.CompareTag(obstacleTag);
        bool isWall      = col.gameObject.CompareTag(wallTag);
        if (!isObstacle && !isWall) return;

        if (Time.time - _lastObstacleHitTime < 0.5f) return;

        _lastObstacleHitTime = Time.time;
        AddReward(-obstacleCollisionPenalty);
        _rewardObstacle -= obstacleCollisionPenalty;

        // Стена и препятствие теперь решают завершение эпизода НЕЗАВИСИМО друг от друга —
        // касание стены (границы арены) считается более серьёзным провалом и по умолчанию
        // всегда обрывает эпизод, даже если столкновения с обычными препятствиями настроены
        // мягче (только штраф, без завершения).
        bool shouldEndEpisode = (isWall && endEpisodeOnWallHit)
                              || (isObstacle && endEpisodeOnObstacleHit);

        if (shouldEndEpisode)
        {
            LogEpisodeStats(success: false);
            EndEpisode();
        }
    }
}