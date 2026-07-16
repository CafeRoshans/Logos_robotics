using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.InputSystem;

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

    [Header("Сервопривод камеры")]
    [Tooltip("Максимальный угол отклонения камеры ± (градусы). Специально НЕ ограничиваем узко — " +
             "камера должна свободно 'осматриваться' и искать мяч в широком диапазоне. Награда за " +
             "центрирование мяча в кадре есть (centeringBonus), но основной и более весомый стимул — " +
             "довернуть КОРПУС туда же, куда смотрит камера (bodyCameraAlignmentBonus), так что широкий " +
             "диапазон камеры не создаёт лазейки 'стою и просто верчу камерой'.")]
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
    [Tooltip("Множитель награды за сближение с мячом (Δd, м)")]
    public float distanceRewardScale   = 1.0f;
    [Tooltip("Порог 'близко' — ниже него награда за сближение усиливается")]
    public float closeDistanceThreshold = 0.5f;
    [Tooltip("Множитель усиления награды за сближение вблизи мяча")]
    public float closeDistanceBonusMul  = 3.0f;
    [Tooltip("Штраф за резкое изменение управляющих сигналов между шагами")]
    public float actionRatePenalty      = 0.001f;
    [Tooltip("Штраф за критически близкую стену (по ИК/УЗ)")]
    public float wallProximityPenalty   = 0.02f;
    [Tooltip("Терминальный бонус за успешный захват мяча")]
    public float grabSuccessReward      = 5.0f;
    [Tooltip("Терминальный штраф за вылет за пределы арены")]
    public float outOfArenaPenalty      = -2.0f;
    [Tooltip("Небольшой штраф за каждый шаг — стимулирует скорость решения")]
    public float perStepPenalty         = -0.0005f;

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

    [Header("Страховка на случай, если Max Step в инспекторе Agent не спасает")]
    [Tooltip("Жёсткий лимит на количество вызовов OnActionReceived за эпизод. 0 = выключено.")]
    public int hardEpisodeStepLimit = 3000;
    [Tooltip("Штраф за окончание эпизода по таймауту")]
    public float timeoutPenalty = -0.5f;

    // --- служебные ---
    private Rigidbody _rb;
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

    // Скорость корпуса, вычисленная вручную по дельте позиции (см. комментарий к классу).
    private Vector3 _prevPosition;
    private Vector3 _lastVelocity;

    public override void Initialize()
    {
        _rb = GetComponent<Rigidbody>();
        _startPosition = transform.position;
        _startRotation = transform.rotation;
        _prevPosition = _startPosition;
        _lastVelocity = Vector3.zero;
        if (targetBall != null) _ballStartPosition = targetBall.position;
    }

    public override void OnEpisodeBegin()
    {
        // Робот
        transform.SetPositionAndRotation(_startPosition, _startRotation);
        if (_rb != null)
        {
            _rb.linearVelocity  = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }
        if (tracks != null)
        {
            tracks.SetTrackInputs(0f, 0f);
        }

        // Мяч — обязательно отпустить, если был в клешне, ДО возвращения позиции
        if (gripper != null && gripper.isHolding) gripper.Release();
        if (gripper != null) gripper.grabCommand = false;

        if (targetBall != null)
        {
            targetBall.position = _ballStartPosition;
            var brb = targetBall.GetComponent<Rigidbody>();
            if (brb != null)
            {
                brb.linearVelocity  = Vector3.zero;
                brb.angularVelocity = Vector3.zero;
            }
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

        // Сброс вручную-считаемой скорости — иначе первый шаг нового эпизода
        // засчитает "прыжок" из старой позиции конца прошлого эпизода в стартовую как движение.
        _prevPosition = transform.position;
        _lastVelocity = Vector3.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. УЗ нормализованный
        sensor.AddObservation(sensors != null ? sensors.ultrasonicNormalized : 1f);
        // 2. Левый ИК препятствия
        sensor.AddObservation(sensors != null ? (float)sensors.leftIR : 0f);
        // 3. Правый ИК препятствия
        sensor.AddObservation(sensors != null ? (float)sensors.rightIR : 0f);
        // 4. ИК клешни
        sensor.AddObservation(sensors != null ? (float)sensors.gripperIR : 0f);

        // 5. Горизонтальный угол до мяча по камере (0, если не виден)
        bool visible = yolo != null && yolo.isVisible;
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
        if (hardEpisodeStepLimit > 0 && _episodeStepCount >= hardEpisodeStepLimit)
        {
            Debug.Log($"[RobotBrain] TIMEOUT эпизода на шаге {_episodeStepCount}. EndEpisode.");
            AddReward(timeoutPenalty);
            EndEpisode();
            return;
        }

        // Теперь сеть напрямую выдаёт скорость каждой гусеницы — так же, как их
        // задавала бы клавиатура через TrackController.SetTrackInputs().
        float leftTrack  = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float rightTrack = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float camSignal  = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        int   gripAct    = actions.DiscreteActions[0]; // 0 = idle, 1 = grab, 2 = release

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

        // 4. Обновление служебных переменных детекции
        if (yolo != null && yolo.isVisible)
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
        ComputeRewards(leftTrack, rightTrack, gripAct);

        _prevLeft  = leftTrack;
        _prevRight = rightTrack;
    }

    void ComputeRewards(float leftTrack, float rightTrack, int gripAct)
    {
        // а) Сближение с мячом (delta distance)
        float curDist = DistanceToBall();
        if (_prevDistanceToBall > 0f && curDist > 0f)
        {
            float delta = _prevDistanceToBall - curDist; // + = приблизились
            float mul   = (curDist < closeDistanceThreshold) ? closeDistanceBonusMul : 1f;
            AddReward(delta * distanceRewardScale * mul);
        }
        _prevDistanceToBall = curDist;

        // б) Штраф за резкость управления
        float dLeft  = Mathf.Abs(leftTrack  - _prevLeft);
        float dRight = Mathf.Abs(rightTrack - _prevRight);
        AddReward(-(dLeft + dRight) * actionRatePenalty);

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
        AddReward(perStepPenalty);

        // е) Штраф за движение назад — по проекции РЕАЛЬНОЙ (вручную посчитанной по дельте
        // позиции) скорости корпуса на forward. Так штраф не срабатывает при развороте на месте
        // (там продольная составляющая ≈ 0) и корректно учитывает фактическое перемещение,
        // а не сигналы гусениц напрямую.
        float forwardSpeed = Vector3.Dot(_lastVelocity, transform.forward);
        if (forwardSpeed < -backwardMovementDeadzone)
        {
            AddReward(forwardSpeed * backwardMovementPenalty); // forwardSpeed < 0 → отрицательная награда
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

        // к) Терминал: успешный захват
        if (gripper != null && gripper.isHolding)
        {
            AddReward(grabSuccessReward);
            EndEpisode();
            return;
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
            AddReward(outOfArenaPenalty);
            EndEpisode();
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