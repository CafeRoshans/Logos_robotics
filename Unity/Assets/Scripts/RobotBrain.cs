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
             "Осторожно: за 500 шагов эпизода может накопить distanceRewardScale × closeDistanceBonusMul × 250. " +
             "Держи в районе 0.1-0.3, иначе Mean Reward уйдёт в сотни.")]
    public float distanceRewardScale   = 0.2f;
    [Tooltip("Порог 'близко' — ниже него награда за сближение усиливается")]
    public float closeDistanceThreshold = 0.5f;
    [Tooltip("Множитель усиления награды за сближение вблизи мяча")]
    public float closeDistanceBonusMul  = 1.5f;
    [Tooltip("Штраф за резкое изменение управляющих сигналов между шагами")]
    public float actionRatePenalty      = 0.001f;
    [Tooltip("Штраф за критически близкую стену (по ИК/УЗ)")]
    public float wallProximityPenalty   = 0.02f;
    [Tooltip("Бонус за то, что мяч в центре кадра")]
    public float centeringBonus         = 0.005f;
    [Tooltip("Терминальный бонус за успешный захват мяча")]
    public float grabSuccessReward      = 5.0f;
    [Tooltip("Терминальный штраф за вылет за пределы арены")]
    public float outOfArenaPenalty      = -2.0f;
    [Tooltip("Небольшой штраф за каждый шаг — стимулирует скорость решения")]
    public float perStepPenalty         = -0.0005f;

    [Header("Случайный спавн робота и мяча")]
    [Tooltip("Спавнить робота и мяч в случайных точках из obstacleSpawner.unusedPoints при каждом эпизоде")]
    public bool randomizeSpawnPositions = true;
    [Tooltip("Минимальное расстояние между роботом и мячом при спавне (м). " +
             "Ставь >= размера мяча + захвата, чтобы робот не заспавнился на мяче.")]
    public float minRobotBallDistance = 0.6f;
    [Tooltip("Рандомизировать поворот робота при спавне (0..360°)")]
    public bool randomizeRobotHeading = true;

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

    public override void Initialize()
    {
        _rb = GetComponent<Rigidbody>();
        _startPosition = transform.position;
        _startRotation = transform.rotation;
        if (targetBall != null) _ballStartPosition = targetBall.position;
    }

    public override void OnEpisodeBegin()
    {
        // 1. Препятствия перераскладываем ПЕРВЫМИ — они определяют свободные точки
        //    для случайного спавна робота и мяча.
        if (obstacleSpawner != null) obstacleSpawner.Respawn();

        // 2. Разжимаем клешню (если был мяч) — до перемещения позиций
        if (gripper != null && gripper.isHolding) gripper.Release();
        if (gripper != null) gripper.grabCommand = false;

        // 3. Определяем позиции робота и мяча
        Vector3 robotPos = _startPosition;
        Quaternion robotRot = _startRotation;
        Vector3 ballPos = _ballStartPosition;

        if (randomizeSpawnPositions && obstacleSpawner != null
            && obstacleSpawner.unusedPoints.Count >= 2)
        {
            var pts = obstacleSpawner.unusedPoints;

            // Робот — случайная свободная точка
            int robotIdx = Random.Range(0, pts.Count);
            robotPos = pts[robotIdx].position;

            // Мяч — другая точка, с проверкой минимального расстояния до робота.
            // Даём до 20 попыток, потом берём просто «не такую же».
            int ballIdx = -1;
            for (int tries = 0; tries < 20; tries++)
            {
                int cand = Random.Range(0, pts.Count);
                if (cand == robotIdx) continue;
                if (Vector3.Distance(pts[cand].position, robotPos) < minRobotBallDistance) continue;
                ballIdx = cand;
                break;
            }
            if (ballIdx < 0) ballIdx = (robotIdx + 1) % pts.Count;
            ballPos = pts[ballIdx].position;

            if (randomizeRobotHeading)
                robotRot = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        }

        // 4. Ставим робота
        transform.SetPositionAndRotation(robotPos, robotRot);
        if (_rb != null && !_rb.isKinematic)
        {
            _rb.linearVelocity  = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }
        if (tracks != null) tracks.SetTrackInputs(0f, 0f);

        // 5. Ставим мяч
        if (targetBall != null)
        {
            targetBall.position = ballPos;
            var brb = targetBall.GetComponent<Rigidbody>();
            if (brb != null && !brb.isKinematic)
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

        // 5. Награды
        ComputeRewards(leftTrack, rightTrack);

        _prevLeft  = leftTrack;
        _prevRight = rightTrack;
    }

    void ComputeRewards(float leftTrack, float rightTrack)
    {
        float rewardBefore = GetCumulativeReward();

        // а) Сближение с мячом (delta distance)
        float curDist = DistanceToBall();
        if (_prevDistanceToBall > 0f && curDist > 0f)
        {
            float delta = _prevDistanceToBall - curDist; // + = приблизились
            delta = Mathf.Clamp(delta, -0.5f, 0.5f);
            float mul   = (curDist < closeDistanceThreshold) ? closeDistanceBonusMul : 1f;
            float rDist = delta * distanceRewardScale * mul;
            AddReward(rDist);

            // Диагностика: если distance-reward за шаг > 1, что-то не так
            if (Mathf.Abs(rDist) > 1f)
                Debug.LogWarning($"[RobotBrain] Странный distance-reward={rDist:F2} " +
                                 $"(delta={delta:F2}, curDist={curDist:F2}, prevDist={_prevDistanceToBall:F2})");
        }
        _prevDistanceToBall = curDist;

        // б) Штраф за резкость управления
        float dLeft  = Mathf.Abs(leftTrack  - _prevLeft);
        float dRight = Mathf.Abs(rightTrack - _prevRight);
        AddReward(-(dLeft + dRight) * actionRatePenalty);

        // в) Бонус за центрирование мяча в кадре
        if (yolo != null && yolo.isVisible)
        {
            float centered = 1f - Mathf.Abs(yolo.horizontalAngle); // 1 в центре, 0 на краю
            AddReward(centered * centeringBonus);
        }

        // г) Штраф за критически близкие стены
        if (sensors != null)
        {
            if (sensors.ultrasonicNormalized < 0.15f) AddReward(-wallProximityPenalty);
            if (sensors.leftIR  == 1)                 AddReward(-wallProximityPenalty);
            if (sensors.rightIR == 1)                 AddReward(-wallProximityPenalty);
        }

        // д) Мелкий постоянный штраф — не стоять
        AddReward(perStepPenalty);

        // Диагностика: аномалия суммарного вклада за шаг
        float stepDelta = GetCumulativeReward() - rewardBefore;
        if (Mathf.Abs(stepDelta) > 3f)
            Debug.LogWarning($"[RobotBrain] Аномальный вклад за шаг = {stepDelta:F2}. " +
                             $"Cumul={GetCumulativeReward():F2}, gripperIsHolding={(gripper != null && gripper.isHolding)}");

        // е) Терминал: успешный захват
        if (gripper != null && gripper.isHolding)
        {
            AddReward(grabSuccessReward);
            EndEpisode();
            return;
        }

        // ж) Терминал: вылет за арену.
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