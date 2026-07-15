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
    public TrackController tracks;
    public GripperController gripper;
    public VirtualSensors sensors;
    public SimulatedYoloCamera yolo;
    [Tooltip("Transform, вокруг Y которого крутится камера (сервопривод). Может быть родителем самой камеры.")]
    public Transform cameraServo;
    [Tooltip("Мяч, за которым робот охотится")]
    public Transform targetBall;

    [Header("Сервопривод камеры")]
    [Tooltip("Максимальный угол отклонения камеры ± (градусы)")]
    public float cameraServoMaxAngle = 90f;
    [Tooltip("Скорость поворота сервопривода (град/сек) при полном сигнале")]
    public float cameraServoSpeedDegPerSec = 90f;

    [Header("Границы арены (терминал 'вылетел')")]
    public Vector3 arenaCenter = Vector3.zero;
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
    [Tooltip("Бонус за то, что мяч в центре кадра")]
    public float centeringBonus         = 0.005f;
    [Tooltip("Терминальный бонус за успешный захват мяча")]
    public float grabSuccessReward      = 5.0f;
    [Tooltip("Терминальный штраф за вылет за пределы арены")]
    public float outOfArenaPenalty      = -2.0f;
    [Tooltip("Небольшой штраф за каждый шаг — стимулирует скорость решения")]
    public float perStepPenalty         = -0.0005f;

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

        // е) Терминал: успешный захват
        if (gripper != null && gripper.isHolding)
        {
            AddReward(grabSuccessReward);
            EndEpisode();
            return;
        }

        // ж) Терминал: вылет за арену
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
        Gizmos.color = Color.magenta;
        Gizmos.DrawWireCube(arenaCenter, arenaHalfSize * 2f);
    }
}