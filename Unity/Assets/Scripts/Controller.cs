using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Кинематический контроллер гусеничного робота с дифференциальным приводом.
/// Управление: 2 параметра скорости гусениц [-1..1] (leftInput/rightInput) —
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class Controller : MonoBehaviour
{
    [Header("Клавиши (ручной тест)")]
    public Key leftForwardKey = Key.W;
    public Key leftBackwardKey = Key.S;
    public Key rightForwardKey = Key.E;
    public Key rightBackwardKey = Key.D;

    [Header("Точки гусениц — задают реальную колею робота")]
    [Tooltip("Дочерние объекты, размещённые в позициях левой/правой гусеницы")]
    public Transform leftTrackPoint;
    public Transform rightTrackPoint;

    [Header("Скорость (дефолты как в референс-репе для реального GFS-X)")]
    [Tooltip("Полная линейная скорость робота при input=1 (м/с). Реальный робот ~0.57 м/с, но " +
             "для стабильности обучения PPO ограничиваем до 0.25. При input=1 → скорость = maxLinearCmd.")]
    public float maxLinearCmd = 0.25f;

    [Tooltip("Максимальная угловая скорость разворота, град/с. 120°/сек как у реального GFS-X.")]
    public float maxAngularSpeedDeg = 120f;

    [Header("Дифференциальный привод — gas/steering декомпозиция")]
    [Tooltip("Коэффициент смешивания руля со скоростью гусениц. При Move(gas=1, steer=1): " +
             "leftInput = 1 + 1×turnK, rightInput = 1 - 1×turnK. " +
             "0.3 — умеренный поворот, гусеницы разной скорости но обе едут вперёд. " +
             "1.0 — жёсткий поворот, одна гусеница стоит. " +
             "Влияет только на Move(gas, steering); SetTrackInputs напрямую не задействует.")]
    [Range(0.1f, 1.5f)]
    public float turnK = 0.30f;

    [Header("Параметры PWM (эмуляция реального мотора)")]
    public float motorDeadzone = 10f;
    public float minMotorPwm = 35f;
    public float maxPwmStep = 15f;
    public float velocityToPwm = 200f;

    [Header("Управление")]
    [Tooltip("true — клавиатура пишет в leftInput/rightInput. " +
             "Выключай, когда роботом управляет ML-Agents (тогда вызывай SetTrackInputs напрямую).")]
    public bool useManualInput = true;

    [HideInInspector] public float leftInput = 0f;
    [HideInInspector] public float rightInput = 0f;

    // Асимметричные множители моторов — реалистичный износ редукторов.
    // Задаются извне (RobotBrain.OnEpisodeBegin) для доменной рандомизации.
    // 1.0 = мотор работает по-номиналу, 0.9 = чуть слабее, 1.1 = чуть сильнее.
    [HideInInspector] public float leftSpeedMul  = 1f;
    [HideInInspector] public float rightSpeedMul = 1f;

    Rigidbody rb;
    float trackWidth;
    float currentPwmLeft, currentPwmRight;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        if (leftTrackPoint == null || rightTrackPoint == null)
        {
            Debug.LogError("Controller: leftTrackPoint или rightTrackPoint не назначены в инспекторе!");
            return;
        }

        // trackWidth = Vector3.Distance(leftTrackPoint.localPosition, rightTrackPoint.localPosition);
        trackWidth = 0.18f;
        Debug.Log($"Controller: trackWidth = {trackWidth:F3} m (расстояние между точками гусениц).");
        if (trackWidth < 0.01f)
            Debug.LogWarning("Controller: trackWidth почти 0 — проверь позиции leftTrackPoint/rightTrackPoint, повороты будут неадекватными.");
    }

    public void SetTrackInputs(float left, float right)
    {
        leftInput = Mathf.Clamp(left, -1f, 1f);
        rightInput = Mathf.Clamp(right, -1f, 1f);
    }

    /// <summary>
    /// Основной интерфейс для нейронки: газ (вперёд/назад) + руль (влево/вправо).
    /// Внутри раскладывается в leftInput/rightInput по формуле дифференциального привода.
    /// </summary>
    public void Move(float gas, float steering)
    {
        gas      = Mathf.Clamp(gas,      -1f, 1f);
        steering = Mathf.Clamp(steering, -1f, 1f);

        float left  = gas + steering * turnK;
        float right = gas - steering * turnK;

        SetTrackInputs(left, right);
    }

    void Update()
    {
        if (!useManualInput) return;

        var kb = Keyboard.current;
        if (kb == null) return;

        // WASD как gas + steer: W/S — газ, A/D — руль. Именно так работает Heuristic
        // ML-Agents в новом Move(gas, steering) режиме.
        float gas   = kb.wKey.ReadValue() - kb.sKey.ReadValue();
        float steer = kb.dKey.ReadValue() - kb.aKey.ReadValue();
        Move(gas, steer);
    }

    void FixedUpdate()
    {
        if (trackWidth < 0.01f) return; // защита от NaN, если точки не назначены

        float leftCmdSpeed = leftInput * maxLinearCmd;
        float rightCmdSpeed = rightInput * maxLinearCmd;

        float targetPwmLeft = SpeedToPwm(leftCmdSpeed);
        float targetPwmRight = SpeedToPwm(rightCmdSpeed);

        currentPwmLeft = StepToward(currentPwmLeft, targetPwmLeft, maxPwmStep, minMotorPwm);
        currentPwmRight = StepToward(currentPwmRight, targetPwmRight, maxPwmStep, minMotorPwm);

        // Применяем per-side мультипликаторы — эмулируют разное состояние редукторов
        // (левый борт может ехать чуть быстрее правого и наоборот).
        float effLeft  = PwmToSpeed(currentPwmLeft)  * maxLinearCmd * leftSpeedMul;
        float effRight = PwmToSpeed(currentPwmRight) * maxLinearCmd * rightSpeedMul;

        float linearVelocity = (effLeft + effRight) * 0.5f;
        float angularVelocityRad = (effRight - effLeft) / trackWidth;
        float angularVelocityDeg = angularVelocityRad * Mathf.Rad2Deg;
        angularVelocityDeg = Mathf.Clamp(angularVelocityDeg, -maxAngularSpeedDeg, maxAngularSpeedDeg);

        Vector3 newPos = rb.position + transform.forward * linearVelocity * Time.fixedDeltaTime;
        Quaternion deltaRot = Quaternion.Euler(0f, -angularVelocityDeg * Time.fixedDeltaTime, 0f);

        rb.MovePosition(newPos);
        rb.MoveRotation(rb.rotation * deltaRot);
    }

    float SpeedToPwm(float speed)
    {
        float rawPwm = (speed / maxLinearCmd) * 100f;
        if (Mathf.Abs(rawPwm) < motorDeadzone) return 0f;

        float sign = Mathf.Sign(rawPwm);
        float pwm = Mathf.Clamp(Mathf.Abs(rawPwm), minMotorPwm, 100f);
        return sign * pwm;
    }

    float PwmToSpeed(float pwm)
    {
        if (Mathf.Abs(pwm) < motorDeadzone) return 0f;
        return pwm / 100f;
    }

    float StepToward(float current, float target, float maxStep, float minMotorThreshold)
    {
        bool wasStopped = Mathf.Abs(current) < minMotorThreshold;
        bool wantsToMove = Mathf.Abs(target) >= minMotorThreshold;

        if (wasStopped && wantsToMove)
            current = Mathf.Sign(target) * minMotorThreshold;

        float delta = target - current;
        if (Mathf.Abs(delta) <= maxStep)
            return target;
        return current + Mathf.Sign(delta) * maxStep;
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawRay(transform.position, transform.forward * 0.5f);

        if (leftTrackPoint != null && rightTrackPoint != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(leftTrackPoint.position, rightTrackPoint.position);
        }
    }
}