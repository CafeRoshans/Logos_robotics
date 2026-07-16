using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Кинематический контроллер гусеничного робота с дифференциальным приводом.
/// Управление: 2 параметра скорости гусениц [-1..1] (leftInput/rightInput) —
/// именно этот интерфейс ожидает нейронка / ML-Agents brain.
/// Повороты считаются точной кинематической формулой на основе реального
/// расстояния между гусеницами (trackWidth), а не подобранной вручную константой.
/// Силы на Rigidbody НЕ прикладываются — движение целиком через MovePosition/MoveRotation.
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

    [Header("Скорость")]
    public float maxLinearCmd = 0.8f;

    [Tooltip("Реалистичный потолок скорости разворота, град/с. " +
            "Не даёт роботу улетать в нереалистичное вращение даже при ошибке в геометрии.")]
    public float maxAngularSpeedDeg = 200f;

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

        trackWidth = Vector3.Distance(leftTrackPoint.localPosition, rightTrackPoint.localPosition);
        if (trackWidth < 0.01f)
            Debug.LogWarning("Controller: trackWidth почти 0 — проверь позиции leftTrackPoint/rightTrackPoint, повороты будут неадекватными.");
    }

    public void SetTrackInputs(float left, float right)
    {
        leftInput = Mathf.Clamp(left, -1f, 1f);
        rightInput = Mathf.Clamp(right, -1f, 1f);
    }

    void Update()
    {
        if (!useManualInput) return;

        var kb = Keyboard.current;
        if (kb == null) return;

        float left = 0f;
        if (kb[leftForwardKey].isPressed) left += 1f;
        if (kb[leftBackwardKey].isPressed) left -= 1f;

        float right = 0f;
        if (kb[rightForwardKey].isPressed) right += 1f;
        if (kb[rightBackwardKey].isPressed) right -= 1f;

        SetTrackInputs(left, right);
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

        float effLeft = PwmToSpeed(currentPwmLeft) * maxLinearCmd;
        float effRight = PwmToSpeed(currentPwmRight) * maxLinearCmd;

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