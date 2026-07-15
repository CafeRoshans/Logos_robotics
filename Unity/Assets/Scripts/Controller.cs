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
public class TrackController : MonoBehaviour
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
    public float maxAngularSpeedDeg = 200f; // подбери под реальный робот, если появятся данные

    [Header("Параметры PWM (эмуляция реального мотора)")]
    public float motorDeadzone = 10f;
    public float minMotorPwm = 35f;
    public float maxPwmStep = 15f;
    public float velocityToPwm = 200f;

    [Header("Управление")]
    [Tooltip("true — клавиатура пишет в leftInput/rightInput. " +
             "Выключай, когда роботом управляет ML-Agents (тогда вызывай SetTrackInputs напрямую).")]
    public bool useManualInput = true;

    // Главный интерфейс: именно эти 2 числа будет подавать нейронка
    [HideInInspector] public float leftInput = 0f;   // [-1..1]
    [HideInInspector] public float rightInput = 0f;  // [-1..1]

    Rigidbody rb;
    float trackWidth;               // расстояние между гусеницами, считается один раз
    float currentPwmLeft, currentPwmRight;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.isKinematic = true; // силы не нужны — двигаем вручную

        trackWidth = Vector3.Distance(leftTrackPoint.localPosition, rightTrackPoint.localPosition);
        if (trackWidth < 0.01f)
            Debug.LogWarning("TrackController: trackWidth почти 0 — проверь позиции leftTrackPoint/rightTrackPoint, повороты будут неадекватными.");
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
        // 1) Команда скорости гусеницы напрямую из входа
        float leftCmdSpeed = leftInput * maxLinearCmd;
        float rightCmdSpeed = rightInput * maxLinearCmd;

        // 2) Скорость → PWM (мёртвая зона, минимальный порог)
        float targetPwmLeft = SpeedToPwm(leftCmdSpeed);
        float targetPwmRight = SpeedToPwm(rightCmdSpeed);

        // 3) Плавный разгон/торможение PWM
        currentPwmLeft = StepToward(currentPwmLeft, targetPwmLeft, maxPwmStep);
        currentPwmRight = StepToward(currentPwmRight, targetPwmRight, maxPwmStep);

        // 4) PWM обратно → реальная скорость каждой гусеницы, м/с
        float effLeft = PwmToSpeed(currentPwmLeft) * maxLinearCmd;
        float effRight = PwmToSpeed(currentPwmRight) * maxLinearCmd;

        // 5) Точная кинематика дифференциального привода на основе реальной колеи
        float linearVelocity = (effLeft + effRight) * 0.5f;
        float angularVelocityRad = (effRight - effLeft) / trackWidth/100;
        float angularVelocityDeg = angularVelocityRad * Mathf.Rad2Deg;
        angularVelocityDeg = Mathf.Clamp(angularVelocityDeg, -maxAngularSpeedDeg, maxAngularSpeedDeg); // страховка

        // 6) Применение к Rigidbody без сил
        Vector3 newPos = rb.position + transform.forward * linearVelocity * Time.fixedDeltaTime;
        Quaternion deltaRot = Quaternion.Euler(0f, -angularVelocityDeg * Time.fixedDeltaTime, 0f);
        // минус — компенсация направления Y-поворота в Unity (см. предыдущий баг с W/E)

        rb.MovePosition(newPos);
        rb.MoveRotation(rb.rotation * deltaRot);
    }

    float SpeedToPwm(float speed)
    {
        float rawPwm = (speed / maxLinearCmd) * 100f; // нормируем к [-100..100]
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

    float StepToward(float current, float target, float maxStep)
    {
        float delta = target - current;
        if (Mathf.Abs(delta) <= maxStep) return target;
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