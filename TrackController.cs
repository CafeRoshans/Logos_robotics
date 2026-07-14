using UnityEngine;

/// <summary>
/// Контроллер гусеничного робота с дифференциальным приводом.
/// Принимает команды gas [-1..1] и steer [-1..1],
/// переводит в PWM с учётом мёртвой зоны и применяет к Rigidbody.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class TrackController : MonoBehaviour
{
    [Header("Скорости")]
    [Tooltip("Базовая линейная скорость (м/с)")]
    public float moveSpeed = 0.57f;

    [Tooltip("Базовая скорость поворота (град/с)")]
    public float turnSpeed = 120f;

    [Tooltip("Коэффициент влияния поворота на скорость гусениц")]
    public float turnK = 0.30f;

    [Tooltip("Лимит поступательной скорости (м/с)")]
    public float maxLinearCmd = 0.25f;

    [Header("Параметры PWM")]
    [Tooltip("Мёртвая зона мотора (% PWM) — ниже этого значения мотор стоит")]
    public float motorDeadzone = 10f;

    [Tooltip("Минимальный стартовый порог PWM для трогания")]
    public float minMotorPwm = 35f;

    [Tooltip("Максимальное изменение PWM за один FixedUpdate (плавность разгона)")]
    public float maxPwmStep = 15f;

    [Tooltip("Коэффициент перевода м/с → PWM (подбирается под диаметр колеса/редуктор)")]
    public float velocityToPwm = 200f;

    // Текущие значения PWM для сглаживания
    private float _currentPwmLeft  = 0f;
    private float _currentPwmRight = 0f;

    private Rigidbody _rb;

    // Входные команды (задаются извне: вручную или из MLAgents)
    [HideInInspector] public float gas   = 0f;  // [-1..1]
    [HideInInspector] public float steer = 0f;  // [-1..1]

    void Awake()
    {
        _rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        // Ручное управление с клавиатуры (для теста без ML-Agents)
        gas   = Input.GetAxis("Vertical");
        steer = Input.GetAxis("Horizontal");
    }

    void FixedUpdate()
    {
        // 1. Смешивание скоростей — дифференциальный привод
        float linearCmd = Mathf.Clamp(gas * maxLinearCmd, -maxLinearCmd, maxLinearCmd);
        float turnCmd   = steer * turnK;

        float leftSpeed  = linearCmd + turnCmd;   // м/с левый борт
        float rightSpeed = linearCmd - turnCmd;   // м/с правый борт

        // 2. Перевод физической скорости → целевой PWM
        float targetPwmLeft  = SpeedToPwm(leftSpeed);
        float targetPwmRight = SpeedToPwm(rightSpeed);

        // 3. Сглаживание разгона (ramp)
        _currentPwmLeft  = StepToward(_currentPwmLeft,  targetPwmLeft,  maxPwmStep);
        _currentPwmRight = StepToward(_currentPwmRight, targetPwmRight, maxPwmStep);

        // 4. Перевод PWM обратно → физическая скорость для Unity
        float effectiveLeft  = PwmToSpeed(_currentPwmLeft);
        float effectiveRight = PwmToSpeed(_currentPwmRight);

        float linearVelocity  = (effectiveLeft + effectiveRight) * 0.5f * moveSpeed;
        float angularVelocity = (effectiveRight - effectiveLeft) / moveSpeed * turnSpeed;

        // 5. Применение к Rigidbody
        Vector3 newPosition = _rb.position + transform.forward * linearVelocity * Time.fixedDeltaTime;
        _rb.MovePosition(newPosition);

        Quaternion deltaRotation = Quaternion.Euler(0f, angularVelocity * Time.fixedDeltaTime, 0f);
        _rb.MoveRotation(_rb.rotation * deltaRotation);
    }

    /// <summary>
    /// Переводит скорость (м/с) в PWM [-100..100] с учётом мёртвой зоны.
    /// </summary>
    float SpeedToPwm(float speed)
    {
        float rawPwm = speed * velocityToPwm;  // масштаб м/с → PWM

        if (Mathf.Abs(rawPwm) < motorDeadzone)
            return 0f;  // мёртвая зона — мотор стоит

        // Применяем минимальный стартовый порог
        float sign = Mathf.Sign(rawPwm);
        float pwm  = Mathf.Abs(rawPwm);
        pwm = Mathf.Max(pwm, minMotorPwm);
        pwm = Mathf.Min(pwm, 100f);

        return sign * pwm;
    }

    /// <summary>
    /// Переводит PWM обратно в нормализованную скорость [-1..1].
    /// </summary>
    float PwmToSpeed(float pwm)
    {
        if (Mathf.Abs(pwm) < motorDeadzone)
            return 0f;

        return pwm / 100f;
    }

    /// <summary>
    /// Плавный шаг от current к target с ограничением maxStep.
    /// </summary>
    float StepToward(float current, float target, float maxStep)
    {
        float delta = target - current;
        if (Mathf.Abs(delta) <= maxStep)
            return target;
        return current + Mathf.Sign(delta) * maxStep;
    }

    // Отладочный Gizmo — направление движения
    void OnDrawGizmos()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawRay(transform.position, transform.forward * 0.5f);
    }
}
