using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Кинематический контроллер гусеничного робота с дифференциальным приводом.
///
/// Move(gas, steering) — основной интерфейс для нейронки/ROS: [-1..1] каждый, доля от
/// РЕАЛЬНЫХ физических пределов робота (maxLinearCmd, maxAngularSpeedDeg). Внутри считается
/// честная инверсная кинематика по РЕАЛЬНОЙ колее (trackWidth) — а не произвольный
/// коэффициент подмешивания. Раньше здесь стоял turnK (константа 0.3), не связанная с
/// физической геометрией робота — leftTrackPoint/rightTrackPoint при этом были объявлены,
/// но фактически не влияли на повороты при ручном/сетевом управлении (только на прямую
/// кинематику в FixedUpdate). Теперь колея используется и на входе, и на выходе — оба
/// места считают поворот одной и той же физикой.
/// SetTrackInputs(left, right) — низкоуровневый прямой доступ к гусеницам [-1..1] каждая,
/// используется ручным тестом с клавиатуры и как результат работы Move().
/// Силы на Rigidbody НЕ прикладываются — движение целиком через MovePosition/MoveRotation.
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

    [Tooltip("Максимальная угловая скорость разворота, град/с. Понижена с 120° до 75° " +
             "по запросу — резкие развороты дестабилизировали физику при столкновениях.")]
    public float maxAngularSpeedDeg = 75f;

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
        // Захардкожено в реально измеренное значение реального GFS-X (было: авто-расчёт по
        // трансформам сцены, из ветки feat/making-rl-work — "теперь мы знаем как будет
        // происходить в реальности"). Старая строка оставлена закомментированной для сравнения.
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
    /// Основной интерфейс для нейронки/ROS: газ (вперёд/назад) + руль (влево/вправо),
    /// [-1..1] каждый — доля от РЕАЛЬНЫХ физических пределов ЭТОГО эпизода (maxLinearCmd,
    /// maxAngularSpeedDeg, которые могут быть рандомизированы доменной рандомизацией).
    /// Внутри — точная инверсная кинематика дифференциального привода по РЕАЛЬНОЙ колее
    /// (trackWidth), обратная той же формуле, что считает прямую кинематику в FixedUpdate:
    ///   angularVelocityRad = (effRight - effLeft) / trackWidth
    ///   linearVelocity     = (effLeft + effRight) / 2
    /// Если требуемая скорость гусеницы превышает физический потолок мотора — ОБЕ гусеницы
    /// пропорционально уменьшаются (не клипаются по отдельности), сохраняя заданное
    /// соотношение gas/steering максимально близко к тому, что просила сеть/ROS.
    /// Совместимо с ROS /cmd_vel (Twist): linear.x = gas × maxLinearCmd, angular.z = steering × maxAngularSpeedDeg.
    /// </summary>
    public void Move(float gas, float steering)
    {
        gas      = Mathf.Clamp(gas,      -1f, 1f);
        steering = Mathf.Clamp(steering, -1f, 1f);

        float desiredLinear     = gas * maxLinearCmd;                          // м/с
        float desiredAngularRad = steering * maxAngularSpeedDeg * Mathf.Deg2Rad; // рад/с

        float vLeft  = desiredLinear - desiredAngularRad * trackWidth * 0.5f;
        float vRight = desiredLinear + desiredAngularRad * trackWidth * 0.5f;

        float left  = maxLinearCmd > 0.0001f ? vLeft  / maxLinearCmd : 0f;
        float right = maxLinearCmd > 0.0001f ? vRight / maxLinearCmd : 0f;

        // Пропорциональное уменьшение при выходе за физический предел мотора —
        // сохраняет заданное соотношение gas/steering вместо искажающего клипа по отдельности.
        float maxMag = Mathf.Max(Mathf.Abs(left), Mathf.Abs(right), 1f);
        left  /= maxMag;
        right /= maxMag;

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

        // Гасим любую скорость/угловую скорость, накопленную физическим движком между
        // кадрами (например, от импульса столкновения со стеной) — мы двигаем робота
        // ПОЛНОСТЬЮ через MovePosition/MoveRotation, а не через физическую интеграцию.
        // Без этого не-кинематический Rigidbody может резко "выстрелить" после удара:
        // столкновение даёт скачок rb.linearVelocity/angularVelocity, который потом
        // складывается с нашим собственным MovePosition в следующих кадрах, разгоняя
        // робота неконтролируемо (вплоть до вылета за границы арены — именно это,
        // похоже, и обрывало эпизод, а не сам штраф за столкновение). Столкновения
        // (OnCollisionEnter) при этом продолжают честно обнаруживаться — это зависит
        // от факта контакта коллайдеров в течение кадра, а не от скорости после него.
        rb.linearVelocity  = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

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