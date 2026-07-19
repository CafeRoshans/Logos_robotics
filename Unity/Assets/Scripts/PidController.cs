using UnityEngine;

/// <summary>
/// Простой переиспользуемый PID-регулятор. По умолчанию Ki = Kd = 0 — работает как чистый
/// P-регулятор: для задачи "держать мяч в центре кадра" пропорциональной составляющей обычно
/// достаточно, а I/D можно включить отдельно, не переписывая код, который его использует.
/// Не MonoBehaviour — обычный C#-класс, создаётся через `new PidController(...)` внутри
/// любого компонента (см. CameraAimController).
/// </summary>
[System.Serializable]
public class PidController
{
    public float Kp;
    public float Ki;
    public float Kd;

    [Tooltip("Ограничение накопленного интеграла — защита от windup при долгой ошибке одного " +
             "знака (например, мяч упирается в физический предел серво и центр недостижим).")]
    public float integralClamp = 1f;

    float _integral;
    float _prevError;
    bool  _hasPrevError;

    public PidController(float kp, float ki = 0f, float kd = 0f, float integralClampValue = 1f)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        integralClamp = integralClampValue;
        Reset();
    }

    /// <summary>Сбрасывает интеграл и производную — вызывай при смене режима/цели, иначе
    /// регулятор "помнит" накопленную ошибку из прошлого контекста.</summary>
    public void Reset()
    {
        _integral = 0f;
        _prevError = 0f;
        _hasPrevError = false;
    }

    /// <summary>
    /// Считает управляющий сигнал по ошибке `error` за прошедшее время `dt`.
    /// Возвращаемое значение НЕ ограничено диапазоном — вызывающий код сам решает,
    /// как его применить (например, только знак — для направления дискретного шага).
    /// </summary>
    public float Update(float error, float dt)
    {
        dt = Mathf.Max(dt, 0.0001f);

        _integral += error * dt;
        _integral = Mathf.Clamp(_integral, -integralClamp, integralClamp);

        float derivative = _hasPrevError ? (error - _prevError) / dt : 0f;
        _prevError = error;
        _hasPrevError = true;

        return Kp * error + Ki * _integral + Kd * derivative;
    }
}
