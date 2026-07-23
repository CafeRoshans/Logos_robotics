using UnityEngine;

/// <summary>
/// Симуляция бортовой YOLO-камеры. Вместо рендеринга и детектора
/// используется геометрическая проекция 3D-позиции мяча в 2D через
/// Camera.WorldToViewportPoint + проверка ограничений (FOV, дальность, отсутствие препятствий).
/// </summary>
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("Ссылки")]
    [Tooltip("Камера робота. Если не задана — берётся с этого же GameObject.")]
    public Camera cameraComponent;

    [Tooltip("Целевой мяч (Transform)")]
    public Transform targetBall;

    [Header("Параметры видимости")]
    [Tooltip("Горизонтальный угол обзора камеры (градусы)")]
    public float horizontalFovDegrees = 40f;

    [Tooltip("Вертикальный угол обзора камеры (градусы) — для расчёта verticalAngle. " +
             "Отдельно от horizontalFovDegrees и от Camera.fieldOfView намеренно: используем " +
             "свою геометрию (SignedAngle), а не встроенный vp.y — именно vp.y (завязанный на " +
             "настройки самой Camera, не на этот скрипт) был причиной прошлого бага 'мяч рядом, " +
             "а камера не видит', когда физический наклон Camera не совпадал с ожиданиями.")]
    public float verticalFovDegrees = 40f;

    [Tooltip("Максимальная дальность детекции мяча (м) — ПОДТВЕРЖДЕНО на практике (2026-07-22): " +
             "реальный YOLO надёжно видит мяч до 2.5м, дальше уже не в счёт. Обычный шум " +
             "(visionDistanceNoise/visionAngleNoise) на всей дистанции до этого предела достаточен, " +
             "отдельно масштабировать его по дистанции не нужно.")]
    public float maxRange = 2.5f;

    [Tooltip("Слои для проверки препятствий (стены и т.п.)")]
    public LayerMask obstacleMask = ~0;

    [Tooltip("Тег мяча — если Raycast упёрся в него, считаем не перекрытым")]
    public string targetBallTag = "TargetBall";

    [Header("Синтетическая confidence (аналог packet.conf с реального YOLO)")]
    [Tooltip("На каком нормализованном расстоянии (0..1 от maxRange) confidence падает до " +
             "минимума. 1.0 = падает линейно вплоть до самого maxRange. Меньше = детектор " +
             "'уверен' только на действительно близких дистанциях.")]
    [Range(0.1f, 1f)]
    public float confidenceFalloff = 0.8f;
    [Tooltip("Случайный джиттер confidence кадр-к-кадру (реальная детекция не идеально стабильна " +
             "даже на постоянной дистанции — блики, шум сенсора и т.п.)")]
    [Range(0f, 0.3f)]
    public float confidenceJitter = 0.08f;

    // Выходные показания (читает RobotBrain)
    [HideInInspector] public bool  isVisible = false;
    [HideInInspector] public float horizontalAngle = 0f;    // -1..1  (лево..право относительно центра кадра)
    [HideInInspector] public float verticalAngle   = 0f;    // -1..1  (верх..низ относительно центра кадра)
    [HideInInspector] public float normalizedDistance = 1f; //  0..1  (вплотную..на пределе видимости)
    [HideInInspector] public float rawDistance = -1f;       // м, -1 если не виден
    [HideInInspector] public float confidence = 0f;         //  0..1  (аналог packet.conf реального YOLO)

    [Header("Отладка")]
    public bool drawGizmos = true;

    void Awake()
    {
        if (cameraComponent == null)
            cameraComponent = GetComponent<Camera>();
    }

    void Update()
    {
        isVisible          = false;
        horizontalAngle    = 0f;
        verticalAngle      = 0f;
        normalizedDistance = 1f;
        rawDistance        = -1f;
        confidence         = 0f;

        if (cameraComponent == null || targetBall == null) return;

        Vector3 toBall = targetBall.position - cameraComponent.transform.position;

        // Мяч должен быть впереди камеры (не сзади) — проверяем через знак проекции
        // на forward, не через Camera.WorldToViewportPoint (тот завязан на настройки
        // самой Camera и был источником прошлого бага — камера не видела мяч рядом
        // просто потому, что Camera.fieldOfView/наклон не совпадали с тем, что тут настроено).
        if (Vector3.Dot(toBall, cameraComponent.transform.forward) <= 0f) return; // мяч за камерой

        // 1. Горизонтальный угол через геометрию — не зависит от aspect ratio/FOV Camera
        Vector3 flatToBallH = Vector3.ProjectOnPlane(toBall, cameraComponent.transform.up);
        float angleH = Vector3.SignedAngle(cameraComponent.transform.forward,
                                            flatToBallH,
                                            cameraComponent.transform.up);
        if (Mathf.Abs(angleH) > horizontalFovDegrees * 0.5f) return;

        // 2. Вертикальный угол — та же логика, но плоскость перпендикулярна right,
        // ось поворота — right. Положительный angleV = мяч НИЖЕ центра кадра (см. пояснение
        // ниже про знак и его использование в CameraAimController).
        Vector3 flatToBallV = Vector3.ProjectOnPlane(toBall, cameraComponent.transform.right);
        float angleV = Vector3.SignedAngle(cameraComponent.transform.forward,
                                            flatToBallV,
                                            cameraComponent.transform.right);
        if (Mathf.Abs(angleV) > verticalFovDegrees * 0.5f) return;

        // 3. Дальность
        float dist = toBall.magnitude;
        if (dist > maxRange) return;

        // 4. Проверка перекрытия стеной
        Vector3 dir = toBall.normalized;
        if (Physics.Raycast(cameraComponent.transform.position, dir, out RaycastHit hit,
                            dist + 0.05f, obstacleMask, QueryTriggerInteraction.Ignore))
        {
            if (string.IsNullOrEmpty(targetBallTag) || !hit.collider.CompareTag(targetBallTag))
                return; // луч уперся в стену раньше мяча
        }

        // Всё ок — мяч виден
        isVisible          = true;
        horizontalAngle    = Mathf.Clamp(angleH / (horizontalFovDegrees * 0.5f), -1f, 1f);
        verticalAngle      = Mathf.Clamp(angleV / (verticalFovDegrees   * 0.5f), -1f, 1f);
        rawDistance        = dist;
        normalizedDistance = Mathf.Clamp01(dist / maxRange);

        // Синтетический аналог packet.conf: падает с дистанцией (дальше — меньше пикселей на
        // мяче, ниже уверенность реального детектора) + случайный джиттер кадр-к-кадру.
        float distFrac = Mathf.Clamp01(normalizedDistance / Mathf.Max(0.01f, confidenceFalloff));
        confidence = Mathf.Clamp01(1f - distFrac + Random.Range(-confidenceJitter, confidenceJitter));
    }

    void OnDrawGizmos()
    {
        if (!drawGizmos || cameraComponent == null) return;

        Gizmos.color = isVisible ? Color.green : new Color(1f, 1f, 1f, 0.3f);
        Vector3 origin = cameraComponent.transform.position;
        Quaternion leftRot  = Quaternion.AngleAxis(-horizontalFovDegrees * 0.5f, cameraComponent.transform.up);
        Quaternion rightRot = Quaternion.AngleAxis( horizontalFovDegrees * 0.5f, cameraComponent.transform.up);
        Gizmos.DrawRay(origin, leftRot  * cameraComponent.transform.forward * maxRange);
        Gizmos.DrawRay(origin, rightRot * cameraComponent.transform.forward * maxRange);
        Gizmos.DrawRay(origin, cameraComponent.transform.forward * maxRange);
    }
}
