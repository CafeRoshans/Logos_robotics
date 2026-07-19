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

    [Tooltip("Максимальная дальность детекции мяча (м)")]
    public float maxRange = 2.0f;

    [Tooltip("Слои для проверки препятствий (стены и т.п.)")]
    public LayerMask obstacleMask = ~0;

    [Tooltip("Тег мяча — если Raycast упёрся в него, считаем не перекрытым")]
    public string targetBallTag = "TargetBall";

    // Выходные показания (читает RobotBrain)
    [HideInInspector] public bool  isVisible = false;
    [HideInInspector] public float horizontalAngle = 0f;    // -1..1  (лево..право относительно центра кадра)
    [HideInInspector] public float normalizedDistance = 1f; //  0..1  (вплотную..на пределе видимости)
    [HideInInspector] public float rawDistance = -1f;       // м, -1 если не виден

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
        normalizedDistance = 1f;
        rawDistance        = -1f;

        if (cameraComponent == null || targetBall == null) return;

        // 1. Проекция в viewport
        Vector3 vp = cameraComponent.WorldToViewportPoint(targetBall.position);
        if (vp.z <= 0f) return;                          // мяч за камерой
        if (vp.x < 0f || vp.x > 1f) return;              // за границей кадра по X
        if (vp.y < 0f || vp.y > 1f) return;              // за границей кадра по Y

        // 2. Горизонтальный угол через геометрию — не зависит от aspect ratio
        Vector3 toBall     = targetBall.position - cameraComponent.transform.position;
        Vector3 flatToBall = Vector3.ProjectOnPlane(toBall, cameraComponent.transform.up);
        float angleFromForward = Vector3.SignedAngle(cameraComponent.transform.forward,
                                                     flatToBall,
                                                     cameraComponent.transform.up);
        if (Mathf.Abs(angleFromForward) > horizontalFovDegrees * 0.5f) return;

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
        horizontalAngle    = Mathf.Clamp(angleFromForward / (horizontalFovDegrees * 0.5f), -1f, 1f);
        rawDistance        = dist;
        normalizedDistance = Mathf.Clamp01(dist / maxRange);
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