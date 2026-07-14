using UnityEngine;

/// <summary>
/// Виртуальные датчики робота: ультразвуковой (конусный), ИК-датчики препятствий по бокам
/// и ИК-датчик наличия мяча в клешне.
/// Точки-якоря (Empty GameObjects) задают позицию и направление луча:
///   forward каждого якоря — направление сканирования.
/// </summary>
public class VirtualSensors : MonoBehaviour
{
    [Header("Точки-якоря датчиков")]
    [Tooltip("УЗ-датчик: смотрит вперёд от центра робота")]
    public Transform centerPoint;

    [Tooltip("ИК препятствия слева")]
    public Transform leftIRPoint;

    [Tooltip("ИК препятствия справа")]
    public Transform rightIRPoint;

    [Tooltip("ИК внутри клешни (смотрит между губок)")]
    public Transform gripperIRPoint;

    [Header("Ультразвуковой датчик (US)")]
    [Tooltip("Максимальная дальность УЗ (м)")]
    public float usMaxRange = 2.0f;

    [Tooltip("Половина угла раскрытия конуса УЗ (градусы). Полный конус ~30° → 15°")]
    public float usHalfConeAngle = 15f;

    [Tooltip("Кол-во лучей вдоль ОДНОЙ оси веера (итого лучей = usRaysPerAxis^2)")]
    [Range(1, 9)]
    public int usRaysPerAxis = 5;

    [Tooltip("Тег мяча — УЗ его игнорирует (слишком мал для отражения)")]
    public string targetBallTag = "TargetBall";

    [Header("ИК-датчики препятствий (боковые)")]
    [Tooltip("Максимальная дальность ИК препятствия (м) — около 15 см")]
    public float irObstacleRange = 0.15f;

    [Header("ИК-датчик клешни")]
    [Tooltip("Максимальная дальность ИК клешни (м) — 7-8 см")]
    public float gripperIRRange = 0.08f;

    [Header("Слои")]
    [Tooltip("Слои, по которым сканируют УЗ и ИК препятствий (стены, объекты сцены)")]
    public LayerMask obstacleMask = ~0;

    [Tooltip("Слои, по которым сканирует датчик клешни (обычно = всё, ищем по тегу)")]
    public LayerMask gripperScanMask = ~0;

    [Header("Отладка")]
    public bool drawGizmos = true;

    // Текущие показания датчиков (доступны извне для ML-агента / логики)
    [HideInInspector] public float ultrasonicNormalized = 1f; // 0 = вплотную, 1 = чисто
    [HideInInspector] public float ultrasonicDistance   = -1f; // сырое значение (м), -1 если нет попадания
    [HideInInspector] public int   leftIR    = 0; // 1 = стена рядом, 0 = свободно
    [HideInInspector] public int   rightIR   = 0;
    [HideInInspector] public int   gripperIR = 0; // 1 = мяч в клешне
    [HideInInspector] public GameObject gripperIRHitObject = null; // ссылка на замеченный мяч

    void Update()
    {
        ReadUltrasonic();
        leftIR    = ReadObstacleIR(leftIRPoint);
        rightIR   = ReadObstacleIR(rightIRPoint);
        gripperIR = ReadGripperIR();
    }

    /// <summary>
    /// УЗ-датчик: пускает веер лучей внутри конуса, берёт минимальную дистанцию
    /// среди попаданий, игнорируя мяч.
    /// </summary>
    void ReadUltrasonic()
    {
        if (centerPoint == null)
        {
            ultrasonicDistance   = -1f;
            ultrasonicNormalized = 1f;
            return;
        }

        float minDist = usMaxRange;
        bool  anyHit  = false;

        // Веер лучей: сетка углов [-half..+half] по yaw и pitch
        int rays = Mathf.Max(1, usRaysPerAxis);
        float step = (rays > 1) ? (2f * usHalfConeAngle) / (rays - 1) : 0f;

        Vector3 origin = centerPoint.position;

        for (int i = 0; i < rays; i++)
        {
            float yaw = (rays == 1) ? 0f : -usHalfConeAngle + step * i;
            for (int j = 0; j < rays; j++)
            {
                float pitch = (rays == 1) ? 0f : -usHalfConeAngle + step * j;

                // Направление: forward якоря, повёрнутое на (yaw, pitch)
                Quaternion rot = centerPoint.rotation * Quaternion.Euler(pitch, yaw, 0f);
                Vector3 dir = rot * Vector3.forward;

                if (Physics.Raycast(origin, dir, out RaycastHit hit, usMaxRange, obstacleMask, QueryTriggerInteraction.Ignore))
                {
                    // УЗ не видит мяч — пропускаем
                    if (!string.IsNullOrEmpty(targetBallTag) && hit.collider.CompareTag(targetBallTag))
                        continue;

                    if (hit.distance < minDist)
                    {
                        minDist = hit.distance;
                        anyHit  = true;
                    }
                }
            }
        }

        if (anyHit)
        {
            ultrasonicDistance   = minDist;
            ultrasonicNormalized = Mathf.Clamp01(minDist / usMaxRange);
        }
        else
        {
            ultrasonicDistance   = -1f;
            ultrasonicNormalized = 1f; // чисто
        }
    }

    /// <summary>
    /// Одиночный ИК-луч на короткую дистанцию. 1 = стена, 0 = свободно.
    /// Мяч игнорируется (боковые ИК не должны реагировать на него как на препятствие).
    /// </summary>
    int ReadObstacleIR(Transform anchor)
    {
        if (anchor == null) return 0;

        if (Physics.Raycast(anchor.position, anchor.forward, out RaycastHit hit, irObstacleRange, obstacleMask, QueryTriggerInteraction.Ignore))
        {
            if (!string.IsNullOrEmpty(targetBallTag) && hit.collider.CompareTag(targetBallTag))
                return 0;
            return 1;
        }
        return 0;
    }

    /// <summary>
    /// ИК клешни: 1 если между губок есть объект с тегом TargetBall в пределах gripperIRRange.
    /// </summary>
    int ReadGripperIR()
    {
        gripperIRHitObject = null;
        if (gripperIRPoint == null) return 0;

        // Луч
        if (Physics.Raycast(gripperIRPoint.position, gripperIRPoint.forward, out RaycastHit hit, gripperIRRange, gripperScanMask, QueryTriggerInteraction.Collide))
        {
            if (!string.IsNullOrEmpty(targetBallTag) && hit.collider.CompareTag(targetBallTag))
            {
                gripperIRHitObject = hit.collider.gameObject;
                return 1;
            }
        }

        // Дублирующая сфера — страхуем, если луч промахнулся мимо круглого мяча
        Collider[] sphereHits = Physics.OverlapSphere(gripperIRPoint.position, gripperIRRange, gripperScanMask, QueryTriggerInteraction.Collide);
        foreach (var c in sphereHits)
        {
            if (!string.IsNullOrEmpty(targetBallTag) && c.CompareTag(targetBallTag))
            {
                gripperIRHitObject = c.gameObject;
                return 1;
            }
        }
        return 0;
    }

    void OnDrawGizmos()
    {
        if (!drawGizmos) return;

        // УЗ-конус
        if (centerPoint != null)
        {
            Gizmos.color = new Color(0f, 0.7f, 1f, 0.8f);
            int rays = Mathf.Max(1, usRaysPerAxis);
            float step = (rays > 1) ? (2f * usHalfConeAngle) / (rays - 1) : 0f;
            for (int i = 0; i < rays; i++)
            {
                float yaw = (rays == 1) ? 0f : -usHalfConeAngle + step * i;
                for (int j = 0; j < rays; j++)
                {
                    float pitch = (rays == 1) ? 0f : -usHalfConeAngle + step * j;
                    Quaternion rot = centerPoint.rotation * Quaternion.Euler(pitch, yaw, 0f);
                    Vector3 dir = rot * Vector3.forward;
                    Gizmos.DrawRay(centerPoint.position, dir * usMaxRange);
                }
            }
        }

        // Боковые ИК
        Gizmos.color = Color.red;
        if (leftIRPoint  != null) Gizmos.DrawRay(leftIRPoint.position,  leftIRPoint.forward  * irObstacleRange);
        if (rightIRPoint != null) Gizmos.DrawRay(rightIRPoint.position, rightIRPoint.forward * irObstacleRange);

        // ИК клешни
        if (gripperIRPoint != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawRay(gripperIRPoint.position, gripperIRPoint.forward * gripperIRRange);
        }
    }
}
