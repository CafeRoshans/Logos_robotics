using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Контроллер клешни с логическим захватом мяча.
/// Физика захвата круглого мяча твёрдыми губками нестабильна (проскальзывание, вылет),
/// поэтому используем kinematic-родительство: при захвате мяч крепится к HoldPoint,
/// его Rigidbody становится kinematic, коллайдер отключается.
/// При отпускании — восстанавливаем исходное состояние.
/// </summary>
public class GripperController : MonoBehaviour
{
    [Header("Ссылки")]
    [Tooltip("Датчики робота — читаем gripperIR")]
    public VirtualSensors sensors;

    [Tooltip("Точка удержания — пустой дочерний объект между губками клешни")]
    public Transform holdPoint;

    [Header("Настройки")]
    [Tooltip("Тег объекта, который можно захватить")]
    public string targetBallTag = "TargetBall";

    [Tooltip("Радиус поиска мяча вокруг HoldPoint при попытке захвата (м) — это ЗАПАСНОЙ путь " +
             "(если ИК-датчик клешни промолчал). БЫЛО в 2× больше основного gripperIRRange — " +
             "мяч (реальный радиус ~3.5 см при текущем масштабе) мог засчитаться захваченным " +
             "почти в 6 своих диаметрах от HoldPoint, то есть 'мягкий запас' был мягче основного " +
             "сенсора, а не наоборот. Приведено к масштабу мяча + небольшой запас на геометрию " +
             "губок — ПРОВЕРЬ по факту реальный охват клешни GFS-X и поправь, если отличается.")]
    public float grabSearchRadius = 0.06f;

    [Tooltip("Автоматически захватывать мяч, если gripperIR == 1 и команда захвата активна")]
    public bool autoGrabOnSensor = true;

    [Header("Тестовое управление")]
    [Tooltip("Клавиша для тестового захвата/отпускания (для отладки без ML). Используется новый Input System.")]
    public Key toggleKey = Key.Space;

    // Состояние
    [HideInInspector] public bool isHolding = false;
    [HideInInspector] public bool grabCommand = false;   // внешняя команда: 1 = сжать, 0 = разжать

    // Счётчики за эпизод — для диагностики в TensorBoard (см. RobotBrain.LogEpisodeStats).
    // Разделение полезно: если сеть почти всегда хватает через "резерв" (сферу), а не через
    // ИК-датчик — это сигнал либо плохо откалиброванного ИК, либо того, что сеть не может
    // точно прицелиться и полагается на более широкий/мягкий запас.
    [HideInInspector] public int grabsViaSensorThisEpisode   = 0;
    [HideInInspector] public int grabsViaFallbackThisEpisode = 0;

    /// <summary>Сбрасывает счётчики захвата — вызывай из RobotBrain.OnEpisodeBegin.</summary>
    public void ResetGrabStats()
    {
        grabsViaSensorThisEpisode   = 0;
        grabsViaFallbackThisEpisode = 0;
    }

    private Rigidbody heldRb;
    /// <summary>Публичный доступ к захваченному объекту — RobotBrain использует
    /// чтобы верифицировать: захвачен ли ИМЕННО его целевой мяч, а не чужой из соседней арены.</summary>
    public Rigidbody HeldRigidbody => heldRb;
    private Collider  heldCollider;
    private Transform heldOriginalParent;
    private bool      heldOriginalKinematic;
    private bool      heldOriginalColliderEnabled;
    private Vector3   heldOriginalLocalScale;
    private Vector3   heldOriginalWorldPosition;

    void Update()
    {
        // Ручное переключение для отладки (новый Input System)
        if (Keyboard.current != null && Keyboard.current[toggleKey].wasPressedThisFrame)
            grabCommand = !grabCommand;

        if (grabCommand)
        {
            if (!isHolding && CanGrab())
                TryGrab();
        }
        else
        {
            if (isHolding)
                Release();
        }
    }

    /// <summary>
    /// Условие захвата: либо датчик клешни видит мяч, либо (для мягкого запаса) он в радиусе поиска.
    /// </summary>
    bool CanGrab()
    {
        if (autoGrabOnSensor && sensors != null && sensors.gripperIRHitObject != null)
            return true;

        return FindBallNearHoldPoint() != null;
    }

    /// <summary>
    /// Пытается зацепить мяч. Приоритет:
    ///   1) объект, который сейчас видит датчик клешни (gripperIRHitObject);
    ///   2) ближайший мяч в grabSearchRadius вокруг HoldPoint.
    /// </summary>
    public void TryGrab()
    {
        if (isHolding) return;
        if (holdPoint == null)
        {
            Debug.LogWarning("[GripperController] HoldPoint не назначен.");
            return;
        }

        GameObject ball = null;
        bool viaSensor = sensors != null && sensors.gripperIRHitObject != null;

        if (viaSensor)
            ball = sensors.gripperIRHitObject;
        else
            ball = FindBallNearHoldPoint();

        if (ball == null)
        {
            Debug.Log("[GripperController] Мяч не найден: датчик клешни ничего не видит, и в grabSearchRadius пусто.");
            return;
        }

        if (viaSensor) grabsViaSensorThisEpisode++;
        else           grabsViaFallbackThisEpisode++;

        Rigidbody rb  = ball.GetComponent<Rigidbody>();
        Collider  col = ball.GetComponent<Collider>();
        if (rb == null)
        {
            Debug.LogWarning($"[GripperController] У '{ball.name}' нет Rigidbody — захват невозможен.");
            return;
        }

        // Сохраняем исходное состояние — чтобы корректно восстановить при отпускании.
        // ВАЖНО: сохраняем и localScale, потому что после SetParent(holdPoint, false)
        // мировой scale мяча становится = 1 × holdPoint.lossyScale.
        // Если у клешни/арены в цепочке родителей есть нестандартный scale,
        // мяч приобретает странный размер, а после Release он "запомнится" физикой
        // и выдаст NaN в distanceForSort/AABB.
        heldRb                       = rb;
        heldCollider                 = col;
        heldOriginalParent           = ball.transform.parent;
        heldOriginalKinematic        = rb.isKinematic;
        heldOriginalColliderEnabled  = col != null ? col.enabled : true;
        heldOriginalLocalScale       = ball.transform.localScale;
        heldOriginalWorldPosition    = ball.transform.position;

        // Останавливаем физику
        rb.linearVelocity  = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.isKinematic     = true;

        if (col != null) col.enabled = false;

        // Крепим к HoldPoint. worldPositionStays=false: мяч теряет свои world-координаты
        // и позиционируется по HoldPoint. localScale ставим (1,1,1), потом мы всё равно
        // восстановим оригинал при Release.
        ball.transform.SetParent(holdPoint, worldPositionStays: false);
        ball.transform.localPosition = Vector3.zero;
        ball.transform.localRotation = Quaternion.identity;


        // Попытка схватить мяч н 1
        // if (rosBridge != null)
        // {
        //     rosBridge.PublishGripperCmd(2); // Отправить команду закрытия в ROS
        // }

        isHolding = true;
    }

    /// <summary>
    /// Отпустить мяч: вернуть физику, коллайдер и родителя.
    /// </summary>
    public void Release()
    {
        if (!isHolding || heldRb == null)
        {
            isHolding = false;
            return;
        }

        Transform ballT = heldRb.transform;

        // 1. Отвязываем от клешни. worldPositionStays=false — не сохраняем
        //    мировой transform, потому что он мог получить кривой scale от иерархии клешни.
        //    Восстановим руками ниже.
        ballT.SetParent(heldOriginalParent, worldPositionStays: false);

        // 2. Восстанавливаем оригинальный localScale — ключевой момент,
        //    без него мяч может остаться с scale = 0 или огромным,
        //    и физика уронит NaN на всех последующих кадрах.
        ballT.localScale = heldOriginalLocalScale;

        // 3. Ставим мировую позицию туда, где мяч был до захвата (или где сейчас HoldPoint —
        //    но безопаснее сначала положить в исходную точку, потом RobotBrain сам его
        //    переставит через targetBall.position = ballPos).
        ballT.position = heldOriginalWorldPosition;
        ballT.rotation = Quaternion.identity;

        // 4. Возвращаем коллайдер и кинематику
        if (heldCollider != null) heldCollider.enabled = heldOriginalColliderEnabled;
        heldRb.isKinematic = heldOriginalKinematic;

        // 5. Обнуляем скорости — мяч ляжет, а не улетит.
        //    (Только если Rigidbody не остался kinematic, иначе Unity ругнётся.)
        if (!heldRb.isKinematic)
        {
            heldRb.linearVelocity  = Vector3.zero;
            heldRb.angularVelocity = Vector3.zero;
        }

        heldRb       = null;
        heldCollider = null;
        heldOriginalParent = null;
        isHolding    = false;
    }

    /// <summary>
    /// Поиск ближайшего мяча в сфере вокруг HoldPoint.
    /// </summary>
    GameObject FindBallNearHoldPoint()
    {
        if (holdPoint == null) return null;

        Collider[] hits = Physics.OverlapSphere(holdPoint.position, grabSearchRadius, ~0, QueryTriggerInteraction.Collide);
        float bestDist = float.MaxValue;
        GameObject best = null;

        foreach (var h in hits)
        {
            if (!h.CompareTag(targetBallTag)) continue;

            float d = Vector3.Distance(h.transform.position, holdPoint.position);
            if (d < bestDist)
            {
                bestDist = d;
                best     = h.gameObject;
            }
        }
        return best;
    }

    void OnDrawGizmosSelected()
    {
        if (holdPoint == null) return;
        Gizmos.color = isHolding ? Color.green : new Color(1f, 0.8f, 0f, 0.6f);
        Gizmos.DrawWireSphere(holdPoint.position, grabSearchRadius);
    }
}