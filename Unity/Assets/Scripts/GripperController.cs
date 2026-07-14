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

    [Tooltip("Радиус поиска мяча вокруг HoldPoint при попытке захвата (м)")]
    public float grabSearchRadius = 0.1f;

    [Tooltip("Автоматически захватывать мяч, если gripperIR == 1 и команда захвата активна")]
    public bool autoGrabOnSensor = true;

    [Header("Тестовое управление")]
    [Tooltip("Клавиша для тестового захвата/отпускания (для отладки без ML). Используется новый Input System.")]
    public Key toggleKey = Key.Space;

    // Состояние
    [HideInInspector] public bool isHolding = false;
    [HideInInspector] public bool grabCommand = false;   // внешняя команда: 1 = сжать, 0 = разжать

    private Rigidbody heldRb;
    private Collider  heldCollider;
    private Transform heldOriginalParent;
    private bool      heldOriginalKinematic;
    private bool      heldOriginalColliderEnabled;

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

        if (sensors != null && sensors.gripperIRHitObject != null)
            ball = sensors.gripperIRHitObject;
        else
            ball = FindBallNearHoldPoint();

        if (ball == null)
        {
            Debug.Log("[GripperController] Мяч не найден: датчик клешни ничего не видит, и в grabSearchRadius пусто.");
            return;
        }

        Rigidbody rb  = ball.GetComponent<Rigidbody>();
        Collider  col = ball.GetComponent<Collider>();
        if (rb == null)
        {
            Debug.LogWarning($"[GripperController] У '{ball.name}' нет Rigidbody — захват невозможен.");
            return;
        }

        // Сохраняем исходное состояние — чтобы корректно восстановить при отпускании
        heldRb                       = rb;
        heldCollider                 = col;
        heldOriginalParent           = ball.transform.parent;
        heldOriginalKinematic        = rb.isKinematic;
        heldOriginalColliderEnabled  = col != null ? col.enabled : true;

        // Останавливаем физику
        rb.linearVelocity  = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.isKinematic     = true;

        if (col != null) col.enabled = false;

        // Крепим к HoldPoint
        ball.transform.SetParent(holdPoint, worldPositionStays: false);
        ball.transform.localPosition = Vector3.zero;
        ball.transform.localRotation = Quaternion.identity;

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

        // Отвязываем от клешни — важно ДО включения физики,
        // чтобы не унаследовать движение родителя как трансформ.
        ballT.SetParent(heldOriginalParent, worldPositionStays: true);

        if (heldCollider != null) heldCollider.enabled = heldOriginalColliderEnabled;
        heldRb.isKinematic = heldOriginalKinematic;

        // Обнуляем скорости — мяч ляжет, а не улетит.
        heldRb.linearVelocity  = Vector3.zero;
        heldRb.angularVelocity = Vector3.zero;

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
