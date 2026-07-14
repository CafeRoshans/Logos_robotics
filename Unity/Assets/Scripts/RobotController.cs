using UnityEngine;
using UnityEngine.InputSystem; // Новая библиотека ввода

public class RobotController : MonoBehaviour
{
    [Header("Настройки движения")]
    public float moveSpeed = 5f;
    public float rotationSpeed = 10f;

    private Rigidbody rb;
    private Vector2 moveInput; // Новое поле для ввода

    void Start()
    {
        // interpolation = RigidbodyInterpolation.Interpolate;
        // collisionDetectionMode = CollisionDetectionMode.Continuous;

        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
        }
        rb.freezeRotation = true;
    }

    void Update()
    {
        // Читаем ввод из новой системы Input System
        // "WASD" и стрелки работают автоматически
        moveInput = Keyboard.current != null
            ? new Vector2(Keyboard.current.dKey.ReadValue() - Keyboard.current.aKey.ReadValue(),
                          Keyboard.current.wKey.ReadValue() - Keyboard.current.sKey.ReadValue())
            : Vector2.zero;

        if (moveInput.magnitude >= 0.1f)
        {
            Vector3 moveDirection = new Vector3(moveInput.x, 0f, moveInput.y);

            float targetAngle = Mathf.Atan2(moveDirection.x, moveDirection.z) * Mathf.Rad2Deg;
            float smoothAngle = Mathf.LerpAngle(transform.eulerAngles.y, targetAngle, rotationSpeed * Time.deltaTime);
            transform.rotation = Quaternion.Euler(0f, smoothAngle, 0f);

            transform.Translate(moveDirection.normalized * moveSpeed * Time.deltaTime, Space.World);
        }
    }
}