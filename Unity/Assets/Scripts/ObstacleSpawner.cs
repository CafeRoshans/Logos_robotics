using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Спавнит случайное подмножество препятствий (блоков) в предопределённых точках арены.
/// Один экземпляр — на одну арену. Позиции точек и префаб блока настраиваются в инспекторе.
/// Вызов Respawn() удаляет предыдущий комплект и раскладывает новый.
///
/// Использование:
///   RobotBrain.OnEpisodeBegin → obstacleSpawner.Respawn();
/// </summary>
public class ObstacleSpawner : MonoBehaviour
{
    [Header("Что спавнить")]
    [Tooltip("Префаб блока-препятствия (стандартный Cube с коллайдером)")]
    public GameObject obstaclePrefab;

    [Header("Точки-кандидаты (10 штук на арене)")]
    [Tooltip("Empty GameObjects, где может появиться препятствие. " +
             "Позиция и rotation берутся из этого Transform.")]
    public Transform[] candidatePoints;

    [Header("Настройки")]
    [Tooltip("Сколько препятствий спавнить из candidatePoints за эпизод")]
    [Min(0)]
    public int spawnCount = 5;

    [Tooltip("Родитель для заспавненных препятствий. Если null — эта же арена (transform).")]
    public Transform spawnParent;

    [Tooltip("Случайный поворот препятствия вокруг Y при спавне. При включении на каждом " +
             "эпизоде препятствие получает поворот 0/45/90° (равновероятно) — добавляет " +
             "разнообразие ориентаций для доменной рандомизации.")]
    public bool randomizeYawOnSpawn = true;

    [Tooltip("Список углов (град) для случайного Y-поворота. Пример: 0,45,90.")]
    public float[] spawnYawChoices = new float[] { 0f, 45f, 90f };

    private readonly List<GameObject> _current = new List<GameObject>();

    /// <summary>
    /// Список точек-кандидатов, которые НЕ заняты препятствиями после последнего Respawn().
    /// Используется для случайного спавна робота и мяча в свободных местах.
    /// </summary>
    public readonly List<Transform> unusedPoints = new List<Transform>();

    /// <summary>
    /// Удалить старые препятствия и разложить новый случайный набор.
    /// </summary>
    public void Respawn()
    {
        // 1. Убрать предыдущие
        for (int i = 0; i < _current.Count; i++)
            if (_current[i] != null) Destroy(_current[i]);
        _current.Clear();

        if (obstaclePrefab == null || candidatePoints == null || candidatePoints.Length == 0)
            return;

        // 2. Собрать индексы точек, перемешать (Fisher-Yates)
        int n = candidatePoints.Length;
        var indices = new int[n];
        for (int i = 0; i < n; i++) indices[i] = i;
        for (int i = n - 1; i > 0; i--)
        {
            int j = Random.Range(0, i + 1);
            (indices[i], indices[j]) = (indices[j], indices[i]);
        }

        // 3. Взять первые spawnCount из перемешанного массива
        int take = Mathf.Min(spawnCount, n);

        for (int i = 0; i < take; i++)
        {
            var pt = candidatePoints[indices[i]];
            if (pt == null) continue;

            // Валидация точки — если Transform каким-то образом получил NaN/Inf,
            // не спавним в эту позицию (иначе Rigidbody вылетит в бесконечность).
            Vector3 p = pt.position;
            if (float.IsNaN(p.x) || float.IsInfinity(p.x) ||
                float.IsNaN(p.y) || float.IsInfinity(p.y) ||
                float.IsNaN(p.z) || float.IsInfinity(p.z))
            {
                Debug.LogWarning($"[ObstacleSpawner] Точка '{pt.name}' имеет невалидную позицию: {p}");
                continue;
            }

            // Спавним в мировых координатах БЕЗ родителя. Так на блок не
            // наследуется никакой lossyScale от арены — размеры блока всегда
            // такие, как в префабе. Раньше компенсировали scale формулой,
            // но при экстремальных значениях (< 0.001 или отрицательных)
            // это давало бесконечные scale и NaN в физике.
            Quaternion rot = pt.rotation;
            if (randomizeYawOnSpawn && spawnYawChoices != null && spawnYawChoices.Length > 0)
            {
                float yaw = spawnYawChoices[Random.Range(0, spawnYawChoices.Length)];
                rot *= Quaternion.Euler(0f, yaw, 0f);
            }
            var obj = Instantiate(obstaclePrefab, p, rot);
            _current.Add(obj);
        }

        // Публикуем оставшиеся точки как «свободные» — из них RobotBrain
        // выберет случайные позиции для робота
        unusedPoints.Clear();
        for (int i = take; i < n; i++)
        {
            var pt = candidatePoints[indices[i]];
            if (pt != null) unusedPoints.Add(pt);
        }
    }

    void OnDrawGizmos()
    {
        if (candidatePoints == null) return;
        for (int i = 0; i < candidatePoints.Length; i++)
        {
            var pt = candidatePoints[i];
            if (pt == null) continue;
            Gizmos.color = new Color(1f, 0.7f, 0f, 0.6f);
            Gizmos.DrawWireCube(pt.position, Vector3.one * 0.15f);
            Gizmos.DrawLine(pt.position, pt.position + Vector3.up * 0.3f);
        }
    }
}
