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
        Transform parent = spawnParent != null ? spawnParent : transform;

        for (int i = 0; i < take; i++)
        {
            var pt = candidatePoints[indices[i]];
            if (pt == null) continue;

            // Спавним БЕЗ родителя (в мировых координатах) — так на объект не
            // наследуется lossyScale арены/пола (иначе блок стал бы плоским,
            // если пол растянут по Y ~ 0.1).
            var obj = Instantiate(obstaclePrefab, pt.position, pt.rotation);

            // Прикрепляем к родителю с сохранением мировой позиции И компенсируем
            // масштаб родителя, чтобы мировые размеры блока = prefab.localScale.
            obj.transform.SetParent(parent, worldPositionStays: true);
            Vector3 ls = obstaclePrefab.transform.localScale;
            Vector3 pScale = parent.lossyScale;
            obj.transform.localScale = new Vector3(
                ls.x / Mathf.Max(0.0001f, pScale.x),
                ls.y / Mathf.Max(0.0001f, pScale.y),
                ls.z / Mathf.Max(0.0001f, pScale.z));

            _current.Add(obj);
        }

        // Публикуем оставшиеся точки как «свободные» — из них RobotBrain
        // выберет случайные позиции для робота и мяча.
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
