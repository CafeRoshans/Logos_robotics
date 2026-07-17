using UnityEngine;

/// <summary>
/// Раскладывает N копий префаба тренировочной арены в сетку grid×grid с шагом spacing.
/// Каждая копия — независимая среда для параллельного обучения ML-Agents.
/// Все агенты имеют одинаковый Behavior Name, тренер объединяет их опыт автоматически.
/// </summary>
public class AreaSpawner : MonoBehaviour
{
    [Header("Что спавнить")]
    [Tooltip("Префаб TrainingArea (робот + мяч + арена + камера)")]
    public GameObject areaPrefab;

    [Header("Сетка")]
    [Tooltip("Сторона сетки. Итого будет gridSize×gridSize арен.")]
    [Range(1, 8)]
    public int gridSize = 4;

    [Tooltip("Расстояние между центрами соседних арен (м). Ставь с запасом: " +
             "если < 2× дальности УЗ (usMaxRange), лучи одной арены могут задевать соседнюю.")]
    public float spacing = 10f;

    [Header("Расположение")]
    [Tooltip("Y стартовой позиции всех арен")]
    public float baseHeight = 0f;

    void Awake()
    {
        if (areaPrefab == null)
        {
            Debug.LogError("[AreaSpawner] areaPrefab не назначен.");
            return;
        }

        float half = (gridSize - 1) * spacing * 0.5f;

        for (int x = 0; x < gridSize; x++)
        {
            for (int z = 0; z < gridSize; z++)
            {
                Vector3 pos = new Vector3(
                    x * spacing - half,   // центрируем сетку по X
                    baseHeight,
                    z * spacing - half    // центрируем сетку по Z
                );
                var area = Instantiate(areaPrefab, pos, Quaternion.identity, transform);
                area.name = $"Area_{x}_{z}";
            }
        }

        Debug.Log($"[AreaSpawner] Разложено {gridSize * gridSize} арен с шагом {spacing} м.");
    }

    void OnDrawGizmosSelected()
    {
        // Превью сетки в редакторе, до Play
        Gizmos.color = new Color(0f, 1f, 1f, 0.4f);
        float half = (gridSize - 1) * spacing * 0.5f;

        for (int x = 0; x < gridSize; x++)
        {
            for (int z = 0; z < gridSize; z++)
            {
                Vector3 pos = transform.position + new Vector3(
                    x * spacing - half,
                    baseHeight,
                    z * spacing - half
                );
                Gizmos.DrawWireCube(pos, Vector3.one * (spacing * 0.9f));
            }
        }
    }
}
