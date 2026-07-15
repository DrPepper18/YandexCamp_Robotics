using UnityEngine;

/// <summary>
/// Отладочный компонент: показывает реальный мировой размер объекта в метрах.
/// Вешать на любой GameObject с Renderer или Collider — и он в Inspector
/// покажет bounds.size (в метрах), а также нарисует ограничивающий бокс.
/// </summary>
[ExecuteAlways]
public class SizeInspector : MonoBehaviour
{
    public enum SizeSource { Renderer, Collider, Both }

    [SerializeField] private SizeSource source = SizeSource.Both;
    [SerializeField] private bool drawGizmo = true;

    [Header("Результат (только для чтения)")]
    [SerializeField] private Vector3 rendererSize;
    [SerializeField] private Vector3 colliderSize;
    [SerializeField] private Vector3 worldScale;

    private void Update()
    {
        worldScale = transform.lossyScale;

        if (source == SizeSource.Renderer || source == SizeSource.Both)
        {
            var r = GetComponentInChildren<Renderer>();
            rendererSize = r != null ? r.bounds.size : Vector3.zero;
        }

        if (source == SizeSource.Collider || source == SizeSource.Both)
        {
            var c = GetComponentInChildren<Collider>();
            colliderSize = c != null ? c.bounds.size : Vector3.zero;
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawGizmo) return;

        var r = GetComponentInChildren<Renderer>();
        if (r != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(r.bounds.center, r.bounds.size);
        }

        var c = GetComponentInChildren<Collider>();
        if (c != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(c.bounds.center, c.bounds.size);
        }
    }
}