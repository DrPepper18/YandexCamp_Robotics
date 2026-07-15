using UnityEngine;

/// <summary>
/// Симуляция сенсоров робота через Physics.Raycast.
/// - Два УЗ датчика (левый и правый): веер лучей в конусе ~30°, минимальная дистанция, игнор мяча.
/// - Три ИК датчика препятствий (лево, центр, право): короткий луч, 1 если стена рядом, 0 если пусто.
/// - ИК датчик клешни: короткий луч внутрь захвата, реагирует только на объект с тегом мяча.
/// Все лучи стреляют по локальной оси forward соответствующей точки-якоря —
/// поэтому просто разверни каждую точку в редакторе так, чтобы её синяя стрелка
/// смотрела туда, куда должен «смотреть» датчик.
/// </summary>
public class VirtualSensors : MonoBehaviour
{
    [Header("УЗ датчики (Empty GameObject'ы)")]
    [SerializeField] private Transform leftSuperSonic;   // левый УЗ
    [SerializeField] private Transform rightSuperSonic;  // правый УЗ

    [Header("ИК препятствий")]
    [SerializeField] private Transform leftIRPoint;      // ИК слева
    [SerializeField] private Transform centerIRPoint;    // ИК по центру
    [SerializeField] private Transform rightIRPoint;     // ИК справа

    [Header("ИК клешни (опционально)")]
    [SerializeField] private Transform gripperIRPoint;   // ИК внутрь захвата

    [Header("Настройки ультразвука")]
    [Tooltip("Максимальная дистанция УЗ, м")]
    [SerializeField] private float ultrasonicMaxDistance = 2.0f;
    [Tooltip("Полный угол конуса обзора, градусов")]
    [SerializeField] private float ultrasonicConeAngle = 30f;
    [Tooltip("Количество лучей в веере (нечётное — центральный + симметричные)")]
    [SerializeField] private int ultrasonicRayCount = 5;
    [Tooltip("Тег объекта, который УЗ игнорирует (обычно мяч)")]
    [SerializeField] private string ultrasonicIgnoreTag = "TargetBall";

    [Header("Настройки ИК препятствий")]
    [Tooltip("Дальность ИК препятствий, м (реальные ~15 см)")]
    [SerializeField] private float irObstacleDistance = 0.15f;

    [Header("Настройки ИК клешни")]
    [Tooltip("Дальность ИК клешни, м (реальные ~7-8 см)")]
    [SerializeField] private float gripperIRDistance = 0.08f;
    [Tooltip("Тег мяча, который ловит датчик клешни")]
    [SerializeField] private string ballTag = "TargetBall";

    [Header("Отладка")]
    [SerializeField] private bool drawGizmos = true;

    // ---- Публичные показания датчиков ----
    /// <summary>Левый УЗ: 0 — вплотную, 1 — чисто.</summary>
    public float UltrasonicLeft { get; private set; } = 1f;
    /// <summary>Правый УЗ: 0 — вплотную, 1 — чисто.</summary>
    public float UltrasonicRight { get; private set; } = 1f;

    /// <summary>1 если слева близко стена, иначе 0.</summary>
    public int LeftIR { get; private set; }
    /// <summary>1 если по центру близко стена, иначе 0.</summary>
    public int CenterIR { get; private set; }
    /// <summary>1 если справа близко стена, иначе 0.</summary>
    public int RightIR { get; private set; }

    /// <summary>1 если в клешне мяч, иначе 0.</summary>
    public int GripperIR { get; private set; }

    private void FixedUpdate()
    {
        UltrasonicLeft  = ReadUltrasonic(leftSuperSonic);
        UltrasonicRight = ReadUltrasonic(rightSuperSonic);

        LeftIR   = ReadObstacleIR(leftIRPoint,   irObstacleDistance);
        CenterIR = ReadObstacleIR(centerIRPoint, irObstacleDistance);
        RightIR  = ReadObstacleIR(rightIRPoint,  irObstacleDistance);

        GripperIR = ReadGripperIR();
    }

    // ---- УЗ: веер лучей, ищем минимум, игнорируя мяч ----
    private float ReadUltrasonic(Transform anchor)
    {
        if (anchor == null) return 1f;

        float minDistance = ultrasonicMaxDistance;
        int rays = Mathf.Max(1, ultrasonicRayCount);
        float halfCone = ultrasonicConeAngle * 0.5f;

        for (int i = 0; i < rays; i++)
        {
            float t = rays == 1 ? 0.5f : (float)i / (rays - 1);
            float angle = Mathf.Lerp(-halfCone, halfCone, t);

            Vector3 dir = Quaternion.AngleAxis(angle, anchor.up) * anchor.forward;

            // RaycastAll, чтобы можно было пропустить мяч и упереться в стену за ним
            RaycastHit[] hits = Physics.RaycastAll(anchor.position, dir, ultrasonicMaxDistance);
            System.Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));

            foreach (var hit in hits)
            {
                if (!string.IsNullOrEmpty(ultrasonicIgnoreTag) && hit.collider.CompareTag(ultrasonicIgnoreTag))
                    continue;
                // не считаем самого себя (если коллайдеры робота попали в луч)
                if (hit.collider.transform.IsChildOf(transform)) continue;

                if (hit.distance < minDistance) minDistance = hit.distance;
                break; // ближайший подходящий — берём его
            }
        }

        // 0 (вплотную) .. 1 (чисто)
        return Mathf.Clamp01(minDistance / ultrasonicMaxDistance);
    }

    // ---- Короткий ИК: 1 если что-то ближе distance, 0 если пусто ----
    private int ReadObstacleIR(Transform anchor, float distance)
    {
        if (anchor == null) return 0;

        if (Physics.Raycast(anchor.position, anchor.forward, out RaycastHit hit, distance))
        {
            if (hit.collider.transform.IsChildOf(transform)) return 0; // не считаем себя
            return 1;
        }
        return 0;
    }

    // ---- ИК клешни: 1 только если попал по мячу ----
    private int ReadGripperIR()
    {
        if (gripperIRPoint == null) return 0;

        RaycastHit[] hits = Physics.RaycastAll(gripperIRPoint.position, gripperIRPoint.forward, gripperIRDistance);
        foreach (var hit in hits)
        {
            if (hit.collider.CompareTag(ballTag)) return 1;
        }
        return 0;
    }

    // ---- Визуализация в редакторе ----
    private void OnDrawGizmos()
    {
        if (!drawGizmos) return;

        // УЗ конусы — левый и правый
        Gizmos.color = Color.cyan;
        DrawUltrasonicCone(leftSuperSonic);
        DrawUltrasonicCone(rightSuperSonic);

        // ИК препятствий
        Gizmos.color = Color.yellow;
        if (leftIRPoint   != null) Gizmos.DrawRay(leftIRPoint.position,   leftIRPoint.forward   * irObstacleDistance);
        if (centerIRPoint != null) Gizmos.DrawRay(centerIRPoint.position, centerIRPoint.forward * irObstacleDistance);
        if (rightIRPoint  != null) Gizmos.DrawRay(rightIRPoint.position,  rightIRPoint.forward  * irObstacleDistance);

        // ИК клешни
        Gizmos.color = Color.magenta;
        if (gripperIRPoint != null) Gizmos.DrawRay(gripperIRPoint.position, gripperIRPoint.forward * gripperIRDistance);
    }

    private void DrawUltrasonicCone(Transform anchor)
    {
        if (anchor == null) return;

        int rays = Mathf.Max(1, ultrasonicRayCount);
        float halfCone = ultrasonicConeAngle * 0.5f;
        for (int i = 0; i < rays; i++)
        {
            float t = rays == 1 ? 0.5f : (float)i / (rays - 1);
            float angle = Mathf.Lerp(-halfCone, halfCone, t);
            Vector3 dir = Quaternion.AngleAxis(angle, anchor.up) * anchor.forward;
            Gizmos.DrawRay(anchor.position, dir * ultrasonicMaxDistance);
        }
    }
}