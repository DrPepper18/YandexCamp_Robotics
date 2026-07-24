using UnityEngine;

/// <summary>
/// Симуляция сенсоров робота через Physics.Raycast.
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

    [Header("Настройки ИК препятствий")]
    [Tooltip("Дальность ИК препятствий, м (реальные ~15 см)")]
    [SerializeField] private float irObstacleDistance = 0.15f;

    [Header("Настройки ИК клешни")]
    [Tooltip("Дальность ИК клешни, м (реальные ~7-8 см)")]
    [SerializeField] private float gripperIRDistance = 0.08f;

    [Header("Layers / tags")]
    [Tooltip("Слои, которые считаются препятствиями/стенами")]
    public LayerMask obstacleMask = ~0; // everything by default
    [Tooltip("Тег мяча — УЗ его игнорирует, ИК клешни наоборот ищет именно его")]
    public string ballTag = "TargetBall";

    [Header("Отладка")]
    [SerializeField] private bool drawGizmos = true;

    // ---- Публичные показания датчиков ----
    public float UltrasonicLeft { get; private set; } = 1f;
    public float UltrasonicRight { get; private set; } = 1f;

    public int LeftIR { get; private set; }
    public int CenterIR { get; private set; }
    public int RightIR { get; private set; }

    public int GripperIR { get; private set; }
    public Collider GripperDetectedBall { get; private set; }

    // Переносим считывание в FixedUpdate для точной синхронизации с физикой
    private void FixedUpdate()
    {
        UltrasonicLeft = ReadUltrasonic(leftSuperSonic);
        UltrasonicRight = ReadUltrasonic(rightSuperSonic);

        LeftIR = ReadObstacleIR(leftIRPoint, irObstacleDistance);
        CenterIR = ReadObstacleIR(centerIRPoint, irObstacleDistance);
        RightIR = ReadObstacleIR(rightIRPoint, irObstacleDistance);

        GripperIR = ReadGripperIR();
    }

    private float ReadUltrasonic(Transform anchor)
    {
        if (anchor == null) return 1f;

        float closestDistance = ultrasonicMaxDistance;
        bool hitSomething = false;

        int rays = Mathf.Max(1, ultrasonicRayCount);
        float halfAngle = ultrasonicConeAngle * 0.5f;

        for (int i = 0; i < rays; i++)
        {
            float t = rays == 1 ? 0.5f : (float)i / (rays - 1);
            float angle = Mathf.Lerp(-halfAngle, halfAngle, t);

            Vector3 dir = Quaternion.AngleAxis(angle, anchor.up) * anchor.forward;

            RaycastHit[] hits = Physics.RaycastAll(anchor.position, dir, ultrasonicMaxDistance, obstacleMask);
            System.Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));

            foreach (var hit in hits)
            {
                if (hit.collider.CompareTag(ballTag))
                {
                    continue; // игнорируем мяч
                }

                hitSomething = true;
                if (hit.distance < closestDistance)
                {
                    closestDistance = hit.distance;
                }
                break;
            }
        }

        if (!hitSomething) return 1f;

        return Mathf.Clamp01(closestDistance / ultrasonicMaxDistance);
    }

    private int ReadObstacleIR(Transform origin, float range)
    {
        if (origin == null) return 0;

        RaycastHit[] hits = Physics.RaycastAll(origin.position, origin.forward, range, obstacleMask);
        foreach (var hit in hits)
        {
            // Игнорируем мяч, реагируем только на стены
            if (!hit.collider.CompareTag(ballTag))
            {
                return 1;
            }
        }

        return 0;
    }

    private int ReadGripperIR()
    {
        if (gripperIRPoint == null)
        {
            GripperDetectedBall = null;
            return 0;
        }

        if (Physics.Raycast(gripperIRPoint.position, gripperIRPoint.forward, out RaycastHit hit, gripperIRDistance, obstacleMask))
        {
            if (hit.collider.CompareTag(ballTag))
            {
                GripperDetectedBall = hit.collider;
                return 1;
            }
        }

        GripperDetectedBall = null;
        return 0;
    }

    // ---- Улучшенная отладка в Scene View ----
    private void OnDrawGizmos()
    {
        if (!drawGizmos) return;

        // УЗ конусы
        DrawUltrasonicCone(leftSuperSonic);
        DrawUltrasonicCone(rightSuperSonic);

        // ИК препятствий (подсветка красным, если есть попадание)
        DrawIRGizmo(leftIRPoint, irObstacleDistance, LeftIR == 1);
        DrawIRGizmo(centerIRPoint, irObstacleDistance, CenterIR == 1);
        DrawIRGizmo(rightIRPoint, irObstacleDistance, RightIR == 1);

        // ИК клешни (зеленый при зажатии мяча, фиолетовый — в поиске)
        if (gripperIRPoint != null)
        {
            Gizmos.color = GripperIR == 1 ? Color.green : Color.magenta;
            Gizmos.DrawRay(gripperIRPoint.position, gripperIRPoint.forward * gripperIRDistance);
        }
    }

    private void DrawIRGizmo(Transform point, float range, bool isHitting)
    {
        if (point == null) return;
        Gizmos.color = isHitting ? Color.red : Color.yellow;
        Gizmos.DrawRay(point.position, point.forward * range);
    }

    private void DrawUltrasonicCone(Transform anchor)
    {
        if (anchor == null) return;

        Gizmos.color = Color.cyan;
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