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

    [Header("Layers / tags")]
    public LayerMask obstacleMask = ~0; // everything by default
    public string ballTag = "TargetBall";

<<<<<<< HEAD
    // --- Public read-only results, updated each frame ---
    public float UltrasonicDistance01 { get; private set; } = 1f; // 0 = touching, 1 = clear
    public int LeftIR { get; private set; }
    public int RightIR { get; private set; }
=======
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
>>>>>>> ade5866945b7458351b0d9a434b11a7a8c2180bb
    public int GripperIR { get; private set; }

    private void Update()
    {
<<<<<<< HEAD
        UltrasonicDistance01 = ReadUltrasonic();
        LeftIR = ReadShortRangeIR(leftIRPoint, irObstacleRange, ignoreBall: false);
        RightIR = ReadShortRangeIR(rightIRPoint, irObstacleRange, ignoreBall: false);
        GripperIR = ReadGripperIR();
    }

    /// <summary>
    /// Casts a fan of rays across the ultrasonic cone, finds the closest hit
    /// (ignoring the ball, since it's too small for real ultrasonic sensors to see),
    /// and returns a normalized distance: 0 = obstacle right at the sensor, 1 = clear.
    /// </summary>
    private float ReadUltrasonic()
=======
        UltrasonicLeft  = ReadUltrasonic(leftSuperSonic);
        UltrasonicRight = ReadUltrasonic(rightSuperSonic);

        LeftIR   = ReadObstacleIR(leftIRPoint,   irObstacleDistance);
        CenterIR = ReadObstacleIR(centerIRPoint, irObstacleDistance);
        RightIR  = ReadObstacleIR(rightIRPoint,  irObstacleDistance);

        GripperIR = ReadGripperIR();
    }

    // ---- УЗ: веер лучей, ищем минимум, игнорируя мяч ----
    private float ReadUltrasonic(Transform anchor)
>>>>>>> ade5866945b7458351b0d9a434b11a7a8c2180bb
    {
        if (anchor == null) return 1f;

        float closestDistance = ultrasonicRange;
        bool hitSomething = false;

        int rays = Mathf.Max(1, ultrasonicRayCount);
        float halfAngle = ultrasonicConeAngle * 0.5f;

        for (int i = 0; i < rays; i++)
        {
<<<<<<< HEAD
            float t = rays == 1 ? 0f : (float)i / (rays - 1); // 0..1
            float angle = Mathf.Lerp(-halfAngle, halfAngle, t);

            Vector3 direction = Quaternion.Euler(0f, angle, 0f) * centerPoint.forward;

            if (Physics.Raycast(centerPoint.position, direction, out RaycastHit hit, ultrasonicRange, obstacleMask))
=======
            float t = rays == 1 ? 0.5f : (float)i / (rays - 1);
            float angle = Mathf.Lerp(-halfCone, halfCone, t);

            Vector3 dir = Quaternion.AngleAxis(angle, anchor.up) * anchor.forward;

            // RaycastAll, чтобы можно было пропустить мяч и упереться в стену за ним
            RaycastHit[] hits = Physics.RaycastAll(anchor.position, dir, ultrasonicMaxDistance);
            System.Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));

            foreach (var hit in hits)
>>>>>>> ade5866945b7458351b0d9a434b11a7a8c2180bb
            {
                if (hit.collider.CompareTag(ballTag))
                {
                    continue; // ultrasonic can't reliably see the small ball, ignore it
                }

                hitSomething = true;
                if (hit.distance < closestDistance)
                {
                    closestDistance = hit.distance;
                }
            }
        }

        if (!hitSomething)
        {
            return 1f;
        }

        return Mathf.Clamp01(closestDistance / ultrasonicRange);
    }

    /// <summary>
    /// Single short raycast used for the left/right IR obstacle sensors.
    /// Returns 1 if a wall/obstacle is detected within range, 0 otherwise.
    /// </summary>
    private int ReadShortRangeIR(Transform origin, float range, bool ignoreBall)
    {
        if (origin == null) return 0;

        if (Physics.Raycast(origin.position, origin.forward, out RaycastHit hit, range, obstacleMask))
        {
            if (ignoreBall && hit.collider.CompareTag(ballTag))
            {
                return 0;
            }
            return 1;
        }

        return 0;
    }

    /// <summary>
    /// Detects the target ball at close range inside the gripper.
    /// Returns 1 if the ball is present within gripperIRRange, 0 otherwise.
    /// </summary>
    private int ReadGripperIR()
    {
        if (gripperIRPoint == null) return 0;

        if (Physics.Raycast(gripperIRPoint.position, gripperIRPoint.forward, out RaycastHit hit, gripperIRRange, obstacleMask))
        {
<<<<<<< HEAD
            if (hit.collider.CompareTag(ballTag))
            {
                return 1;
            }
        }

        return 0;
    }
=======
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
>>>>>>> ade5866945b7458351b0d9a434b11a7a8c2180bb
}