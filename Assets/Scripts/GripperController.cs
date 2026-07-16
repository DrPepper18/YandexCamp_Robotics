using UnityEngine;

/// <summary>
/// Controls the robot's claw/gripper.
/// Since physically holding a round object with rigid jaws is unreliable in
/// a physics simulation (the ball slips or is ejected due to friction solver
/// inaccuracies), this uses a "logical grip": when the gripper IR sensor
/// detects the ball, physics is disabled for it and it is parented to a
/// HoldPoint, so it moves rigidly with the claw. Opening the claw restores
/// normal physics.
/// </summary>
public class GripperController : MonoBehaviour
{
    [Header("Собственный поиск мяча")]
    [SerializeField] private Transform gripperIRPoint;   // точка внутри клешни
    [SerializeField] private float gripperRange = 0.08f;  // дальность ИК клешни
    [SerializeField] private string ballTag = "TargetBall"; // тег мяча для проверки

    [Header("References")]
    [Tooltip("Empty transform positioned between the claw's jaws")]
    public Transform holdPoint;

    
    [Header("Debug")]
    [Tooltip("Radius used only to draw the gizmo at holdPoint, purely visual")]
    [SerializeField] private float grabSearchRadius = 0.05f;

    [Header("State")]
    [Tooltip("True while the claw is commanded to be closed")]
    public bool isClawClosed;

    /// <summary>True while the gripper is currently holding the ball.</summary>
    public bool IsHolding => heldBallRb != null;

    /// <summary>True if a ball is currently detected in the gripper IR sensor.</summary>
    public bool GripDetected { get; private set; }

    private Rigidbody heldBallRb;
    private Collider heldBallCollider;
    private Transform heldBallOriginalParent;

    /// <summary>Command the claw to close and attempt to grip the ball.</summary>
    public void GripCommand()
    {
        isClawClosed = true;
    }

    /// <summary>Command the claw to open, releasing the ball if held.</summary>
    public void ReleaseCommand()
    {
        isClawClosed = false;

        if (heldBallRb != null)
        {
            ReleaseBall();
        }
    }

    private void Update()
    {
        // Обновляем состояние обнаружения мяча в клешне каждый кадр
        UpdateGripperDetection();

        // Пытаемся схватить мяч, если клешня закрыта и IR видит мяч
        if (isClawClosed && heldBallRb == null && GripDetected)
        {
            TryGripBall();
        }
    }

    /// <summary>
    /// Собственный Raycast для обнаружения мяча в клешне.
    /// Результат доступен через GripDetected — используется RobotBrain для наблюдения o04.
    /// </summary>
    private void UpdateGripperDetection()
    {
        if (gripperIRPoint == null)
        {
            GripDetected = false;
            return;
        }

        if (Physics.Raycast(gripperIRPoint.position, gripperIRPoint.forward, 
                            out RaycastHit hit, gripperRange))
        {
            GripDetected = hit.collider.CompareTag(ballTag);
        }
        else
        {
            GripDetected = false;
        }
    }

    /// <summary>
    /// Attempt to grip the ball if the claw is closed and IR detects the ball.
    /// Called from RobotBrain.OnActionReceived via GripCommand().
    /// </summary>
    private void TryGripBall()
    {
        if (holdPoint == null)
            return;

        // Проверяем, что клешня закрыта и мы ещё не держим мяч
        if (!isClawClosed || heldBallRb != null)
            return;

        // Проверяем, что IR обнаружил мяч (проверено в UpdateGripperDetection)
        if (!GripDetected)
            return;

        // --- Логика присоединения мяча ---
        // Ищем мяч через Physics.OverlapSphere в зоне клешни, так как Raycast
        // в UpdateGripperDetection мог попасть не точно в rigidbody.
        Collider[] hits = Physics.OverlapSphere(gripperIRPoint.position, gripperRange * 1.5f);
        
        foreach (var hit in hits)
        {
            if (!hit.CompareTag(ballTag))
                continue;

            Rigidbody ballRb = hit.attachedRigidbody;
            if (ballRb == null)
                continue;

            heldBallRb = ballRb;
            heldBallCollider = hit;
            heldBallOriginalParent = ballRb.transform.parent;

            heldBallRb.isKinematic = true;
            heldBallCollider.enabled = false;

            // worldPositionStays: true сохраняет мировой масштаб мяча
            ballRb.transform.SetParent(holdPoint, worldPositionStays: true);

            // Принудительно помещаем мяч в точку захвата
            ballRb.transform.position = holdPoint.position;
            ballRb.transform.rotation = holdPoint.rotation;

            return;
        }
    }

    /// <summary>
    /// Restores the held ball's physics and re-parents it back to the world.
    /// </summary>
    private void ReleaseBall()
    {
        if (heldBallRb == null) return;

        heldBallRb.transform.SetParent(heldBallOriginalParent);

        heldBallCollider.enabled = true;
        heldBallRb.isKinematic = false;

        heldBallRb = null;
        heldBallCollider = null;
        heldBallOriginalParent = null;
    }

    private void OnDrawGizmosSelected()
    {
        if (holdPoint == null) return;
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(holdPoint.position, grabSearchRadius);

        // Рисуем зону обнаружения мяча в клешне
        if (gripperIRPoint != null)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawRay(gripperIRPoint.position, gripperIRPoint.forward * gripperRange);
        }
    }
}