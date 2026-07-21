using UnityEngine;

/// <summary>
/// Attach directly to the trigger collider GameObject (e.g. "TriggerBamper") on the
/// robot's bumper. A script on RobotBrain's own root can't tell WHICH of the robot's
/// child colliders fired an OnTriggerEnter, so this lives on the bumper collider
/// itself and reports ball contact straight to RobotBrain.
/// </summary>
public class BumperSensor : MonoBehaviour
{
    [SerializeField] private string ballTag = "TargetBall";
    [Tooltip("Leave empty to auto-find the RobotBrain in a parent object")]
    [SerializeField] private RobotBrain brain;

    private void Awake()
    {
        if (brain == null) brain = GetComponentInParent<RobotBrain>();
    }

    // OnTriggerStay (not OnTriggerEnter) - fires every physics tick for as long as the
    // ball keeps overlapping this trigger, so the penalty accrues every step of contact,
    // not just once on the initial touch.
    private void OnTriggerStay(Collider other)
    {
        if (brain != null && other.CompareTag(ballTag))
            brain.OnBallHitBumper();
    }
}
