using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; // QuaternionMsg

/// <summary>
/// Practice 7 HIL: ROS subscriber for the real robot's IR/ultrasonic telemetry.
/// team1/unity_master_team1.py's sensor_timer_callback publishes /sensor/data every
/// 0.1s as a Quaternion (repurposed as a 4-float bundle, not an actual rotation):
///   x = ultrasonic distance in METERS (already filtered_cm / 100 on the robot side)
///   y = left obstacle IR   (0/1, already debounced on the robot side)
///   z = right obstacle IR  (0/1, already debounced on the robot side)
///   w = gripper IR         (0/1, already debounced on the robot side)
/// (team1/unity_gripper_ir_team1.py ALSO publishes gripper IR alone on
/// /sensor/gripper_ir, but that's the exact same GPIO pin (IR_M) read a second time -
/// /sensor/data's w already carries it, so subscribing to just this one topic is enough.)
///
/// Exposes the same shape as VirtualSensors (UltrasonicNorm/LeftIR/RightIR/GripperIR) so
/// RobotBrain can switch between simulated and real sensors - same "useReal" master-switch
/// pattern as RealVision, just over ROS instead of raw UDP.
/// </summary>
public class RealSensors : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/sensor/data";
    [Tooltip("Master switch: when ON, RobotBrain reads ultrasonic/IR from here instead of VirtualSensors")]
    public bool useReal = false;
    [Tooltip("If no packet arrives for this many seconds, sensors are treated as fail-safe defaults (ultrasonic 'clear', IR 'nothing detected') rather than freezing on the last known reading")]
    public float signalTimeout = 0.5f;
    [Tooltip("Matches VirtualSensors' ultrasonicMaxDistance - normalizes the real distance (already in meters) the same way the sim does: 0 = touching, 1 = clear")]
    public float ultrasonicMaxDistance = 2.0f;

    [Header("Telemetry (read-only)")]
    [SerializeField] private float ultrasonicNorm = 1f;
    [SerializeField] private int leftIR;
    [SerializeField] private int rightIR;
    [SerializeField] private int gripperIR;
    [SerializeField] private float lastPacketAge;

    public float UltrasonicNorm => ultrasonicNorm;
    public int LeftIR => leftIR;
    public int RightIR => rightIR;
    public int GripperIR => gripperIR;

    private ROSConnection ros;
    private float lastPacketTime = -999f;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<QuaternionMsg>(topicName, OnSensorData);
    }

    // ROS TCP Connector dispatches subscriber callbacks on the main thread (drained from
    // its internal queue during Update), so touching Unity state here directly is safe -
    // no ConcurrentQueue marshaling needed, unlike RealVision's raw UDP listener thread.
    private void OnSensorData(QuaternionMsg msg)
    {
        lastPacketTime = Time.time;
        float distanceMeters = (float)msg.x;
        ultrasonicNorm = Mathf.Clamp01(distanceMeters / ultrasonicMaxDistance);
        leftIR    = msg.y > 0.5 ? 1 : 0;
        rightIR   = msg.z > 0.5 ? 1 : 0;
        gripperIR = msg.w > 0.5 ? 1 : 0;
    }

    private void Update()
    {
        lastPacketAge = Time.time - lastPacketTime;
        if (lastPacketAge > signalTimeout)
        {
            // Watchdog: node crashed / WiFi dropped -> fail safe rather than trusting a
            // stale reading (a stuck "wall detected" IR would otherwise wedge the robot).
            ultrasonicNorm = 1f;
            leftIR = 0;
            rightIR = 0;
            gripperIR = 0;
        }
    }
}
