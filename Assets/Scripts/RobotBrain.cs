using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// Interceptor GFS-X AI agent. Glues the team scripts (TrackController,
/// VirtualSensors, SimulatedYoloCamera) into one "brain":
///   - collects 13 observation SLOTS, but all values are hardcoded to 0 (sanity-check
///     configuration - see CollectObservations),
///   - dispatches 3 continuous actions (no discrete action - the claw is autonomous,
///     driven directly by the GripperIR sensor, matching real hardware exactly),
///   - computes a single reward: gasReward * gas (rewards W/gas -> +1.0), plus flat
///     steerPenalty/cameraPenalty whenever steer/camCmd is nonzero (see ComputeRewards),
///   - exposes the public fields that BrainDebugHUD reads.
/// Inherits Agent (ML-Agents). Attach to the "robot" object.
///
/// Behavior Parameters on the robot MUST match:
///   Behavior Name = GFSX_Brain (same as config.yaml)
///   Vector Observation -> Space Size = 13, Stacked Vectors = 4
///   Continuous Actions = 3, Discrete Branches = 0
/// Leave the inspector "Max Step" = 0; this script caps the episode itself.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class RobotBrain : Agent
{
    [Header("Robot component references")]
    [SerializeField] private TrackController track;
    [SerializeField] private VirtualSensors sensors;
    [SerializeField] private SimulatedYoloCamera yolo;
    [Tooltip("Practice 7 (HIL): optional real-YOLO receiver. When assigned AND its useYOLO is ON, vision comes from the real robot instead of the simulated camera")]
    [SerializeField] private RealVision realVision;
    [Tooltip("Practice 7 (HIL): optional real ultrasonic/IR receiver (ROS /sensor/data). When assigned AND its useReal is ON, Obs01-04 come from the real robot instead of VirtualSensors' raycasts")]
    [SerializeField] private RealSensors realSensors;
    [Tooltip("Practice 7 (HIL): optional bridge to the real robot over ROS. When assigned, every action step's gas/steer/camera command is published over ROS - but only while NOT connected to the ML-Agents trainer (IsTraining == false), so training runs never spam ROS topics with a nonexistent robot listening. This is what lets driving in Unity (WASD via Heuristic, or a running inference policy) actually move the physical GFS-X.")]
    [SerializeField] private ROSBridge rosBridge;
    [Tooltip("Optional CSV telemetry logger (sim-to-real diagnostics). When assigned and its own enableLogging is ON, one row is appended every decision step")]
    [SerializeField] private DiagnosticLogger diagnosticLogger;

    [Header("Domain Randomization (optional)")]
    [Tooltip("Optional DomainRandomizer component. Leave empty to train on a clean environment")]
    [SerializeField] private DomainRandomizer randomizer;
    [Tooltip("Optional ObstacleRandomizer: scatters the obstacle belt every episode (competition rule). Leave empty for a fixed layout")]
    [SerializeField] private ObstacleRandomizer obstacleRandomizer;

    [Header("Camera servo")]
    [Tooltip("Transform rotated by the camera command (usually the camera object / pan joint)")]
    [SerializeField] private Transform cameraServo;
    [SerializeField] private float maxServoAngle = 90f;   // +/-90 deg
    [SerializeField] private float servoSpeed = 90f;      // deg/s

    [Header("Target and arena")]
    [SerializeField] private Transform ball;
    [SerializeField] private string ballTag = "TargetBall";
    [Tooltip("The claw's HoldPoint on the robot prefab - the ground-truth ball-distance meta reward/observation is measured from here, not from the robot's root/model center. Auto-found by name if left empty")]
    [SerializeField] private Transform holdPoint;
    [Tooltip("Optional arena center. If empty, world origin (0,0,0) is used")]
    [SerializeField] private Transform arenaCenter;

    [Header("Ball spawn zone (rectangle centered on Finish - independent of the robot)")]
    [Tooltip("The ball always spawns at a random point in a rectangle centered on THIS Transform's position, using its own local right/forward axes. Required for ball spawning - there is no robot-relative fallback anymore")]
    [SerializeField] private Transform finishArea;
    [Tooltip("Half-width (radius) of the ball spawn rectangle along Finish's local RIGHT axis: spawn range on that axis = center +- this")]
    [SerializeField] private float ballSpawnHalfWidth = 1.5f;
    [Tooltip("Full length of the ball spawn rectangle along Finish's local FORWARD axis, centered on Finish: spawn range on that axis = center +- length/2")]
    [SerializeField] private float ballSpawnLength = 3f;

    [Header("Robot spawn zone (rectangle centered on an explicit spawn point)")]
    [Tooltip("Explicit reference point/orientation the robot spawn rectangle is measured from. If empty, the scene-start pose is used")]
    [SerializeField] private Transform spawnPoint;
    [Tooltip("Half-width (radius) of the robot spawn rectangle along spawnPoint's local RIGHT axis: spawn range on that axis = center +- this")]
    [SerializeField] private float robotSpawnHalfWidth = 0.5f;
    [Tooltip("Full length of the robot spawn rectangle along spawnPoint's local FORWARD axis, centered on spawnPoint: spawn range on that axis = center +- length/2")]
    [SerializeField] private float robotSpawnLength = 0f;
    [Tooltip("Draw the ball/robot spawn rectangles as gizmos in the Scene view (yellow = ball around Finish, cyan = robot around spawnPoint)")]
    [SerializeField] private bool drawSpawnGizmos = true;

    [Header("Episode")]
    [Tooltip("Hard cap on decision steps per episode. Guarantees the episode always resets.")]
    [SerializeField] private int maxEpisodeSteps = 4000;
    [Tooltip("MISSION MODE (Practice 7): for inference demos with the state machine. Disables all teleports/EndEpisode - the FSM owns the mission cycle. MUST be OFF for training!")]
    [SerializeField] private bool missionMode = false;

    [Header("Rewards (sanity-check configuration: single reward + two flat penalties)")]
    [Tooltip("Reward = gasReward * gas. Maximal at gas == +1.0 (full W/forward), zero at gas == 0, negative if gas is negative (reverse)")]
    [SerializeField] private float gasReward = 1f;
    [Tooltip("Flat penalty applied whenever |steer| is not ~0")]
    [SerializeField] private float steerPenalty = 1f;
    [Tooltip("Flat penalty applied whenever |camCmd| (camera command) is not ~0")]
    [SerializeField] private float cameraPenalty = 1f;

    [Header("Debug - live reward breakdown (read-only, updates every step in Play mode)")]
    [SerializeField] private float dbg_GasApplied;
    [SerializeField] private float dbg_SteerPenaltyApplied;
    [SerializeField] private float dbg_CameraPenaltyApplied;

    // ---- Rigidbody / start poses ----
    private Rigidbody rb;
    private Vector3 startPos;
    private Quaternion startRot;
    private Quaternion cameraServoBase;
    private float ballStartY;

    // ---- internal state ----
    private float servoAngle;
    private int episodeSteps;

    // ---- DiagnosticLogger ground-truth tracking (sim-to-real telemetry only,
    // never fed into observations/rewards) ----
    private Vector3 episodeStartPos;
    private Vector3 lastLogPos;
    private float lastLogTime;

    /// <summary>True when connected to the Python trainer (used later for domain randomization / ROSBridge gating).</summary>
    public bool IsTraining => Academy.Instance.IsCommunicatorOn;

    // ============ PUBLIC API FOR BrainDebugHUD ============
    public float Obs01_Ultrasonic        { get; private set; }
    public int   Obs02_LeftIR            { get; private set; }
    public int   Obs03_RightIR           { get; private set; }
    public int   Obs04_GripperIR         { get; private set; }
    public float Obs05_BallAngle         { get; private set; }
    public float Obs06_BallDistance      { get; private set; }
    public float Obs07_LastKnownAngle    { get; private set; }
    public float Obs08_BallVisible       { get; private set; }
    public float Obs09_ServoAngleNorm    { get; private set; }
    public float Obs10_HasBall           { get; private set; }
    public float Obs11_TimeSinceBallNorm { get; private set; }
    public float Obs12_GroundTruthBallDistance { get; private set; }
    public float Obs13_GroundTruthBallAngle { get; private set; }

    public float ActGas       { get; private set; }
    public float ActSteer     { get; private set; }
    public float ActCameraCmd { get; private set; }

    public float StepReward       { get; private set; }
    public float CumulativeReward => GetCumulativeReward();
    // =====================================================

    private Vector3 ArenaCenter => arenaCenter != null ? arenaCenter.position : Vector3.zero;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();

        // Practice 7 HIL: auto-find ROSBridge/RealVision if not wired up in the Inspector
        // (e.g. dropped a component in via Add Component instead of dragging a reference).
        // Checks this GameObject first, then children, then anywhere in the scene.
        if (rosBridge == null) rosBridge = GetComponent<ROSBridge>();
        if (rosBridge == null) rosBridge = GetComponentInChildren<ROSBridge>(true);
        if (rosBridge == null) rosBridge = FindFirstObjectByType<ROSBridge>();

        if (realVision == null) realVision = GetComponent<RealVision>();
        if (realVision == null) realVision = GetComponentInChildren<RealVision>(true);
        if (realVision == null) realVision = FindFirstObjectByType<RealVision>();

        if (realSensors == null) realSensors = GetComponent<RealSensors>();
        if (realSensors == null) realSensors = GetComponentInChildren<RealSensors>(true);
        if (realSensors == null) realSensors = FindFirstObjectByType<RealSensors>();

        if (diagnosticLogger == null) diagnosticLogger = GetComponent<DiagnosticLogger>();
        if (diagnosticLogger == null) diagnosticLogger = GetComponentInChildren<DiagnosticLogger>(true);
        if (diagnosticLogger == null) diagnosticLogger = FindFirstObjectByType<DiagnosticLogger>();

        if (holdPoint == null) holdPoint = FindDeep(transform, "HoldPoint");

        startPos = spawnPoint != null ? spawnPoint.position : transform.position;
        startRot = spawnPoint != null ? spawnPoint.rotation : transform.rotation;

        if (cameraServo != null) cameraServoBase = cameraServo.localRotation;

        if (ball == null && !string.IsNullOrEmpty(ballTag))
        {
            var go = GameObject.FindWithTag(ballTag);
            if (go != null) ball = go.transform;
        }
        if (ball != null) ballStartY = ball.position.y;
    }

    public override void OnEpisodeBegin()
    {
        episodeSteps = 0;

        // MISSION MODE: the state machine owns the world - no teleports,
        // no ball respawn, no obstacle shuffle. Only internal counters reset.
        if (missionMode)
        {
            episodeStartPos = transform.position;
            lastLogPos = transform.position;
            lastLogTime = Time.time;
            return;
        }

        // Competition rule: scatter the obstacle belt BEFORE spawning the ball,
        // so the ball's overlap check sees the new obstacle positions
        if (obstacleRandomizer != null) obstacleRandomizer.Shuffle();

        // Reset robot: random point in a rectangle centered on startPos/startRot (from
        // spawnPoint - see Initialize()), using its own local right/forward axes.
        // Teleport through the rigidbody so PhysX state matches the new pose.
        rb.linearVelocity = Vector3.zero;   // Unity < 6: rb.velocity
        rb.angularVelocity = Vector3.zero;
        Vector3 spawn = startPos
            + (startRot * Vector3.right)   * Random.Range(-robotSpawnHalfWidth, robotSpawnHalfWidth)
            + (startRot * Vector3.forward) * Random.Range(-robotSpawnLength * 0.5f, robotSpawnLength * 0.5f);
        rb.position = spawn;
        rb.rotation = startRot;
        transform.SetPositionAndRotation(spawn, startRot);

        episodeStartPos = spawn;
        lastLogPos = spawn;
        lastLogTime = Time.time;

        // Reset camera servo
        servoAngle = 0f;
        if (cameraServo != null) cameraServo.localRotation = cameraServoBase;

        // Ball spawn: random point in a rectangle centered on the "Finish" Transform's
        // OWN position, using ITS OWN local right/forward axes - entirely independent of
        // the robot (no more forward/side band relative to the robot's start pose).
        if (ball != null && finishArea != null)
        {
            Vector3 center = finishArea.position;
            Vector3 side   = finishArea.right;
            Vector3 fwd    = finishArea.forward;
            float ballR = ball.localScale.x * 0.5f + 0.02f;

            // Kept only to exclude the Finish floor marker's own trigger collider from the
            // "solid" check below - its bounds are no longer used for spawn positioning.
            Collider finishCollider = finishArea.GetComponent<Collider>();

            Vector3 p = ball.position; int guard = 0;
            bool ok = false;
            while (!ok && guard < 40)
            {
                guard++;

                p = center
                  + side * Random.Range(-ballSpawnHalfWidth, ballSpawnHalfWidth)
                  + fwd  * Random.Range(-ballSpawnLength * 0.5f, ballSpawnLength * 0.5f);
                p.y = ballStartY;
                // No arena-bounds clamp here anymore - physical walls contain the
                // arena, so an artificial software boundary is redundant.

                // not inside anything solid: allow only the floor (arenaCenter), the
                // finish marker itself, and the ball itself
                ok = true;
                foreach (var col in Physics.OverlapSphere(p, ballR))
                {
                    if (col.transform == ball) continue;                              // the ball itself
                    if (finishCollider != null && col == finishCollider) continue;     // the finish floor marker
                    if (arenaCenter != null &&
                        (col.transform == arenaCenter || col.transform.IsChildOf(arenaCenter)))
                        continue;                                                     // the floor
                    ok = false; break;                                                // wall / obstacle / robot
                }
            }

            ball.position = p;
            var brb = ball.GetComponent<Rigidbody>();
            if (brb != null) { brb.isKinematic = false; brb.linearVelocity = Vector3.zero; brb.angularVelocity = Vector3.zero; }
            var bcol = ball.GetComponent<Collider>();
            if (bcol != null) bcol.enabled = true;
        }

        // Domain Randomization (Practice 5): physics + latency queue reset
        if (randomizer != null) randomizer.ApplyEpisodeRandomization(IsTraining);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Sanity-check configuration: every observation is hardcoded to 0, regardless of
        // any real sensor/vision/ground-truth state. The properties still exist (and are
        // still fed to the sensor, matching Space Size = 13) purely so BrainDebugHUD and
        // DiagnosticLogger keep compiling/working - they'll just always read 0.
        Obs01_Ultrasonic = 0f;
        Obs02_LeftIR = 0;
        Obs03_RightIR = 0;
        Obs04_GripperIR = 0;
        Obs05_BallAngle = 0f;
        Obs06_BallDistance = 0f;
        Obs07_LastKnownAngle = 0f;
        Obs08_BallVisible = 0f;
        Obs09_ServoAngleNorm = 0f;
        Obs10_HasBall = 0f;
        Obs11_TimeSinceBallNorm = 0f;
        Obs12_GroundTruthBallDistance = 0f;
        Obs13_GroundTruthBallAngle = 0f;

        sensor.AddObservation(Obs01_Ultrasonic);
        sensor.AddObservation(Obs02_LeftIR);
        sensor.AddObservation(Obs03_RightIR);
        sensor.AddObservation(Obs04_GripperIR);
        sensor.AddObservation(Obs05_BallAngle);
        sensor.AddObservation(Obs06_BallDistance);
        sensor.AddObservation(Obs07_LastKnownAngle);
        sensor.AddObservation(Obs08_BallVisible);
        sensor.AddObservation(Obs09_ServoAngleNorm);
        sensor.AddObservation(Obs10_HasBall);
        sensor.AddObservation(Obs11_TimeSinceBallNorm);
        sensor.AddObservation(Obs12_GroundTruthBallDistance);
        sensor.AddObservation(Obs13_GroundTruthBallAngle);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        StepReward = 0f;
        episodeSteps++;

        float gas    = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        // Steering and camera pan are hard-disabled - forced to 0 regardless of what the
        // policy outputs, not just penalized. The robot physically cannot turn or pan the
        // camera in this sanity-check configuration.
        float steer  = 0f;
        float camCmd = 0f;

        ActGas = gas; ActSteer = steer; ActCameraCmd = camCmd;

        // Domain Randomization: action latency FIFO (no-op when latency is 0)
        if (randomizer != null) randomizer.DelayActions(ref gas, ref steer, ref camCmd);

        // Actuation gate: during actual mlagents-learn training (IsTraining) actions are
        // ALWAYS applied - this must never block the RL training loop. Outside training
        // (inference/real robot), actions are held back until missionMode is turned on -
        // normally that's the Inspector checkbox, but it's also exactly what
        // RobotBrain.StartMission() flips (called by e.g. MissionHttpTrigger on an
        // external "go" signal). Lets the robot sit motionless at a start line instead of
        // immediately driving off the moment the model starts running inference.
        bool actuationAllowed = IsTraining || missionMode;

        if (actuationAllowed)
        {
            // Drive (SetCommand clamps to [-1,1] itself)
            if (track != null) track.SetCommand(gas, steer);

            // ROS bridge (Practice 7 HIL): forward the same command to the real robot.
            // Gated on !IsTraining - during mlagents-learn training there is no physical
            // robot listening, so publishing would just spam the topics for nothing.
            if (rosBridge != null && !IsTraining)
            {
                rosBridge.PublishCommand(gas, steer);
            }

            // Camera pan: rotate around the WORLD vertical axis (robust to any
            // tilted parent hierarchy in the imported model - guarantees the pan
            // is left/right, never up/down)
            servoAngle = Mathf.Clamp(servoAngle + camCmd * servoSpeed * Time.fixedDeltaTime,
                                     -maxServoAngle, maxServoAngle);
            if (cameraServo != null)
                cameraServo.localRotation = cameraServoBase *
                    Quaternion.AngleAxis(servoAngle,
                        cameraServo.parent != null
                            ? cameraServo.parent.InverseTransformDirection(Vector3.up)
                            : Vector3.up);

            // ROSBridge.PublishCameraCmd must carry the ABSOLUTE target pan position,
            // normalized to [-1, 1] - this is what the robot's own camera_callback expects
            // (team1/unity_master_team1.py: "target = 90 - (yaw * 90)", i.e. yaw=-1 -> 180 deg,
            // yaw=0 -> 90 deg (center), yaw=+1 -> 0 deg). Sending the raw per-tick camCmd
            // (-1/0/+1, a RATE) made the servo barely twitch since it re-reads as "go to
            // (roughly) 0 or 180 every tick" instead of a smooth target; sending servoAngle
            // in raw degrees (previous attempt) overflowed the same formula and pinned the
            // servo at 0 or 180. Dividing by maxServoAngle converts the already-accumulated
            // absolute angle to the [-1, 1] range the robot-side formula actually expects.
            if (rosBridge != null && !IsTraining)
            {
                float normalizedServoAngle = maxServoAngle > 0f ? servoAngle / maxServoAngle : 0f;
                rosBridge.PublishCameraCmd(normalizedServoAngle);
            }
        }

        ComputeRewards(gas, steer, camCmd);
    }

    /// <summary>
    /// External trigger (e.g. MissionHttpTrigger, on an HTTP "go" signal) to start
    /// actually applying inferred actions during inference/real-robot runs. Equivalent to
    /// switching the Inspector's Mission Mode checkbox on at runtime. No effect during
    /// actual mlagents-learn training - actions are already applied unconditionally then.
    /// </summary>
    public void StartMission()
    {
        missionMode = true;
    }

    private void ComputeRewards(float gas, float steer, float camCmd)
    {
        // ---- Diagnostic telemetry (sim-to-real CSV log) - unrelated to the reward system,
        // left running as-is. One row per decision step (all-zero sensor fields, since
        // observations are hardcoded to 0 - see CollectObservations).
        if (diagnosticLogger != null)
        {
            float dt = Time.time - lastLogTime;
            float speed = dt > 0.0001f ? Vector3.Distance(transform.position, lastLogPos) / dt : 0f;
            diagnosticLogger.LogStep(
                episodeSteps, Obs08_BallVisible > 0.5f, Obs05_BallAngle, Obs06_BallDistance,
                Obs01_Ultrasonic, Obs02_LeftIR, Obs03_RightIR, Obs04_GripperIR, servoAngle,
                gas, steer, Obs10_HasBall > 0.5f, 0, false,
                transform.position.x - episodeStartPos.x, transform.position.z - episodeStartPos.z,
                transform.eulerAngles.y, speed);
            lastLogPos = transform.position;
            lastLogTime = Time.time;
        }

        // --- Sole reward: gas toward +1.0 (W = full forward). Proportional, so it scales
        // down as gas drops from 1, hits 0 at gas == 0, and goes negative for reverse. ---
        float gasR = gasReward * gas;
        Add(gasR);
        dbg_GasApplied = gasR;

        // --- Flat penalty whenever steer is not ~0 ---
        if (Mathf.Abs(steer) > 0.0001f)
        {
            Add(-steerPenalty);
            dbg_SteerPenaltyApplied = -steerPenalty;
        }
        else
        {
            dbg_SteerPenaltyApplied = 0f;
        }

        // --- Flat penalty whenever the camera command is not ~0 ---
        if (Mathf.Abs(camCmd) > 0.0001f)
        {
            Add(-cameraPenalty);
            dbg_CameraPenaltyApplied = -cameraPenalty;
        }
        else
        {
            dbg_CameraPenaltyApplied = 0f;
        }

        // Mission mode: the FSM owns the mission - never terminate episodes.
        if (missionMode)
        {
            RecordRewardStats();
            return;
        }

        // Structural episode boundary only - no reward/penalty attached, just keeps
        // training moving through multiple episodes instead of one endless rollout.
        if (episodeSteps >= maxEpisodeSteps)
        {
            EndEpisode();
        }

        RecordRewardStats();
    }

    /// <summary>
    /// Pushes every reward-breakdown debug field to ML-Agents' StatsRecorder, so each
    /// component shows up as its own chart in TensorBoard (nested under "Rewards/") instead
    /// of only being visible live in the Inspector during Play mode. Safe to call regardless
    /// of whether a trainer is attached - StatsRecorder just has nowhere to send stats to
    /// when not training, it doesn't throw.
    /// </summary>
    private void RecordRewardStats()
    {
        var stats = Academy.Instance.StatsRecorder;
        stats.Add("Rewards/Gas", dbg_GasApplied);
        stats.Add("Rewards/SteerPenalty", dbg_SteerPenaltyApplied);
        stats.Add("Rewards/CameraPenalty", dbg_CameraPenaltyApplied);
    }

    /// <summary>
    /// Called by BumperSensor (attached to the TriggerBamper collider on the robot's
    /// bumper) whenever the ball touches it. No-op in this sanity-check configuration -
    /// kept only so BumperSensor's existing reference keeps compiling.
    /// </summary>
    public void OnBallHitBumper()
    {
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        // GetAxis (smoothed), not GetAxisRaw: raw instant -1/0/+1 made manual keyboard
        // driving violently bang-bang (visibly shaking left-right from the robot's own
        // camera). This only affects Heuristic/manual testing - during actual training the
        // policy's continuous outputs never go through Input Manager at all, so the sudden-
        // move detection still works correctly for real training regardless of this.
        cont[0] = Input.GetAxis("Vertical");     // W/S -> gas
        cont[1] = Input.GetAxis("Horizontal");   // A/D -> steer
        cont[2] = (Input.GetKey(KeyCode.E) ? 1f : 0f) - (Input.GetKey(KeyCode.Q) ? 1f : 0f); // Q/E -> camera
    }

    // ---- helpers ----
    private void Add(float r) { AddReward(r); StepReward += r; }

    /// <summary>Recursive child search by name (Transform.Find only checks direct children).</summary>
    private static Transform FindDeep(Transform root, string name)
    {
        if (root.name == name) return root;
        foreach (Transform child in root)
        {
            Transform found = FindDeep(child, name);
            if (found != null) return found;
        }
        return null;
    }

    // ---- Editor visualization ----
    private void OnDrawGizmos()
    {
        if (!drawSpawnGizmos) return;

        // Ball spawn rectangle: centered on Finish's own position/axes.
        DrawSpawnRect(finishArea, ballSpawnHalfWidth, ballSpawnLength, Color.yellow);

        // Robot spawn rectangle: centered on spawnPoint, or this transform if unset -
        // mirrors the startPos/startRot fallback in Initialize() so the gizmo matches
        // what actually happens in Play mode.
        Transform robotRef = spawnPoint != null ? spawnPoint : transform;
        DrawSpawnRect(robotRef, robotSpawnHalfWidth, robotSpawnLength, Color.cyan);
    }

    private static void DrawSpawnRect(Transform reference, float halfWidth, float length, Color color)
    {
        if (reference == null) return;

        Vector3 center = reference.position;
        Vector3 halfSide = reference.right * halfWidth;
        Vector3 halfFwd = reference.forward * (length * 0.5f);

        Vector3 p1 = center + halfSide + halfFwd;
        Vector3 p2 = center - halfSide + halfFwd;
        Vector3 p3 = center - halfSide - halfFwd;
        Vector3 p4 = center + halfSide - halfFwd;

        Gizmos.color = color;
        Gizmos.DrawLine(p1, p2);
        Gizmos.DrawLine(p2, p3);
        Gizmos.DrawLine(p3, p4);
        Gizmos.DrawLine(p4, p1);
        Gizmos.DrawLine(center - halfFwd, center + halfFwd); // center line - marks the "forward" axis
    }
}
