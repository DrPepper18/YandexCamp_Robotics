using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

/// <summary>
/// Interceptor GFS-X AI agent. Glues the team scripts (TrackController,
/// VirtualSensors, SimulatedYoloCamera) into one "brain":
///   - collects 13 observations (order strictly matches the Practice 3 table,
///     minus odometry/heading/speed - the real GFS-X has none of those sensors -
///     plus a 12th, toggleable, ground-truth ball distance (see
///     gtBallDistanceObsEnabled) and a 13th, ground-truth ball angle - neither has
///     a real-hardware equivalent, both are sim-only meta information),
///   - dispatches 3 continuous actions (no discrete action - the claw is autonomous,
///     driven directly by the GripperIR sensor, matching real hardware exactly),
///   - computes bounded rewards (all fixed amounts, no multipliers/percentages),
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

    [Header("Rewards (all fixed amounts - no multipliers/percentages anywhere)")]
    [Tooltip("Flat penalty per step while standing still (|gas| ~ 0). Moving forward alone gives nothing (0).")]
    [SerializeField] private float standingStillPenalty = 0.001f;
    [Tooltip("ballDistanceReward * (1-|BallDistance|), rewards being close to a VISIBLE ball regardless of whether this step happened to close distance")]
    [SerializeField] private float ballDistanceReward = 0.1f;
    [Tooltip("Meta-reward, ground truth (not fed into observations - this branch stays at 11 obs). While the robot's true distance to the ball is actually shrinking vs the last decision, reward = gtBallApproachReward * (prevDist - currDist). NOT gated by camera visibility - gives a navigation gradient even while the ball is out of view. See gtBallRetreatPenalty for the opposite case")]
    [SerializeField] private float gtBallApproachReward = 0.001f;
    [Tooltip("Meta-reward, ground truth: flat penalty applied instead of gtBallApproachReward when the robot's true distance to the ball increased since the last decision (moving away). Independently tunable from gtBallApproachReward - not just its negative")]
    [SerializeField] private float gtBallRetreatPenalty = 0.001f;
    [Tooltip("Normalization range (meters) for the gtBallApproachReward/gtBallRetreatPenalty progress term AND for the 12th observation (Obs12_GroundTruthBallDistance, see gtBallDistanceObsEnabled)")]
    [SerializeField] private float groundTruthBallDistanceMaxRange = 5f;
    [Tooltip("Meta-reward, ground truth: rewards the ROBOT BODY facing the ball directly (Obs13_GroundTruthBallAngle, 0 = facing it dead-on), regardless of camera/servo alignment or visibility. Same formula shape as centeringRewardScale above: gtCenteringReward * (1-|angle|)^3, always active - NOT gated by ballVisible")]
    [SerializeField] private float gtCenteringReward = 0.001f;
    [Tooltip("Scale for centeringRewardScale * (1-|ServoAngle|) * (1-|BallAngle|), only while the ball is visible")]
    [SerializeField] private float centeringRewardScale = 0.001f;
    [Tooltip("Flat penalty per step for driving backwards while the ball IS visible")]
    [SerializeField] private float backwardVisiblePenalty = 0.001f;
    [Tooltip("Flat penalty per step for driving backwards while the ball is NOT visible")]
    [SerializeField] private float backwardBlindPenalty = 0.0001f;
    [Tooltip("Flat penalty for reversing gas/steer/camera direction (not just a large step - see suddenMoveSignificance)")]
    [SerializeField] private float suddenMovePenalty = 0.001f;
    [Tooltip("Minimum |value| for gas/steer/camCmd to count as a deliberate direction. Kept low so the SMOOTHED steer/gas (Input.GetAxis, or a smooth policy) still registers - a high threshold widens the near-zero dead band the ramp must cross, which the oscillation window then can't span")]
    [SerializeField] private float suddenMoveSignificance = 0.2f;
    [Tooltip("A direction reversal only counts as 'sudden' if the opposite significant reading appears within this many decisions of the last one. Must exceed how long a smoothed steer/gas takes to ramp across the dead band (else motor reversals never register), yet stay well under the straight-driving gap between two genuinely separate turns")]
    [SerializeField] private int oscillationWindowDecisions = 12;
    [Tooltip("Flat penalty when left/right IR is active (wall close on that side)")]
    [SerializeField] private float sideIRCriticalPenalty = 50f;
    [Tooltip("Flat penalty when the front ultrasonic reads under ultrasonicCriticalThreshold")]
    [SerializeField] private float ultrasonicCriticalPenalty = 50f;
    [SerializeField] private float ultrasonicCriticalThreshold = 0.2f;

    [Header("Claw hold / success (based on the raw GripperIR sensor - matches real hardware, not a physics-only 'IsHolding' state)")]
    [Tooltip("Decisions per reward tick. 5 decisions = 0.5 s at Decision Period 5 / 50 Hz physics")]
    [SerializeField] private int gripRewardIntervalDecisions = 5;
    [Tooltip("Decisions to hold before success (episode ends). 20 decisions = ~2 s at Decision Period 5 / 50 Hz physics (matches the real gripper closing time)")]
    [SerializeField] private int gripHoldMaxDecisions = 20;
    [Tooltip("Flat reward granted every gripRewardIntervalDecisions while the claw IR sensor stays continuously active")]
    [SerializeField] private float gripHoldTickBonus = 50f;
    [Tooltip("Flat penalty when the claw IR sensor was active and then deactivates before reaching gripHoldMaxDecisions (ball touched the claw and was lost)")]
    [SerializeField] private float gripLostPenalty = 50f;
    [Tooltip("Flat bonus on top of everything else when the episode ends via a SUCCESSFUL hold (not a timeout)")]
    [SerializeField] private float successEpisodeReward = 20f;
    [Tooltip("Flat penalty when the episode ends because time ran out without ever completing the claw hold (never found/caught the ball in time)")]
    [SerializeField] private float timeoutPenalty = 20f;
    [Tooltip("Flat penalty each time the ball touches the TriggerBamper collider (reported by BumperSensor)")]
    [SerializeField] private float ballBumperPenalty = 1f;

    [Header("Debug - live reward breakdown (read-only, updates every step in Play mode)")]
    [SerializeField] private float dbg_StandingStillApplied;
    [SerializeField] private float dbg_BallDistanceApplied;
    [SerializeField] private float dbg_GTBallApproachApplied;
    [SerializeField] private float dbg_GTCenteringApplied;
    [SerializeField] private float dbg_CenteringApplied;
    [SerializeField] private float dbg_BackwardApplied;
    [SerializeField] private float dbg_SideIRCriticalApplied;
    [SerializeField] private float dbg_UltrasonicCriticalApplied;
    [SerializeField] private float dbg_SuddenMoveApplied;
    [SerializeField] private float dbg_GripHoldBonusApplied;
    [SerializeField] private float dbg_GripLostApplied;
    [SerializeField] private float dbg_SuccessEpisodeApplied;
    [SerializeField] private float dbg_TimeoutPenaltyApplied;
    [SerializeField] private float dbg_BallBumperApplied;

    [Header("Observation normalization")]
    [SerializeField] private float timeSinceBallCap = 10f;  // s
    [Tooltip("12th observation: ground truth (ODOMETRY - no real-hardware equivalent) distance to the ball, Clamp01(distance/groundTruthBallDistanceMaxRange). NOT gated by camera visibility, unlike Obs06_BallDistance. Toggle off to fall back to a neutral 1 (far) placeholder - the observation slot always exists (Space Size stays 12) either way, only its content changes")]
    [SerializeField] private bool gtBallDistanceObsEnabled = true;

    // ---- Rigidbody / start poses ----
    private Rigidbody rb;
    private Vector3 startPos;
    private Quaternion startRot;
    private Quaternion cameraServoBase;
    private float ballStartY;

    // ---- internal state ----
    private float servoAngle;
    private float lastKnownBallAngle;
    private float timeSinceBall;
    // Last SIGNIFICANT (|value| > suddenMoveSignificance) direction seen for each channel
    // (-1, +1, or 0 = none recorded yet), and how many decisions ago that was. A reversal
    // only counts as "sudden" if the opposite significant direction reappears within
    // oscillationWindowDecisions - otherwise it's a normal, well-separated turn (e.g. went
    // right a while ago, now going left to navigate), not left-right-left-right jitter.
    private float lastSigGas, lastSigSteer, lastSigCam;
    private int gasSigAge = 999999, steerSigAge = 999999, camSigAge = 999999;
    private int episodeSteps;
    private int gripHoldDecisions;
    private int gripRewardTicksGranted;

    // ---- DiagnosticLogger ground-truth tracking (sim-to-real telemetry only,
    // never fed into observations/rewards) ----
    private Vector3 episodeStartPos;
    private Vector3 lastLogPos;
    private float lastLogTime;

    // GT ball distance (unclamped, normalized by groundTruthBallDistanceMaxRange) as of the
    // previous DECISION, purely for the gtBallApproachReward/gtBallRetreatPenalty meta-reward -
    // never fed into observations. -1 = no baseline yet (first decision after reset).
    // pendingGtBallReward is staged once per decision in CollectObservations (the only method
    // that runs exactly once per real ML-Agents decision - OnActionReceived/ComputeRewards
    // fires every physics tick under TakeActionsBetweenDecisions, which would otherwise
    // compare a stale distance against itself on the ticks in between) and consumed exactly
    // once in ComputeRewards.
    private float prevGtBallDistRaw = -1f;
    private float pendingGtBallReward;

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
            lastKnownBallAngle = 0f;
            timeSinceBall = timeSinceBallCap;
            lastSigGas = lastSigSteer = lastSigCam = 0f;
            gasSigAge = steerSigAge = camSigAge = 999999;
            gripHoldDecisions = 0;
            gripRewardTicksGranted = 0;
            episodeStartPos = transform.position;
            lastLogPos = transform.position;
            lastLogTime = Time.time;
            prevGtBallDistRaw = -1f;
            pendingGtBallReward = 0f;
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
        prevGtBallDistRaw = -1f;
        pendingGtBallReward = 0f;

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

        // Reset internal state
        lastKnownBallAngle = 0f;
        timeSinceBall = timeSinceBallCap;
        lastSigGas = lastSigSteer = lastSigCam = 0f;
        gasSigAge = steerSigAge = camSigAge = 999999;
        gripHoldDecisions = 0;
        gripRewardTicksGranted = 0;

        // Domain Randomization (Practice 5): physics + latency queue reset
        if (randomizer != null) randomizer.ApplyEpisodeRandomization(IsTraining);
    }

    private void FixedUpdate()
    {
        // Grows continuously; reset in CollectObservations when the ball is visible
        timeSinceBall += Time.fixedDeltaTime;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1-4. Ultrasonic + IR. Source: real robot (Practice 7 HIL, ROS /sensor/data)
        // when RealSensors is assigned and switched on, otherwise VirtualSensors' raycasts.
        bool useRealSensors = realSensors != null && realSensors.useReal;

        // 1. Ultrasonic: take the nearer side (0 = touching .. 1 = clear)
        Obs01_Ultrasonic = useRealSensors
            ? realSensors.UltrasonicNorm
            : (sensors != null ? Mathf.Min(sensors.UltrasonicLeft, sensors.UltrasonicRight) : 1f);
        if (randomizer != null)
            Obs01_Ultrasonic = randomizer.NoisySonar(Obs01_Ultrasonic, IsTraining);

        // 2-4. IR
        Obs02_LeftIR    = useRealSensors ? realSensors.LeftIR    : (sensors != null ? sensors.LeftIR    : 0);
        Obs03_RightIR   = useRealSensors ? realSensors.RightIR   : (sensors != null ? sensors.RightIR   : 0);
        Obs04_GripperIR = useRealSensors ? realSensors.GripperIR : (sensors != null ? sensors.GripperIR : 0);

        // 5-8. Camera / YOLO. Source: real robot (Practice 7 HIL) when RealVision
        // is assigned and switched on, otherwise the simulated camera.
        bool useReal = realVision != null && realVision.useYOLO;
        bool visible = useReal ? realVision.IsVisible
                               : (yolo != null && yolo.IsVisible);
        float visAngle = useReal ? realVision.RelativeAngle
                                 : (yolo != null ? yolo.RelativeAngle : 0f);
        float visDist  = useReal ? realVision.NormalizedDistance
                                 : (yolo != null ? yolo.NormalizedDistance : 1f);
        if (randomizer != null && rb != null)
            visible = randomizer.FilterBallVisibility(visible, rb.angularVelocity.magnitude, IsTraining);
        Obs08_BallVisible  = visible ? 1f : 0f;
        Obs05_BallAngle    = visible ? visAngle : 0f;
        Obs06_BallDistance = visible ? visDist : 1f;
        if (visible) { lastKnownBallAngle = visAngle; timeSinceBall = 0f; }
        Obs07_LastKnownAngle = lastKnownBallAngle;

        // 9. Camera servo
        Obs09_ServoAngleNorm = Mathf.Clamp(servoAngle / maxServoAngle, -1f, 1f);

        // 10. Has ball: mirrors the raw GripperIR sensor directly (not the physics-only
        // "IsHolding" state) - the real robot only ever has the raw sensor to go on.
        Obs10_HasBall = Obs04_GripperIR == 1 ? 1f : 0f;

        // 11. Time since last detection
        Obs11_TimeSinceBallNorm = Mathf.Clamp01(timeSinceBall / timeSinceBallCap);

        // Ground truth ball distance: also the basis of the gtBallApproachReward/
        // gtBallRetreatPenalty meta-reward below, computed HERE (not in ComputeRewards)
        // because CollectObservations is the only method that runs exactly once per real
        // ML-Agents DECISION. With DecisionPeriod > 1 and TakeActionsBetweenDecisions,
        // OnActionReceived/ComputeRewards fires every physics tick, re-using the same
        // cached values on the ticks in between real decisions - comparing distance against
        // itself on those ticks would make this fire in an erratic, mostly-empty pattern.
        float gtRawNorm;
        if (ball != null)
        {
            Vector3 a = holdPoint != null ? holdPoint.position : transform.position; a.y = 0f;
            Vector3 b = ball.position;       b.y = 0f;
            gtRawNorm = Vector3.Distance(a, b) / groundTruthBallDistanceMaxRange;
        }
        else
        {
            gtRawNorm = 1f;
        }
        if (prevGtBallDistRaw >= 0f)
        {
            float delta = prevGtBallDistRaw - gtRawNorm;
            if (delta > 0f)      pendingGtBallReward = gtBallApproachReward * delta;   // approaching
            else if (delta < 0f) pendingGtBallReward = -gtBallRetreatPenalty;          // retreating
            else                 pendingGtBallReward = 0f;
        }
        else
        {
            pendingGtBallReward = 0f;
        }
        prevGtBallDistRaw = gtRawNorm;

        // 12. Ground truth (odometry) ball distance - NOT gated by camera visibility,
        // unlike Obs06_BallDistance. Toggleable: when disabled, falls back to a neutral
        // 1 (far) placeholder, but the slot itself is always fed - Space Size stays 12
        // either way.
        Obs12_GroundTruthBallDistance = gtBallDistanceObsEnabled ? Mathf.Clamp01(gtRawNorm) : 1f;

        // 13. Ground truth (odometry) ball angle - signed angle between the ROBOT BODY's
        // own forward direction and the direction to the ball, NOT the camera/servo (that's
        // Obs05_BallAngle) and NOT gated by visibility. 0 = robot is facing the ball dead-on,
        // +-1 = ball is directly behind. Normalized by 180 deg (the only possible max).
        if (ball != null)
        {
            Vector3 toBall = ball.position - transform.position; toBall.y = 0f;
            float signedAngleDeg = Vector3.SignedAngle(transform.forward, toBall, Vector3.up);
            Obs13_GroundTruthBallAngle = Mathf.Clamp(signedAngleDeg / 180f, -1f, 1f);
        }
        else
        {
            Obs13_GroundTruthBallAngle = 0f;
        }

        // --- Feed strictly in the Practice 3 table order ---
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
        float steer  = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float camCmd = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);

        ActGas = gas; ActSteer = steer; ActCameraCmd = camCmd;

        // Domain Randomization: action latency FIFO (no-op when latency is 0)
        if (randomizer != null) randomizer.DelayActions(ref gas, ref steer, ref camCmd);

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

        ComputeRewards(gas, steer, camCmd);
    }

    private void ComputeRewards(float gas, float steer, float camCmd)
    {
        dbg_GripHoldBonusApplied = 0f;
        dbg_GripLostApplied = 0f;
        dbg_SuccessEpisodeApplied = 0f;
        dbg_TimeoutPenaltyApplied = 0f;
        dbg_BallBumperApplied = 0f;

        // ---- Diagnostic telemetry (sim-to-real CSV log, ground truth only used here -
        // never fed into observations/rewards). One row per decision step.
        if (diagnosticLogger != null)
        {
            float dt = Time.time - lastLogTime;
            float speed = dt > 0.0001f ? Vector3.Distance(transform.position, lastLogPos) / dt : 0f;
            diagnosticLogger.LogStep(
                episodeSteps, Obs08_BallVisible > 0.5f, Obs05_BallAngle, Obs06_BallDistance,
                Obs01_Ultrasonic, Obs02_LeftIR, Obs03_RightIR, Obs04_GripperIR, servoAngle,
                gas, steer, Obs10_HasBall > 0.5f, gripHoldDecisions, false,
                transform.position.x - episodeStartPos.x, transform.position.z - episodeStartPos.z,
                transform.eulerAngles.y, speed);
            lastLogPos = transform.position;
            lastLogTime = Time.time;
        }

        // ---- Claw hold tracking: purely observational - driving is NEVER frozen while
        // the sensor is active. Flat, fixed bonus per tick (NOT a % of cumulative reward -
        // avoids exponential growth and removes any incentive to stall/inflate a baseline
        // before grabbing, since the bonus no longer depends on prior accumulated reward).
        bool gripActive = Obs04_GripperIR == 1;
        if (gripActive)
        {
            if (!missionMode)
            {
                gripHoldDecisions++;
                int ticksReached = gripHoldDecisions / gripRewardIntervalDecisions;
                while (gripRewardTicksGranted < ticksReached)
                {
                    gripRewardTicksGranted++;
                    Add(gripHoldTickBonus);
                    dbg_GripHoldBonusApplied += gripHoldTickBonus;
                }

                if (gripHoldDecisions >= gripHoldMaxDecisions)
                {
                    Add(successEpisodeReward);
                    dbg_SuccessEpisodeApplied = successEpisodeReward;
                    RecordRewardStats();
                    EndEpisode();   // success
                    return;
                }
            }
        }
        else
        {
            // Sensor was active and just dropped before reaching success - the ball
            // touched the claw and was lost before the hold completed. Flat penalty,
            // same fixed scale as the hold-tick bonus (not tied to cumulative reward).
            if (gripHoldDecisions > 0 && !missionMode)
            {
                Add(-gripLostPenalty);
                dbg_GripLostApplied = -gripLostPenalty;
            }
            gripHoldDecisions = 0;
            gripRewardTicksGranted = 0;
        }

        // Real, occlusion-aware visibility - Obs08_BallVisible already went through
        // SimulatedYoloCamera's raycast line-of-sight check (or RealVision on hardware),
        // so this means "actually seen", not just "somewhere in the FOV cone through a wall".
        bool ballVisible = Obs08_BallVisible > 0.5f;

        // --- Standing still: flat penalty. Moving forward alone gives nothing (0). ---
        bool standingStill = Mathf.Abs(gas) <= 0.0001f;
        if (standingStill)
        {
            Add(-standingStillPenalty);
            dbg_StandingStillApplied = -standingStillPenalty;
        }
        else
        {
            dbg_StandingStillApplied = 0f;
        }

        // --- Backward: flat penalty, graded by whether the ball is visible right now ---
        if (gas < 0f)
        {
            float penalty = ballVisible ? backwardVisiblePenalty : backwardBlindPenalty;
            Add(-penalty);
            dbg_BackwardApplied = -penalty;
        }
        else
        {
            dbg_BackwardApplied = 0f;
        }

        // --- Ball distance: reward for BEING close to a visible ball (not for having just
        // moved closer this step) - replaces the old movement-based "driving forward while
        // the ball happens to be visible" reward with a genuine "closer = better" signal.
        // Obs06_BallDistance already defaults to 1 (far) when the ball isn't visible, so
        // this is naturally zero whenever ballVisible is false.
        if (ballVisible)
        {
            float diff = Mathf.Abs(Obs06_BallDistance);
            float ballDist = ballDistanceReward * (1f - diff * diff);
            Add(ballDist);
            dbg_BallDistanceApplied = ballDist;
        }
        else
        {
            dbg_BallDistanceApplied = 0f;
        }

        // --- Ground truth (meta) ball-approach reward - the comparison itself already
        // happened once this decision in CollectObservations (see the comment there); here
        // we just apply-and-consume the staged value exactly once. Always active, NOT
        // gated by ballVisible.
        Add(pendingGtBallReward);
        dbg_GTBallApproachApplied = pendingGtBallReward;
        pendingGtBallReward = 0f;

        // --- Centering: flat-scaled formula, only while visible ---
        if (ballVisible)
        {
            float a = Mathf.Abs(Obs09_ServoAngleNorm);
            float b = Mathf.Abs(Obs05_BallAngle);
            float centering = centeringRewardScale * (1f - a) * (1f - b) * (1f - a) * (1f - b) * (1f - a) * (1f - b);
            Add(centering);
            dbg_CenteringApplied = centering;
        }
        else
        {
            dbg_CenteringApplied = 0f;
        }

        // --- Ground truth (meta) centering: SAME formula shape as the visible-ball
        // centering reward above, just on Obs13_GroundTruthBallAngle (robot BODY facing
        // the ball) instead of the camera-perceived Obs05_BallAngle - and always active,
        // NOT gated by ballVisible, since it's ground truth.
        {
            float c = Mathf.Abs(Obs13_GroundTruthBallAngle);
            float gtCentering = gtCenteringReward * (1f - c) * (1f - c) * (1f - c);
            Add(gtCentering);
            dbg_GTCenteringApplied = gtCentering;
        }

        // --- Critical wall proximity: flat penalties, independent of movement and of each
        // other (both triggering the same step just adds both penalties, no compounding).
        if (Obs02_LeftIR == 1 || Obs03_RightIR == 1)
        {
            Add(-sideIRCriticalPenalty);
            dbg_SideIRCriticalApplied = -sideIRCriticalPenalty;
        }
        else
        {
            dbg_SideIRCriticalApplied = 0f;
        }

        if (Obs01_Ultrasonic < ultrasonicCriticalThreshold)
        {
            Add(-ultrasonicCriticalPenalty);
            dbg_UltrasonicCriticalApplied = -ultrasonicCriticalPenalty;
        }
        else
        {
            dbg_UltrasonicCriticalApplied = 0f;
        }

        // --- Sudden move penalty: covers motors (gas/steer) AND the camera servo command.
        // Compares against the last SIGNIFICANT direction per channel (|value| >
        // suddenMoveSignificance), not the immediately preceding step - a raw step-to-step
        // comparison misses oscillation that ramps gradually through zero (e.g. steer/gas
        // driven by Input.GetAxis smoothing in Heuristic, or a smooth policy output), since
        // every individual small step during the ramp has a tiny delta.
        //
        // A reversal only counts as "sudden" if the opposite significant direction shows up
        // within oscillationWindowDecisions of the last one (*Age tracks decisions since the
        // channel was last significant, of EITHER sign). Otherwise a normal turn taken long
        // after the previous one (e.g. went right a while ago, now going left to navigate)
        // would get flagged just for being opposite - it isn't oscillation, just driving.
        gasSigAge++; steerSigAge++; camSigAge++;
        bool suddenMove = false;

        if (Mathf.Abs(gas) > suddenMoveSignificance)
        {
            float sign = Mathf.Sign(gas);
            if (lastSigGas != 0f && sign != lastSigGas && gasSigAge <= oscillationWindowDecisions) suddenMove = true;
            lastSigGas = sign;
            gasSigAge = 0;
        }
        if (Mathf.Abs(steer) > suddenMoveSignificance)
        {
            float sign = Mathf.Sign(steer);
            if (lastSigSteer != 0f && sign != lastSigSteer && steerSigAge <= oscillationWindowDecisions) suddenMove = true;
            lastSigSteer = sign;
            steerSigAge = 0;
        }
        if (Mathf.Abs(camCmd) > suddenMoveSignificance)
        {
            float sign = Mathf.Sign(camCmd);
            if (lastSigCam != 0f && sign != lastSigCam && camSigAge <= oscillationWindowDecisions) suddenMove = true;
            lastSigCam = sign;
            camSigAge = 0;
        }

        if (suddenMove)
        {
            Add(-suddenMovePenalty);
            dbg_SuddenMoveApplied = -suddenMovePenalty;
        }
        else
        {
            dbg_SuddenMoveApplied = 0f;
        }

        // --- Terminal conditions ---
        // (Success is handled by the claw hold tracking at the top of this method.
        // No out-of-bounds check here - the arena is physically walled in.)

        // Mission mode: the FSM owns the mission - never terminate episodes.
        if (missionMode)
        {
            RecordRewardStats();
            return;
        }

        // Time up: reaching here always means the claw hold was never completed (the
        // success branch above already returned early otherwise) - i.e. the robot failed
        // to find/catch the ball in time. Flat penalty on top of the truncation.
        if (episodeSteps >= maxEpisodeSteps)
        {
            Add(-timeoutPenalty);
            dbg_TimeoutPenaltyApplied = -timeoutPenalty;
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
        stats.Add("Rewards/StandingStill", dbg_StandingStillApplied);
        stats.Add("Rewards/BallDistance", dbg_BallDistanceApplied);
        stats.Add("Rewards/GTBallApproach", dbg_GTBallApproachApplied);
        stats.Add("Rewards/Centering", dbg_CenteringApplied);
        stats.Add("Rewards/GTCentering", dbg_GTCenteringApplied);
        stats.Add("Rewards/Backward", dbg_BackwardApplied);
        stats.Add("Rewards/SideIRCritical", dbg_SideIRCriticalApplied);
        stats.Add("Rewards/UltrasonicCritical", dbg_UltrasonicCriticalApplied);
        stats.Add("Rewards/SuddenMove", dbg_SuddenMoveApplied);
        stats.Add("Rewards/GripHoldBonus", dbg_GripHoldBonusApplied);
        stats.Add("Rewards/GripLost", dbg_GripLostApplied);
        stats.Add("Rewards/SuccessEpisode", dbg_SuccessEpisodeApplied);
        stats.Add("Rewards/TimeoutPenalty", dbg_TimeoutPenaltyApplied);
        stats.Add("Rewards/BallBumper", dbg_BallBumperApplied);
    }

    /// <summary>
    /// Called by BumperSensor (attached to the TriggerBamper collider on the robot's
    /// bumper) whenever the ball touches it. Not called from ComputeRewards because the
    /// physics trigger callback can fire on any physics tick, independent of the
    /// decision cadence - AddReward works correctly regardless of when it's called.
    /// </summary>
    public void OnBallHitBumper()
    {
        Add(-ballBumperPenalty);
        dbg_BallBumperApplied = -ballBumperPenalty;
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
