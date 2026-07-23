using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Classical (No-RL) visual-servo controller for the ball-fetch task.
/// Reads the nearest target from SimulatedYoloCamera and drives TrackController
/// with a simple proportional controller:
///   steer = Kp_angle * RelativeAngle          (center the target in frame)
///   gas   = Kp_size * (targetBboxSize - BboxSize)   (drive forward until the
///                                                   bbox is "big enough", i.e. close)
/// Grabs via the existing GripperController/VirtualSensors.GripperIR (unchanged -
/// that part already works correctly at close range regardless of vision).
///
/// Obstacle avoidance is intentionally NOT implemented yet (per current scope -
/// to be added later). Search behavior (when no target visible) is a simple slow
/// in-place rotation.
/// </summary>
public class VisualServoBrain : MonoBehaviour
{
    public enum State { Searching, Approaching, Grabbing, Done }

    [Header("Component references")]
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private TrackController trackController;
    [SerializeField] private GripperController gripper;
    [SerializeField] private VirtualSensors sensors; // optional, used for final-approach gripper trigger

    [Header("Steering (centering) control")]
    [Tooltip("Proportional gain: steer = Kp_angle * RelativeAngle")]
    [SerializeField] private float kpAngle = 1.0f;
    [Tooltip("Ignore tiny angular errors near the center to avoid jitter and oscillation")]
    [SerializeField] private float angleDeadband = 0.05f;

    [Header("Approach (distance) control")]
    [Tooltip("Apparent bbox size (0..1) considered 'close enough' to stop driving and attempt the grab. " +
             "Tune this in Play mode by watching BboxSize as the robot approaches the real target.")]
    [SerializeField] private float targetBboxSize = 0.35f;
    [Tooltip("Proportional gain: gas = Kp_size * (targetBboxSize - BboxSize), clamped to [0, maxGas]")]
    [SerializeField] private float kpSize = 2.5f;
    [Tooltip("Never drive backward while approaching - only forward, slowing as bbox approaches target size")]
    [SerializeField] private float maxGas = 0.7f;
    [Tooltip("When the target is already large in frame, switch to a straight forward creep to avoid last-second steering turns")]
    [SerializeField] private float finalApproachBboxThreshold = 0.22f;
    [Tooltip("Gas used during the straight creep phase near the ball")]
    [SerializeField] private float finalApproachGas = 0.12f;
    [Tooltip("Reduce steering gain once the object is already large in frame")]
    [SerializeField] private float closeRangeSteerScale = 0.25f;

    [Header("Proximity-loss handling")]
    [Tooltip("If the bbox was at least this big before losing sight, we assume the ball is right in front of us " +
             "and trigger the grace period creep.")]
    [SerializeField] private float proximityAssumeBboxThreshold = 0.2f;
    [Tooltip("How long to keep creeping forward on the last known heading after the ball " +
             "vanishes from frame, before giving up and returning to Searching. In this arena " +
             "line of sight to a VISIBLE ball is never blocked by anything else, so a sudden " +
             "disappearance almost always means the claw/chassis is now occluding it from up " +
             "close - this grace window gives the physical IR sensor a chance to confirm that " +
             "before we spin away and miss it.")]
    [SerializeField] private float visionLossGraceSeconds = 5.0f;

    [Header("Search behavior (no target visible)")]
    [Tooltip("Slow in-place turn rate while searching for a target, -1..1 steer units")]
    [SerializeField] private float searchSteer = 0.3f;

    [Header("Grab")]
    [Tooltip("Consecutive FixedUpdate ticks the claw IR must stay active before we consider the grab successful")]
    [SerializeField] private int gripConfirmTicks = 10;

    [Header("Debug (read-only)")]
    [SerializeField] private State state = State.Searching;
    [SerializeField] private float dbg_gas;
    [SerializeField] private float dbg_steer;
    [SerializeField] private float dbg_bboxSize;

    public State CurrentState => state;

    /// <summary>
    /// Recorded (gas, steer, dt) history since the last Reset(), oldest first.
    /// Intended for a future "replay in reverse to return to start" behavior -
    /// not executed yet, just captured so nothing is lost. Pop from the end
    /// (LIFO) and negate gas (NOT steer - see design notes) to retrace the path.
    /// </summary>
    public readonly List<ActionRecord> ActionHistory = new List<ActionRecord>();

    public struct ActionRecord
    {
        public float gas;
        public float steer;
        public float dt;
    }

    private int gripConfirmCounter;
    private float lastKnownBboxSize;
    
    // State variables for the grace period creep
    private float visionLossTimer = 0f;
    private float lastKnownGas = 0f;
    private float lastKnownSteer = 0f;

    private void FixedUpdate()
    {
        switch (state)
        {
            case State.Searching:
                RunSearching();
                break;
            case State.Approaching:
                RunApproaching();
                break;
            case State.Grabbing:
                RunGrabbing();
                break;
            case State.Done:
                if (trackController != null) trackController.SetCommand(0f, 0f);
                break;
        }
    }

    private void RunSearching()
    {
        if (yoloCamera != null && yoloCamera.IsVisible)
        {
            state = State.Approaching;
            visionLossTimer = 0f; // сбрасываем таймер при обнаружении
            RunApproaching();
            return;
        }

        // No target visible - slow in-place rotation to look for one.
        // Obstacle avoidance intentionally not handled here yet.
        Drive(0f, searchSteer);
    }

    private void RunApproaching()
    {
        bool physicallyClose = sensors != null && sensors.GripperIR == 1;

        if (yoloCamera == null || !yoloCamera.IsVisible)
        {
            // The ball just vanished from frame.
            bool wasCloseBeforeLosingSight = lastKnownBboxSize >= proximityAssumeBboxThreshold;

            if (physicallyClose)
            {
                if (trackController != null) trackController.HardStop();
                state = State.Grabbing;
                gripConfirmCounter = 0;
                if (gripper != null) gripper.GripCommand();
                return;
            }

            // Логика grace-периода: если объект пропал, но мы были близко,
            // продолжаем ползти вперед с последними известными параметрами.
            if (wasCloseBeforeLosingSight && visionLossTimer < visionLossGraceSeconds)
            {
                visionLossTimer += Time.fixedDeltaTime;
                Drive(lastKnownGas, lastKnownSteer);
                return;
            }

            // Genuinely lost from a distance (bbox was never close) OR grace window expired
            // fall back to searching.
            state = State.Searching;
            visionLossTimer = 0f;
            Drive(0f, 0f);
            return;
        }

        // --- Target is VISIBLE ---
        visionLossTimer = 0f; // сбрасываем таймер, пока видим цель
        
        dbg_bboxSize = yoloCamera.BboxSize;
        lastKnownBboxSize = yoloCamera.BboxSize;

        // Centering.
        float steer = Mathf.Clamp(kpAngle * yoloCamera.RelativeAngle, -1f, 1f);
        float angleAbs = Mathf.Abs(yoloCamera.RelativeAngle);

        // Ignore tiny center errors to avoid jitter right before the grab.
        if (angleAbs < angleDeadband)
        {
            steer = 0f;
        }

        // Approach: drive forward while the apparent target is smaller than the
        // "close enough" size, slowing down as it grows (P controller on bbox size).
        float sizeError = targetBboxSize - yoloCamera.BboxSize;
        float gas = Mathf.Clamp(kpSize * sizeError, 0f, maxGas);
        lastKnownGas = gas;

        // Near the ball, reduce steering sharply and switch to a straight forward creep.
        bool nearBall = yoloCamera.BboxSize >= finalApproachBboxThreshold;
        if (nearBall)
        {
            steer *= closeRangeSteerScale;
            gas = Mathf.Min(gas, finalApproachGas);
        }

        lastKnownSteer = steer;

        if (sizeError <= 0f || physicallyClose)
        {
            // HardStop instead of Drive(0,0) [...] 
            if (physicallyClose && trackController != null)
            {
                trackController.HardStop();
            }
            else
            {
                Drive(0f, 0f);
            }

            state = State.Grabbing;
            gripConfirmCounter = 0;
            if (gripper != null) gripper.GripCommand();
            return;
        }
        Drive(gas, steer);
    }

    private void RunGrabbing()
    {
        if (trackController != null) trackController.HardStop();
        else Drive(0f, 0f);

        if (gripper != null) gripper.GripCommand(); // keep commanding closed while confirming

        bool gripActive = sensors != null && sensors.GripperIR == 1;

        if (gripActive)
        {
            gripConfirmCounter++;
            if (gripConfirmCounter >= gripConfirmTicks)
            {
                state = State.Done;
            }
        }
        else
        {
            // Lost the ball before confirming - back off slightly and try approaching again.
            gripConfirmCounter = 0;
            state = State.Approaching;
        }
    }

    /// <summary>
    /// Sends the command to TrackController and records it in ActionHistory
    /// for a future reverse-replay "return to start" behavior.
    /// </summary>
    private void Drive(float gas, float steer)
    {
        dbg_gas = gas;
        dbg_steer = steer;

        if (trackController != null) trackController.SetCommand(gas, steer);

        ActionHistory.Add(new ActionRecord { gas = gas, steer = steer, dt = Time.fixedDeltaTime });
    }

    /// <summary>Resets the state machine and clears recorded history. Call this to start a fresh attempt.</summary>
    public void ResetController()
    {
        state = State.Searching;
        gripConfirmCounter = 0;
        lastKnownBboxSize = 0f;
        visionLossTimer = 0f;
        lastKnownGas = 0f;
        lastKnownSteer = 0f;
        ActionHistory.Clear();
        if (trackController != null) trackController.SetCommand(0f, 0f);
    }
}