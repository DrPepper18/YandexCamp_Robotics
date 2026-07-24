using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Classical (No-RL) visual-servo controller for the ball-fetch task.
/// Reads the nearest target from SimulatedYoloCamera and drives TrackController
/// with a simple proportional controller:
///   steer = Kp_angle * RelativeAngle          (center the target in frame)
///   gas   = Kp_size * (targetBboxSize - BboxSize)   (drive forward until the
///                                                   bbox is "big enough", i.e. close)
/// Grabs via the existing GripperController/VirtualSensors.GripperIR.
/// Replays recorded movements in reverse (LIFO) after a successful grab to return to the start position.
/// </summary>
public class VisualServoBrain : MonoBehaviour
{
    public enum State { Searching, Approaching, Grabbing, Returning, Done }

    [Header("Component references")]
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private TrackController trackController;
    [SerializeField] private GripperController gripper;
    private bool IsBallCaptured => (sensors != null && sensors.GripperIR == 1) || (gripper != null && gripper.IsHolding);
    [SerializeField] private VirtualSensors sensors; // optional, used for final-approach gripper trigger

    [Header("Steering (centering) control")]
    [Tooltip("Proportional gain: steer = Kp_angle * RelativeAngle")]
    [SerializeField] private float kpAngle = 1.0f;
    [Tooltip("Ignore tiny angular errors near the center to avoid jitter and oscillation")]
    [SerializeField] private float angleDeadband = 0.05f;

    [Header("Approach (distance) control")]
    [Tooltip("Apparent bbox size (0..1) considered 'close enough' to stop driving and attempt the grab.")]
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
    [Tooltip("If the bbox was at least this big before losing sight, we assume the ball is right in front of us and trigger the grace period creep.")]
    [SerializeField] private float proximityAssumeBboxThreshold = 0.2f;
    [Tooltip("How long to keep creeping forward on the last known heading after the ball vanishes from frame.")]
    [SerializeField] private float visionLossGraceSeconds = 0.5f;

    [Header("Search behavior (no target visible)")]
    [Tooltip("Slow in-place turn rate while searching for a target, -1..1 steer units")]
    [SerializeField] private float searchSteer = 0.3f;

    [Header("Grab")]
    [Tooltip("Consecutive FixedUpdate ticks the claw IR must stay active before we consider the grab successful")]
    [SerializeField] private int gripConfirmTicks = 10;
    [Header("Grip Settings")]
[SerializeField] private int gripRequiredTicks = 10; // Сколько тиков удерживать мяч перед переходом в Returning

    [Header("Debug (read-only)")]
    [SerializeField] private State state = State.Searching;
    [SerializeField] public float dbg_gas;
    [SerializeField] public float dbg_steer;
    [SerializeField] private float dbg_bboxSize;

    public State CurrentState => state;

    /// <summary>
    /// Recorded (gas, steer, dt) history since the last Reset(), oldest first.
    /// Used for LIFO reverse playback to return to start position after grabbing.
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
            case State.Returning:
                RunReturning();
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
            visionLossTimer = 0f;
            RunApproaching();
            return;
        }

        Drive(0f, searchSteer);
    }

    private void RunApproaching()
    {
        if (IsBallCaptured)
        {
            if (trackController != null) trackController.HardStop();
            state = State.Grabbing;
            gripConfirmCounter = 0;
            if (gripper != null) gripper.GripCommand();
            return;
        }
        bool physicallyClose = sensors != null && sensors.GripperIR == 1;

        if (yoloCamera == null || !yoloCamera.IsVisible)
        {
            bool wasCloseBeforeLosingSight = lastKnownBboxSize >= proximityAssumeBboxThreshold;

            if (physicallyClose)
            {
                if (trackController != null) trackController.HardStop();
                state = State.Grabbing;
                gripConfirmCounter = 0;
                if (gripper != null) gripper.GripCommand();
                return;
            }

            if (wasCloseBeforeLosingSight && visionLossTimer < visionLossGraceSeconds)
            {
                visionLossTimer += Time.fixedDeltaTime;
                Drive(lastKnownGas, lastKnownSteer);
                return;
            }

            state = State.Searching;
            visionLossTimer = 0f;
            Drive(0f, 0f);
            return;
        }

        visionLossTimer = 0f;
        
        dbg_bboxSize = yoloCamera.BboxSize;
        lastKnownBboxSize = yoloCamera.BboxSize;

        float steer = Mathf.Clamp(kpAngle * yoloCamera.RelativeAngle, -1f, 1f);
        float angleAbs = Mathf.Abs(yoloCamera.RelativeAngle);

        if (angleAbs < angleDeadband)
        {
            steer = 0f;
        }

        float sizeError = targetBboxSize - yoloCamera.BboxSize;
        float gas = Mathf.Clamp(kpSize * sizeError, 0f, maxGas);
        lastKnownGas = gas;

        bool nearBall = yoloCamera.BboxSize >= finalApproachBboxThreshold;
        if (nearBall)
        {
            steer *= closeRangeSteerScale;
            gas = Mathf.Min(gas, finalApproachGas);
        }

        lastKnownSteer = steer;

        if (sizeError <= 0f || physicallyClose)
        {
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
        if (gripper != null) gripper.GripCommand();

        // Мяч зажат ИЛИ уже зацеплен клешней
        if (IsBallCaptured)
        {
            gripConfirmCounter++;
            if (gripConfirmCounter >= gripRequiredTicks)
            {
                // УСПЕХ! Мяч у нас, наваливаем назад
                state = State.Returning;
                gripConfirmCounter = 0;
            }
        }
        else
        {
            // Если реально выронили — откатываемся
            state = State.Approaching;
        }
    }

    /// <summary>
    /// Replays recorded actions in LIFO order to return to the starting position.
    /// Negates gas (drives backwards) while keeping steer unchanged.
    /// </summary>
    private void RunReturning()
    {
        // Клешня остается закрытой во время движения назад
        if (gripper != null) gripper.GripCommand();

        // Пропускаем все "нулевые" команды из конца истории (паузы, остановки)
        while (ActionHistory.Count > 0)
        {
            int lastIdx = ActionHistory.Count - 1;
            ActionRecord top = ActionHistory[lastIdx];

            // Если действие было почти нулевым, просто выбрасываем его и смотрим следующее
            if (Mathf.Abs(top.gas) < 0.01f && Mathf.Abs(top.steer) < 0.01f)
            {
                ActionHistory.RemoveAt(lastIdx);
            }
            else
            {
                break; // Нашли реальное движение!
            }
        }

        if (ActionHistory.Count == 0)
        {
            // История пуста — мы полностью вернулись в точку старта
            state = State.Done;
            if (trackController != null) trackController.HardStop();
            return;
        }

        // Извлекаем последнее реальное движение (LIFO)
        int index = ActionHistory.Count - 1;
        ActionRecord record = ActionHistory[index];
        ActionHistory.RemoveAt(index);

        dbg_gas = -record.gas;
        dbg_steer = record.steer;

        // Отправляем инвертированный газ напрямую в контроллер
        if (trackController != null)
        {
            trackController.SetCommand(-record.gas, record.steer);
        }
    }

    /// <summary>
    /// Sends the command to TrackController and records non-zero actions into ActionHistory.
    /// </summary>
    private void Drive(float gas, float steer)
    {
        dbg_gas = gas;
        dbg_steer = steer;

        if (trackController != null) trackController.SetCommand(gas, steer);

        // НЕ записываем в историю паузы / полные остановки, чтобы не засорять стек
        if (Mathf.Abs(gas) > 0.01f || Mathf.Abs(steer) > 0.01f)
        {
            ActionHistory.Add(new ActionRecord { gas = gas, steer = steer, dt = Time.fixedDeltaTime });
        }
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