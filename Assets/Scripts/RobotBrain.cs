using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

/// <summary>
/// ИИ-агент робота на базе ML-Agents (15 наблюдений, 3 непрер. + 1 дискр. действие).
/// Наблюдения хранятся как публичные поля/свойства — доступны из HUD и Inspector
/// (в Debug-режиме Inspector). Порядок наблюдений синхронизирован со спецификацией.
/// </summary>
public class RobotBrain : Agent
{
    [Header("Ссылки на компоненты робота")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private VirtualSensors sensors;
    [SerializeField] private GripperController gripper;
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private Transform cameraServo;   // может быть null, если камера не вращается
    [SerializeField] private Rigidbody rb;

    [Header("Цель и арена")]
    [SerializeField] private Transform ballTransform;
    [SerializeField] private Vector3 arenaMin = new Vector3(-2f, -0.5f, -2f);
    [SerializeField] private Vector3 arenaMax = new Vector3( 2f,  2.0f,  2f);

    [Tooltip("Сбрасывать ли мяч в новую позицию при старте эпизода. Сними для отладки.")]
    [SerializeField] private bool resetBallOnEpisode = true;

    [Tooltip("Если resetBallOnEpisode = true — рандомизировать позицию (иначе фиксированная точка перед роботом).")]
    [SerializeField] private bool randomizeBallOnReset = true;

    [Tooltip("Минимальная дистанция спавна мяча от робота (только для рандомизации), м")]
    [SerializeField] private float ballSpawnMinDistance = 0.5f;

    [Tooltip("Максимальная дистанция спавна мяча от робота (только для рандомизации), м")]
    [SerializeField] private float ballSpawnMaxDistance = 1.5f;

    [Header("Сервопривод камеры (если есть)")]
    [SerializeField] private float servoMaxAngle = 90f;
    [SerializeField] private float servoSpeed    = 60f;

    [Header("Награды")]
    [SerializeField] private float approachRewardScale    = 0.05f;
    [SerializeField] private float actionRatePenaltyScale = 0.001f;
    [SerializeField] private float centeringBonusScale    = 0.01f;
    [SerializeField] private float wallProximityPenalty   = 0.02f;
    [SerializeField] private float successReward          = 5.0f;
    [SerializeField] private float outOfBoundsPenalty     = -2.0f;
    [SerializeField] private float wallCriticalUsThreshold = 0.15f;

    // ---------- Наблюдения (Debug — видны в Inspector, доступны через свойства) ----------
    [Header("OBSERVATIONS (read-only, для отладки)")]
    [SerializeField] private float o01_ultrasonic;
    [SerializeField] private int   o02_leftIR;
    [SerializeField] private int   o03_rightIR;
    [SerializeField] private int   o04_gripperIR;
    [SerializeField] private float o05_ballAngle;
    [SerializeField] private float o06_ballDistance;
    [SerializeField] private float o07_lastKnownAngle;
    [SerializeField] private float o08_ballVisible;
    [SerializeField] private float o09_servoAngleNorm;
    [SerializeField] private float o10_hasBall;
    [SerializeField] private float o11_dxNorm;
    [SerializeField] private float o12_dzNorm;
    [SerializeField] private float o13_headingNorm;
    [SerializeField] private float o14_speed;
    [SerializeField] private float o15_timeSinceBallNorm;

    [Header("ACTIONS (read-only, для отладки)")]
    [SerializeField] private float actGas;
    [SerializeField] private float actSteer;
    [SerializeField] private float actCameraCmd;
    [SerializeField] private int   actGripCmd;

    [Header("REWARD (read-only)")]
    [SerializeField] private float rewardStep;         // награда за последний шаг
    [SerializeField] private float rewardCumulative;   // накопленная за эпизод

    // Публичные аксессоры для HUD
    public float Obs01_Ultrasonic       => o01_ultrasonic;
    public int   Obs02_LeftIR           => o02_leftIR;
    public int   Obs03_RightIR          => o03_rightIR;
    public int   Obs04_GripperIR        => o04_gripperIR;
    public float Obs05_BallAngle        => o05_ballAngle;
    public float Obs06_BallDistance     => o06_ballDistance;
    public float Obs07_LastKnownAngle   => o07_lastKnownAngle;
    public float Obs08_BallVisible      => o08_ballVisible;
    public float Obs09_ServoAngleNorm   => o09_servoAngleNorm;
    public float Obs10_HasBall          => o10_hasBall;
    public float Obs11_DxNorm           => o11_dxNorm;
    public float Obs12_DzNorm           => o12_dzNorm;
    public float Obs13_HeadingNorm      => o13_headingNorm;
    public float Obs14_Speed            => o14_speed;
    public float Obs15_TimeSinceBallNorm=> o15_timeSinceBallNorm;

    public float ActGas        => actGas;
    public float ActSteer      => actSteer;
    public float ActCameraCmd  => actCameraCmd;
    public int   ActGripCmd    => actGripCmd;

    public float StepReward       => rewardStep;
    public float CumulativeReward => rewardCumulative;

    // ---- Внутреннее состояние ----
    private Vector3 startPosition;
    private Quaternion startRotation;
    private float lastKnownBallAngle;
    private float timeSinceLastBallSeen;
    private float servoAngle;
    private float prevDistanceToBall;
    private float prevGas, prevSteer;
    private bool  hasBall;
    private float arenaSize;

    private bool initialized;

    private void Awake()
    {
        EnsureInitialized();
    }

    public override void Initialize()
    {
        EnsureInitialized();
    }

    private void EnsureInitialized()
    {
        if (initialized) return;
        if (rb == null) rb = GetComponent<Rigidbody>();
        startPosition = transform.position;
        startRotation = transform.rotation;
        arenaSize = Mathf.Max(1f, Mathf.Max(arenaMax.x - arenaMin.x, arenaMax.z - arenaMin.z));
        initialized = true;
    }

    // FixedUpdate работает независимо от ML-Agents pipeline —
    // обеспечивает актуальные значения в HUD/Inspector даже если
    // Behavior Parameters/Decision Requester ещё не настроены.
    private void FixedUpdate()
    {
        // Обновляем "память" про мяч
        if (yoloCamera != null && yoloCamera.IsVisible)
        {
            lastKnownBallAngle = yoloCamera.RelativeAngle;
            timeSinceLastBallSeen = 0f;
        }
        else
        {
            timeSinceLastBallSeen += Time.fixedDeltaTime;
        }
        hasBall = gripper != null && gripper.IsHolding;

        // Пересчитываем все 15 наблюдений
        ComputeObservations();
    }

    public override void OnEpisodeBegin()
    {
<<<<<<< HEAD
        startPosition = transform.position;
        currentServoAngle = 0f;
        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        previousLinearAction = 0f;
        previousAngularAction = 0f;

        previousDistanceToBall = GetDistanceToBall();

        if (cameraServoPivot != null)
        {
            cameraServoPivot.localRotation = Quaternion.identity;
        }
    }

    private void Update()
    {
        // Track "time since last ball detection" and "last known direction"
        // continuously, independently of the ML-Agents decision cadence.
        if (yoloCamera != null && yoloCamera.IsBallVisible)
        {
            lastKnownBallDirection = yoloCamera.HorizontalOffset;
            timeSinceLastDetection = 0f;
        }
        else
        {
            timeSinceLastDetection += Time.deltaTime;
        }
=======
        transform.SetPositionAndRotation(startPosition, startRotation);
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        if (gripper != null && gripper.IsHolding) gripper.ReleaseCommand();

        servoAngle = 0f;
        if (cameraServo != null) cameraServo.localRotation = Quaternion.identity;

        // Сброс мяча
        if (ballTransform != null && resetBallOnEpisode)
        {
            Vector3 newBallPos;

            if (randomizeBallOnReset)
            {
                // Случайное направление и дистанция от робота
                float angle = Random.Range(0f, Mathf.PI * 2f);
                float dist  = Random.Range(ballSpawnMinDistance, ballSpawnMaxDistance);
                Vector3 offset = new Vector3(Mathf.Sin(angle) * dist, 0f, Mathf.Cos(angle) * dist);
                newBallPos = startPosition + offset;

                // Ограничиваем аренной, чтобы не улетел за пределы
                newBallPos.x = Mathf.Clamp(newBallPos.x, arenaMin.x + 0.2f, arenaMax.x - 0.2f);
                newBallPos.z = Mathf.Clamp(newBallPos.z, arenaMin.z + 0.2f, arenaMax.z - 0.2f);
                newBallPos.y = startPosition.y;
            }
            else
            {
                // Фиксированная точка — метр перед стартовой позицией робота
                newBallPos = startPosition + startRotation * new Vector3(0f, 0f, 1f);
            }

            ballTransform.position = newBallPos;

            var ballRb = ballTransform.GetComponent<Rigidbody>();
            if (ballRb != null) { ballRb.linearVelocity = Vector3.zero; ballRb.angularVelocity = Vector3.zero; }
        }

        lastKnownBallAngle = 0f;
        timeSinceLastBallSeen = 0f;
        prevGas = 0f;
        prevSteer = 0f;
        hasBall = false;
        rewardStep = 0f;
        rewardCumulative = 0f;

        prevDistanceToBall = ballTransform != null
            ? Vector3.Distance(transform.position, ballTransform.position)
            : 0f;
    }

    /// <summary>Обновляет все 15 полей наблюдений (можно вызывать из HUD или наградного расчёта).</summary>
    private void ComputeObservations()
    {
        // 1. УЗ (мин из двух)
        o01_ultrasonic = sensors != null
            ? Mathf.Min(sensors.UltrasonicLeft, sensors.UltrasonicRight)
            : 1f;

        // 2, 3. Боковые ИК
        o02_leftIR  = sensors != null ? sensors.LeftIR  : 0;
        o03_rightIR = sensors != null ? sensors.RightIR : 0;

        // 4. ИК клешни
        o04_gripperIR = sensors != null ? sensors.GripperIR : 0;

        // 5, 6. Угол и дистанция до мяча (по камере)
        bool visible = yoloCamera != null && yoloCamera.IsVisible;
        o05_ballAngle    = visible ? yoloCamera.RelativeAngle : 0f;
        o06_ballDistance = visible ? yoloCamera.NormalizedDistance : 1f;

        // 7. Последнее известное направление
        o07_lastKnownAngle = lastKnownBallAngle;

        // 8. Флаг видимости
        o08_ballVisible = visible ? 1f : 0f;

        // 9. Угол сервопривода (нормализованный)
        o09_servoAngleNorm = servoAngle / Mathf.Max(1f, servoMaxAngle);

        // 10. hasBall
        o10_hasBall = hasBall ? 1f : 0f;

        // 11, 12. Смещение от старта (нормализованное к размеру арены)
        Vector3 delta = transform.position - startPosition;
        o11_dxNorm = delta.x / arenaSize;
        o12_dzNorm = delta.z / arenaSize;

        // 13. Heading
        float headingDeg = Mathf.DeltaAngle(startRotation.eulerAngles.y, transform.eulerAngles.y);
        o13_headingNorm = headingDeg / 180f;

        // 14. Скорость
        o14_speed = rb != null ? rb.linearVelocity.magnitude : 0f;

        // 15. Время с последней детекции (нормализованное к 10 сек)
        o15_timeSinceBallNorm = Mathf.Clamp(timeSinceLastBallSeen, 0f, 10f) / 10f;
>>>>>>> ade5866945b7458351b0d9a434b11a7a8c2180bb
    }

    public override void CollectObservations(VectorSensor sensor)
    {
<<<<<<< HEAD
        // 1. Normalized ultrasonic distance (0 = touching, 1 = clear).
        sensor.AddObservation(sensors != null ? sensors.UltrasonicDistance01 : 1f);

        // 2. Left IR obstacle sensor (0/1).
        sensor.AddObservation(sensors != null ? sensors.LeftIR : 0);

        // 3. Right IR obstacle sensor (0/1).
        sensor.AddObservation(sensors != null ? sensors.RightIR : 0);

        // 4. Gripper IR sensor (0/1).
        sensor.AddObservation(sensors != null ? sensors.GripperIR : 0);

        // 5. Relative horizontal angle to the ball from the camera (0 if not visible).
        bool ballVisible = yoloCamera != null && yoloCamera.IsBallVisible;
        sensor.AddObservation(ballVisible ? yoloCamera.HorizontalOffset : 0f);

        // 6. Normalized distance to the ball from the camera (1 if not visible).
        sensor.AddObservation(ballVisible ? yoloCamera.NormalizedDistance : 1f);

        // 7. Last known direction to the ball (after it left the frame).
        sensor.AddObservation(lastKnownBallDirection);

        // 8. Ball visibility flag (0.0 / 1.0).
        sensor.AddObservation(ballVisible ? 1f : 0f);

        // 9. Current camera servo rotation, normalized to -1..1.
        float normalizedServo = maxServoAngle > 0f ? currentServoAngle / maxServoAngle : 0f;
        sensor.AddObservation(normalizedServo);

        // 10. Gripper hold status (hasBall -> 0.0 / 1.0).
        bool hasBall = gripper != null && gripper.HasBall;
        sensor.AddObservation(hasBall ? 1f : 0f);

        // 11. Relative X offset from the start position.
        Vector3 offset = transform.position - startPosition;
        sensor.AddObservation(offset.x);

        // 12. Relative Z offset from the start position.
        sensor.AddObservation(offset.z);

        // 13. Normalized heading angle of the robot.
        float heading = transform.eulerAngles.y;
        if (heading > 180f) heading -= 360f; // wrap to -180..180
        sensor.AddObservation(heading / headingNormalizer);

        // 14. Current movement speed from the Rigidbody.
        float speed = rb != null ? rb.linearVelocity.magnitude : 0f;
        sensor.AddObservation(speed);

        // 15. Time elapsed since the last ball detection, normalized.
        sensor.AddObservation(timeSinceLastDetection / timeSinceDetectionNormalizer);
=======
        ComputeObservations();

        sensor.AddObservation(o01_ultrasonic);
        sensor.AddObservation(o02_leftIR);
        sensor.AddObservation(o03_rightIR);
        sensor.AddObservation(o04_gripperIR);
        sensor.AddObservation(o05_ballAngle);
        sensor.AddObservation(o06_ballDistance);
        sensor.AddObservation(o07_lastKnownAngle);
        sensor.AddObservation(o08_ballVisible);
        sensor.AddObservation(o09_servoAngleNorm);
        sensor.AddObservation(o10_hasBall);
        sensor.AddObservation(o11_dxNorm);
        sensor.AddObservation(o12_dzNorm);
        sensor.AddObservation(o13_headingNorm);
        sensor.AddObservation(o14_speed);
        sensor.AddObservation(o15_timeSinceBallNorm);
>>>>>>> ade5866945b7458351b0d9a434b11a7a8c2180bb
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
<<<<<<< HEAD
        // --- Continuous actions ---
        float linear = actions.ContinuousActions[0];   // -1..1
        float angular = actions.ContinuousActions[1];   // -1..1
        float cameraServoSignal = actions.ContinuousActions[2]; // -1..1

        if (trackController != null)
        {
            trackController.SetCommand(linear, angular);
        }

        // Rotate the camera servo pivot based on the signal, clamped to +-maxServoAngle.
        currentServoAngle = Mathf.Clamp(
            currentServoAngle + cameraServoSignal * servoSpeed * Time.deltaTime,
            -maxServoAngle,
            maxServoAngle);

        if (cameraServoPivot != null)
        {
            cameraServoPivot.localRotation = Quaternion.Euler(0f, currentServoAngle, 0f);
        }

        // --- Discrete action: gripper command ---
        // 0 = do nothing, 1 = close, 2 = open.
        int gripperCommand = actions.DiscreteActions[0];

        if (gripper != null)
        {
            if (gripperCommand == 1)
            {
                gripper.SetClawClosed(true);
            }
            else if (gripperCommand == 2)
            {
                gripper.SetClawClosed(false);
            }
            // gripperCommand == 0 -> leave the claw state unchanged.
        }

        CalculateRewards(linear, angular);
    }

    /// <summary>
    /// Computes and applies per-step rewards/penalties, and checks terminal conditions.
    /// Called at the end of OnActionReceived.
    /// </summary>
    private void CalculateRewards(float linear, float angular)
    {
        // 1) Distance-delta reward: reward closing distance to the ball,
        //    with a stronger reward the closer the robot already is.
        float currentDistance = GetDistanceToBall();
        if (currentDistance >= 0f && previousDistanceToBall >= 0f)
        {
            float delta = previousDistanceToBall - currentDistance; // positive = got closer
            bool isClose = currentDistance <= closeDistanceThreshold;
            float scale = isClose ? distanceRewardScaleNear : distanceRewardScaleFar;
            AddReward(delta * scale);
        }
        if (currentDistance >= 0f)
        {
            previousDistanceToBall = currentDistance;
        }

        // 2) Action-rate penalty: discourage jerky gas/steer changes between steps.
        float actionDelta = Mathf.Abs(linear - previousLinearAction) + Mathf.Abs(angular - previousAngularAction);
        AddReward(-actionRatePenaltyScale * actionDelta);
        previousLinearAction = linear;
        previousAngularAction = angular;

        // 3) Centering bonus: reward facing the ball (small horizontal camera offset)
        //    so the robot lines up before attempting a grab.
        if (yoloCamera != null && yoloCamera.IsBallVisible)
        {
            float alignment = 1f - Mathf.Abs(yoloCamera.HorizontalOffset); // 1 = dead center, 0 = at FOV edge
            AddReward(alignment * centeringRewardScale);
        }

        // 4) Distance-sensor penalty: discourage getting dangerously close to walls.
        if (sensors != null)
        {
            if (sensors.UltrasonicDistance01 < wallProximityThreshold)
            {
                AddReward(-wallPenaltyScale);
            }
            if (sensors.LeftIR == 1) AddReward(-irWallPenalty);
            if (sensors.RightIR == 1) AddReward(-irWallPenalty);
        }

        // 5) Terminal conditions.
        if (gripper != null && gripper.HasBall)
        {
            AddReward(successReward);
            EndEpisode();
            return;
        }

        if (HasFallenOffArena())
        {
            AddReward(fallPenalty);
            EndEpisode();
            return;
        }
    }

    /// <summary>
    /// World-space distance from the robot to the target ball, or -1 if no ball is assigned.
    /// Uses the ball reference from the YOLO camera so both systems agree on the same target.
    /// </summary>
    private float GetDistanceToBall()
    {
        if (yoloCamera == null || yoloCamera.targetBall == null)
        {
            return -1f;
        }
        return Vector3.Distance(transform.position, yoloCamera.targetBall.position);
    }

    /// <summary>
    /// True if the robot is considered to have fallen off the arena, either because it left
    /// the arenaBounds collider (if assigned) or dropped below fallYThreshold.
    /// </summary>
    private bool HasFallenOffArena()
    {
        if (arenaBounds != null)
        {
            return !arenaBounds.bounds.Contains(transform.position);
        }

        return transform.position.y < fallYThreshold;
=======
        actGas       = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        actSteer     = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        actCameraCmd = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        actGripCmd   = actions.DiscreteActions[0];

        if (trackController != null) trackController.SetCommand(actGas, actSteer);

        // Сервопривод — только если назначен
        if (cameraServo != null)
        {
            servoAngle = Mathf.Clamp(
                servoAngle + actCameraCmd * servoSpeed * Time.fixedDeltaTime,
                -servoMaxAngle, servoMaxAngle);
            cameraServo.localRotation = Quaternion.Euler(0f, servoAngle, 0f);
        }

        if (gripper != null)
        {
            if (actGripCmd == 1 && !gripper.IsHolding) gripper.GripCommand();
            else if (actGripCmd == 2 && gripper.IsHolding) gripper.ReleaseCommand();
        }

        // Состояние (lastKnownBallAngle, timeSinceLastBallSeen, hasBall) и
        // ComputeObservations обновляются в FixedUpdate независимо от pipeline.

        // Награды с отслеживанием
        float before = GetCumulativeReward();
        ApplyRewards(actGas, actSteer);
        float after = GetCumulativeReward();
        rewardStep = after - before;
        rewardCumulative = after;

        prevGas = actGas;
        prevSteer = actSteer;
    }

    private void ApplyRewards(float gas, float steer)
    {
        if (ballTransform == null) return;

        float currentDistance = Vector3.Distance(transform.position, ballTransform.position);

        float delta = prevDistanceToBall - currentDistance;
        float proximityFactor = Mathf.Clamp01(1f - currentDistance / 2f);
        AddReward(delta * approachRewardScale * (1f + proximityFactor));
        prevDistanceToBall = currentDistance;

        float actionDiff = Mathf.Abs(gas - prevGas) + Mathf.Abs(steer - prevSteer);
        AddReward(-actionDiff * actionRatePenaltyScale);

        if (yoloCamera != null && yoloCamera.IsVisible)
            AddReward((1f - Mathf.Abs(yoloCamera.RelativeAngle)) * centeringBonusScale);

        if (sensors != null)
        {
            float minUs = Mathf.Min(sensors.UltrasonicLeft, sensors.UltrasonicRight);
            if (minUs < wallCriticalUsThreshold) AddReward(-wallProximityPenalty);
            if (sensors.LeftIR == 1 || sensors.RightIR == 1) AddReward(-wallProximityPenalty);
        }

        if (hasBall) { AddReward(successReward); EndEpisode(); return; }

        Vector3 pos = transform.position;
        if (pos.x < arenaMin.x || pos.x > arenaMax.x ||
            pos.y < arenaMin.y || pos.y > arenaMax.y ||
            pos.z < arenaMin.z || pos.z > arenaMax.z)
        {
            AddReward(outOfBoundsPenalty);
            EndEpisode();
        }
>>>>>>> ade5866945b7458351b0d9a434b11a7a8c2180bb
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
<<<<<<< HEAD
        // Manual control fallback for testing without a trained model.
        var continuousActions = actionsOut.ContinuousActions;
        var discreteActions = actionsOut.DiscreteActions;

        var keyboard = Keyboard.current;
        if (keyboard == null)
        {
            continuousActions[0] = 0f;
            continuousActions[1] = 0f;
            continuousActions[2] = 0f;
            discreteActions[0] = 0;
            return;
        }

        float gas = 0f;
        if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) gas += 1f;
        if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) gas -= 1f;

        float steer = 0f;
        if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) steer += 1f;
        if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) steer -= 1f;

        continuousActions[0] = gas;
        continuousActions[1] = steer;
        continuousActions[2] = 0f;

        if (keyboard.qKey.wasPressedThisFrame) discreteActions[0] = 1; // close
        else if (keyboard.eKey.wasPressedThisFrame) discreteActions[0] = 2; // open
        else discreteActions[0] = 0;
=======
        var continuous = actionsOut.ContinuousActions;
        var discrete   = actionsOut.DiscreteActions;

        continuous[0] = Input.GetAxis("Vertical");
        continuous[1] = Input.GetAxis("Horizontal");

        float camCmd = 0f;
        if (Input.GetKey(KeyCode.Q)) camCmd -= 1f;
        if (Input.GetKey(KeyCode.E)) camCmd += 1f;
        continuous[2] = camCmd;

        int grip = 0;
        if (Input.GetKeyDown(KeyCode.Space))     grip = 1;
        if (Input.GetKeyDown(KeyCode.LeftShift)) grip = 2;
        discrete[0] = grip;
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = new Color(1f, 0.6f, 0f, 0.5f);
        Vector3 center = (arenaMin + arenaMax) * 0.5f;
        Vector3 size = arenaMax - arenaMin;
        Gizmos.DrawWireCube(center, size);
>>>>>>> ade5866945b7458351b0d9a434b11a7a8c2180bb
    }
}