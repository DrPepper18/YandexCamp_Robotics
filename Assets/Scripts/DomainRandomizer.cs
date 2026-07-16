using UnityEngine;

/// <summary>
/// Доменная рандомизация параметров робота, датчиков и среды.
/// Вызывается в начале каждого эпизода из RobotBrain.OnEpisodeBegin().
/// </summary>
public class DomainRandomizer : MonoBehaviour
{
    [Header("Робот")]
    [SerializeField] private Rigidbody robotRb;
    [SerializeField] private Collider robotCollider; // для изменения трения
    [SerializeField] private TrackController trackController;
    [SerializeField] private VirtualSensors sensors;
    [SerializeField] private SimulatedYoloCamera yoloCamera;

    [Header("Диапазоны физики робота")]
    [SerializeField] private Vector2 massRange = new Vector2(2.0f, 3.0f);
    [SerializeField] private Vector2 linearDampingRange = new Vector2(6f, 10f);
    [SerializeField] private Vector2 angularDampingRange = new Vector2(8f, 12f);

    [Header("Диапазоны моторов")]
    [SerializeField] private Vector2 moveSpeedRange = new Vector2(0.5f, 0.65f);
    [SerializeField] private Vector2 turnSpeedRange = new Vector2(100f, 140f);
    [SerializeField] private Vector2 motorDeadzoneRange = new Vector2(8f, 12f);
    [SerializeField] private Vector2 minMotorPwmRange = new Vector2(30f, 40f);
    [SerializeField] private Vector2 maxPwmStepRange = new Vector2(12f, 18f);
    [SerializeField] private Vector2 speedToPwmRange = new Vector2(180f, 220f);

    [Header("Диапазоны датчиков")]
    [SerializeField] private Vector2 ultrasonicMaxDistRange = new Vector2(1.8f, 2.2f);
    [SerializeField] private Vector2 ultrasonicConeAngleRange = new Vector2(25f, 35f);
    [SerializeField] private Vector2 irObstacleDistRange = new Vector2(0.12f, 0.18f);
    [SerializeField] private Vector2 gripperIRDistRange = new Vector2(0.06f, 0.10f);

    [Header("Диапазоны камеры")]
    [SerializeField] private Vector2 cameraMaxDistRange = new Vector2(1.8f, 2.2f);
    [SerializeField] private Vector2 cameraFOVRange = new Vector2(35f, 45f);

    [Header("Шум наблюдений (σ)")]
    [SerializeField] private float ultrasonicNoiseStd = 0.03f;   // добавляется к нормализованному расстоянию
    [SerializeField] private float cameraAngleNoiseStd = 0.05f;  // к относительному углу
    [SerializeField] private float cameraDistNoiseStd = 0.05f;   // к нормализованной дистанции

    [Header("Начальная позиция")]
    [SerializeField] private Transform robotTransform;
    [SerializeField] private Vector2 startOffsetRadius = new Vector2(0f, 0.3f); // макс. сдвиг от старта
    [SerializeField] private float startAngleRange = 15f; // градусы

    // Для внешнего использования шума
    public float UltrasonicNoiseStd => ultrasonicNoiseStd;
    public float CameraAngleNoiseStd => cameraAngleNoiseStd;
    public float CameraDistNoiseStd => cameraDistNoiseStd;

    private Vector3 originalStartPosition;
    private Quaternion originalStartRotation;

    private void Awake()
    {
        if (robotTransform != null)
        {
            originalStartPosition = robotTransform.position;
            originalStartRotation = robotTransform.rotation;
        }
    }

    /// <summary>
    /// Применяет рандомизацию ко всем параметрам. Вызывать в начале эпизода.
    /// </summary>
    public void Apply()
    {
        RandomizePhysics();
        RandomizeMotors();
        RandomizeSensors();
        RandomizeStartPose();
    }

    private void RandomizePhysics()
    {
        if (robotRb != null)
        {
            robotRb.mass = Random.Range(massRange.x, massRange.y);
            robotRb.linearDamping = Random.Range(linearDampingRange.x, linearDampingRange.y);
            robotRb.angularDamping = Random.Range(angularDampingRange.x, angularDampingRange.y);
        }

        if (robotCollider != null && robotCollider.material != null)
        {
            var mat = robotCollider.material;
            mat.dynamicFriction = Random.Range(0.3f, 0.7f);
            mat.staticFriction = Random.Range(0.4f, 0.8f);
        }
    }

    private void RandomizeMotors()
    {
        if (trackController == null) return;

        trackController.moveSpeed = Random.Range(moveSpeedRange.x, moveSpeedRange.y);
        trackController.turnSpeed = Random.Range(turnSpeedRange.x, turnSpeedRange.y);
    }

    private void RandomizeSensors()
    {
        if (sensors == null) return;

        sensors.ultrasonicMaxDistance = Random.Range(ultrasonicMaxDistRange.x, ultrasonicMaxDistRange.y);
        sensors.ultrasonicConeAngle = Random.Range(ultrasonicConeAngleRange.x, ultrasonicConeAngleRange.y);
        sensors.irObstacleDistance = Random.Range(irObstacleDistRange.x, irObstacleDistRange.y);
        sensors.gripperIRDistance = Random.Range(gripperIRDistRange.x, gripperIRDistRange.y);
    }

    private void RandomizeStartPose()
    {
        if (robotTransform == null) return;

        Vector2 offset = Random.insideUnitCircle * Random.Range(startOffsetRadius.x, startOffsetRadius.y);
        Vector3 posOffset = new Vector3(offset.x, 0f, offset.y);
        float angleOffset = Random.Range(-startAngleRange, startAngleRange);

        robotTransform.position = originalStartPosition + posOffset;
        robotTransform.rotation = originalStartRotation * Quaternion.Euler(0f, angleOffset, 0f);
    }

    /// <summary>
    /// Применяет гауссовский шум к переданному значению (для использования в RobotBrain).
    /// </summary>
    public float AddNoise(float value, float stdDev)
    {
        // Простой Box-Muller или Mathf.PerlinNoise – используем нормальное распределение через Random
        float u1 = 1.0f - Random.value;
        float u2 = 1.0f - Random.value;
        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        return value + randStdNormal * stdDev;
    }
}