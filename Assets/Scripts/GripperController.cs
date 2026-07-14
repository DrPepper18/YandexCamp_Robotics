using UnityEngine;

/// <summary>
/// Логический захват мяча клешнёй.
/// Когда VirtualSensors.GripperIR == 1 и игрок жмёт кнопку захвата, мяч:
///   - становится kinematic (не считается физикой),
///   - его коллайдер выключается,
///   - он делается дочерним объектом HoldPoint (SetParent), локальная позиция обнуляется.
/// При отпускании — всё восстанавливается в исходное состояние.
/// </summary>
public class GripperController : MonoBehaviour
{
    [Header("Ссылки")]
    [Tooltip("Скрипт сенсоров робота — обычно висит на корне робота")]
    [SerializeField] private VirtualSensors sensors;

    [Tooltip("Пустой дочерний объект между губками клешни")]
    [SerializeField] private Transform holdPoint;

    [Header("Настройки поиска мяча")]
    [Tooltip("Тег мяча")]
    [SerializeField] private string ballTag = "TargetBall";

    [Tooltip("Радиус поиска мяча вокруг HoldPoint при попытке захвата, м")]
    [SerializeField] private float grabSearchRadius = 0.15f;

    [Header("Управление")]
    [Tooltip("Клавиша схватить / отпустить")]
    [SerializeField] private KeyCode gripKey = KeyCode.Space;

    // ---- Состояние ----
    private GameObject heldBall;
    private Rigidbody heldRb;
    private Collider heldCol;
    private Transform heldOriginalParent;
    private bool wasKinematic;
    private bool wasColliderEnabled;

    /// <summary>Держит ли клешня сейчас мяч.</summary>
    public bool IsHolding => heldBall != null;

    private void Update()
    {
        if (Input.GetKeyDown(gripKey))
        {
            if (IsHolding) Release();
            else TryGrip();
        }
    }

    // ---- Внешний API, если управляешь из другого скрипта / нейросети ----
    public void GripCommand()
    {
        if (!IsHolding) TryGrip();
    }

    public void ReleaseCommand()
    {
        if (IsHolding) Release();
    }

    // ---- Логика ----
    private void TryGrip()
    {
        if (holdPoint == null)
        {
            Debug.LogWarning("[GripperController] HoldPoint не назначен.");
            return;
        }
        if (sensors == null)
        {
            Debug.LogWarning("[GripperController] Ссылка на VirtualSensors не назначена.");
            return;
        }
        if (sensors.GripperIR == 0) return; // мяча в клешне нет

        // Ищем ближайший мяч в радиусе вокруг HoldPoint
        Collider[] hits = Physics.OverlapSphere(holdPoint.position, grabSearchRadius);
        GameObject closest = null;
        float closestSqr = float.MaxValue;

        foreach (var c in hits)
        {
            if (!c.CompareTag(ballTag)) continue;
            float d = (c.transform.position - holdPoint.position).sqrMagnitude;
            if (d < closestSqr)
            {
                closestSqr = d;
                closest = c.gameObject;
            }
        }

        if (closest != null) Attach(closest);
    }

    private void Attach(GameObject ball)
    {
        heldBall = ball;
        heldRb = ball.GetComponent<Rigidbody>();
        heldCol = ball.GetComponent<Collider>();
        heldOriginalParent = ball.transform.parent;

        // Запоминаем и переключаем физику
        if (heldRb != null)
        {
            wasKinematic = heldRb.isKinematic;
            heldRb.linearVelocity = Vector3.zero;   // в Unity < 6 — velocity
            heldRb.angularVelocity = Vector3.zero;
            heldRb.isKinematic = true;
        }
        if (heldCol != null)
        {
            wasColliderEnabled = heldCol.enabled;
            heldCol.enabled = false;
        }

        // Крепим к точке
        ball.transform.SetParent(holdPoint, worldPositionStays: false);
        ball.transform.localPosition = Vector3.zero;
        ball.transform.localRotation = Quaternion.identity;
    }

    private void Release()
    {
        if (heldBall == null) return;

        heldBall.transform.SetParent(heldOriginalParent, worldPositionStays: true);

        if (heldCol != null) heldCol.enabled = wasColliderEnabled;
        if (heldRb != null) heldRb.isKinematic = wasKinematic;

        heldBall = null;
        heldRb = null;
        heldCol = null;
        heldOriginalParent = null;
    }

    private void OnDrawGizmosSelected()
    {
        if (holdPoint == null) return;
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(holdPoint.position, grabSearchRadius);
    }
}
