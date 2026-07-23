using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(Camera))]
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("Target detection")]
    public string targetTag = "TargetBall";
    public float targetDiameterMeters = 0.06f;
    public Transform manualTarget;

    [Header("Field of view / range")]
    public float hFov = 40f;
    public float maxViewDistance = 2f;

    [Header("Line of sight")]
    public LayerMask obstacleMask = ~0;

    [Header("Nearest-target selection")]
    public float centerBiasWeight = 0.15f;

    private Camera cam;

    public bool IsVisible { get; private set; }
    public float RelativeAngle { get; private set; }
    public float NormalizedDistance { get; private set; } = 1f;
    public float BboxSize { get; private set; }
    public Transform CurrentTarget { get; private set; }

    private readonly List<Transform> candidateBuffer = new List<Transform>();

    private void Awake()
    {
        cam = GetComponent<Camera>();
    }

    private void Update()
    {
        UpdateDetection();
    }

    private void UpdateDetection()
    {
        if (cam == null)
        {
            SetNotVisible();
            return;
        }

        candidateBuffer.Clear();

        if (manualTarget != null)
        {
            candidateBuffer.Add(manualTarget);
        }
        else if (!string.IsNullOrEmpty(targetTag))
        {
            GameObject[] found = GameObject.FindGameObjectsWithTag(targetTag);
            for (int i = 0; i < found.Length; i++)
            {
                candidateBuffer.Add(found[i].transform);
            }
        }

        if (candidateBuffer.Count == 0)
        {
            SetNotVisible();
            return;
        }

        bool foundAny = false;
        float bestScore = float.NegativeInfinity;
        Transform bestTarget = null;
        float bestOffset = 0f, bestNormDist = 1f, bestBbox = 0f;

        for (int i = 0; i < candidateBuffer.Count; i++)
        {
            Transform candidate = candidateBuffer[i];
            if (candidate == null) continue;

            if (!TryEvaluateCandidate(candidate, out float offset, out float normDist, out float bboxSize))
            {
                continue;
            }

            foundAny = true;

            float score = bboxSize - centerBiasWeight * Mathf.Abs(offset);

            if (score > bestScore)
            {
                bestScore = score;
                bestTarget = candidate;
                bestOffset = offset;
                bestNormDist = normDist;
                bestBbox = bboxSize;
            }
        }

        if (!foundAny)
        {
            SetNotVisible();
            return;
        }

        IsVisible = true;
        CurrentTarget = bestTarget;
        RelativeAngle = bestOffset;
        NormalizedDistance = bestNormDist;
        BboxSize = bestBbox;
    }

    private bool TryEvaluateCandidate(Transform candidate, out float offset, out float normDist, out float bboxSize)
    {
        offset = 0f;
        normDist = 1f;
        bboxSize = 0f;

        Vector3 toTarget = candidate.position - cam.transform.position;
        float distance = toTarget.magnitude;

        // 1) Range check
        if (distance > maxViewDistance || distance < 0.05f) // Не считаем объекты ближе 5 см
        {
            return false;
        }

        // 2) Viewport Check - Проверяем, попадает ли объект в экран камеры
        Vector3 viewportPoint = cam.WorldToViewportPoint(candidate.position);
        
        // Z <= 0 означает, что объект НАЗАДИ камеры
        if (viewportPoint.z <= 0)
        {
            return false;
        }

        // Вычисляем offset (-1..1) строго через Viewport, без математических проекций
        offset = Mathf.Clamp((viewportPoint.x - 0.5f) * 2f, -1f, 1f);

        // 3) FOV Check
        // Если объект вылетел за пределы экрана по горизонтали
        if (Mathf.Abs(offset) > 1.0f)
        {
            return false;
        }

        // 4) Line-of-sight check
        // Пускаем луч с небольшим отступом (0.05f), чтобы не врезаться в линзу/корпус своей же камеры
        if (Physics.Raycast(cam.transform.position + cam.transform.forward * 0.05f, toTarget.normalized, out RaycastHit hit, distance, obstacleMask))
        {
            // Если луч попал НЕ в целевой мяч и НЕ в родительский холдер мяча
            if (hit.transform != candidate && !hit.transform.IsChildOf(candidate))
            {
                return false; 
            }
        }

        // 5) Normalized distance & Bbox size
        normDist = Mathf.Clamp01(distance / maxViewDistance);

        float angularSizeRad = 2f * Mathf.Atan((targetDiameterMeters * 0.5f) / distance);
        float angularSizeDeg = angularSizeRad * Mathf.Rad2Deg;
        bboxSize = Mathf.Clamp01(angularSizeDeg / Mathf.Max(1f, cam.fieldOfView));

        return true;
    }

    private void SetNotVisible()
    {
        IsVisible = false;
        RelativeAngle = 0f;
        NormalizedDistance = 1f;
        BboxSize = 0f;
        CurrentTarget = null;
    }

    private void OnDrawGizmosSelected()
    {
        if (cam == null) cam = GetComponent<Camera>();
        if (cam == null) return;

        Gizmos.color = Color.yellow;
        Vector3 origin = cam.transform.position;
        Quaternion leftRot = Quaternion.AngleAxis(-hFov * 0.5f, Vector3.up);
        Quaternion rightRot = Quaternion.AngleAxis(hFov * 0.5f, Vector3.up);

        Vector3 leftDir = leftRot * cam.transform.forward;
        Vector3 rightDir = rightRot * cam.transform.forward;

        Gizmos.DrawLine(origin, origin + leftDir * maxViewDistance);
        Gizmos.DrawLine(origin, origin + rightDir * maxViewDistance);
        Gizmos.DrawLine(origin, origin + cam.transform.forward * maxViewDistance);

        if (IsVisible && CurrentTarget != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(origin, CurrentTarget.position);
        }
    }
}