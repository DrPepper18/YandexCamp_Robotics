using UnityEngine;

/// <summary>
/// Отладочный HUD для VisualServoBrain и SimulatedYoloCamera.
/// Выводит текущий State робота, параметры YOLO Bounding Box,
/// состояние датчиков и глубину LIFO-стека истории движения.
/// </summary>
public class BrainDebugHUD : MonoBehaviour
{
    [Header("Ссылки на компоненты")]
    [SerializeField] private VisualServoBrain brain;
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private VirtualSensors sensors;
    [SerializeField] private GripperController gripper;

    [Header("Внешний вид")]
    [SerializeField] private int fontSize = 14;
    [SerializeField] private Vector2 position = new Vector2(20, 20);
    [SerializeField] private float panelWidth = 340f;
    [SerializeField] private Color textColor = Color.white;
    [SerializeField] private Color bgColor = new Color(0f, 0f, 0f, 0.75f);

    private GUIStyle style;
    private Texture2D bgTexture;

    private void OnGUI()
    {
        // Гарантируем правильную настройку стиля с поддержкой RichText
        if (style == null)
        {
            style = new GUIStyle(GUI.skin.label)
            {
                fontSize = fontSize,
                richText = true, // ОБЯЗАТЕЛЬНО
                alignment = TextAnchor.UpperLeft
            };
            style.normal.textColor = textColor;
        }

        if (bgTexture == null)
        {
            bgTexture = new Texture2D(1, 1);
            bgTexture.SetPixel(0, 0, bgColor);
            bgTexture.Apply();
        }

        var sb = new System.Text.StringBuilder();

        // 1. STATES
        string stateStr = brain != null ? brain.CurrentState.ToString() : "N/A";
        string stateColor = brain != null ? GetStateColor(brain.CurrentState) : "red";
        
        sb.AppendLine("<b>SYSTEM STATES</b>");
        sb.AppendLine(Row("Robot State", stateStr, stateColor));

        bool isVisible = yoloCamera != null && yoloCamera.IsVisible;
        sb.AppendLine(Row("Ball Visible", isVisible ? "YES" : "NO", isVisible ? "lime" : "red"));

        // Проверяем и ИК-датчик, и сам контроллер клешни
        bool hasBall = (sensors != null && sensors.GripperIR == 1) || (gripper != null && gripper.IsHolding);
        sb.AppendLine(Row("Gripper IR", hasBall ? "BALL GRABBED" : "EMPTY", hasBall ? "lime" : "gray"));

        sb.AppendLine();

        // 2. YOLO
        sb.AppendLine("<b>YOLO METRICS</b>");
        float bboxSize = yoloCamera != null ? yoloCamera.BboxSize : 0f;
        float relAngle = yoloCamera != null ? yoloCamera.RelativeAngle : 0f;
        sb.AppendLine(Row("BBox Size", Fmt(bboxSize), isVisible ? "cyan" : "gray"));
        sb.AppendLine(Row("Relative Angle", FmtSigned(relAngle), isVisible ? "cyan" : "gray"));

        sb.AppendLine();

        // 3. CONTROL
        sb.AppendLine("<b>CONTROL & HISTORY</b>");
        int historyCount = brain != null ? brain.ActionHistory.Count : 0;
        
        // Берем актуальные значения прямо из камеры / брейна
        float gas = brain != null ? brain.dbg_gas : 0f;
        float steer = brain != null ? brain.dbg_steer : 0f;

        sb.AppendLine(Row("Gas Cmd", FmtSigned(gas), "yellow"));
        sb.AppendLine(Row("Steer Cmd", FmtSigned(steer), "yellow"));
        sb.AppendLine(Row("Action History", $"{historyCount} frames", historyCount > 0 ? "white" : "gray"));

        string text = sb.ToString();

        // Отрисовка
        float height = 240f; // Фиксированная высота, чтобы ничего не прыгало
        Rect box = new Rect(position.x, position.y, panelWidth, height);
        
        GUI.DrawTexture(box, bgTexture);
        GUI.Label(new Rect(box.x + 10, box.y + 10, box.width - 20, box.height - 20), text, style);
    }

    // ---- Хелперы форматирования ----
    private static string Row(string label, string value, string color)
    {
        return $"{label} <color={color}>{value}</color>";
    }

    private static string Fmt(float v) => v.ToString("F3");
    private static string FmtSigned(float v) => v.ToString("+0.000;-0.000; 0.000");

    private static string GetStateColor(VisualServoBrain.State state)
    {
        switch (state)
        {
            case VisualServoBrain.State.Searching: return "yellow";
            case VisualServoBrain.State.Approaching: return "cyan";
            case VisualServoBrain.State.Grabbing: return "orange";
            case VisualServoBrain.State.Returning: return "magenta";
            case VisualServoBrain.State.Done: return "lime";
            default: return "white";
        }
    }
}