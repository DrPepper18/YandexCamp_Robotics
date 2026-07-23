using System;
using System.Net;
using System.Threading;
using UnityEngine;

/// <summary>
/// Competition/demo "start" trigger: listens for an HTTP POST on the address configured
/// in the Inspector, and on the first one received, calls RobotBrain.StartMission() -
/// which turns missionMode on and (via RobotBrain's own actuation gate in
/// OnActionReceived) lets the robot actually start applying its inferred actions.
/// Before that POST arrives, the robot sits motionless even though its ONNX model is
/// already running inference every decision step.
///
/// IMPORTANT: Unity runs this on Mono's own HttpListener reimplementation, not Windows'
/// native HTTP.SYS-backed one - it does plain socket binding (no Administrator/netsh
/// urlacl needed for any host form), but its Host-header validation gets confused if
/// multiple specific prefixes (e.g. a literal IP AND "localhost") are registered on the
/// same listener at once - requests can fail with "Bad Request (Invalid host)" even
/// though something IS listening. Binding the wildcard prefix "+" (any Host header, any
/// interface) sidesteps that entirely and is what's used here - it works for both local
/// curl testing (localhost/127.0.0.1) and real requests to the machine's LAN IP alike.
/// The ipAddress field below is informational only (shown in the log message) - it does
/// NOT change what gets bound.
/// </summary>
public class MissionHttpTrigger : MonoBehaviour
{
    [Header("Listen address (informational only - see the wildcard-bind note above)")]
    [SerializeField] private string ipAddress = "192.168.2.152";
    [SerializeField] private int port = 5000;

    [Header("Target")]
    [Tooltip("Leave empty to auto-find the first RobotBrain in the scene")]
    [SerializeField] private RobotBrain robotBrain;

    [Header("Debug")]
    [SerializeField] private bool logRequests = true;

    private HttpListener listener;
    private Thread listenerThread;
    private volatile bool triggerReceived;
    private volatile bool running;

    private void Start()
    {
        if (robotBrain == null) robotBrain = FindFirstObjectByType<RobotBrain>();

        string prefix = $"http://+:{port}/";
        try
        {
            listener = new HttpListener();
            listener.Prefixes.Add(prefix);
            listener.Start();
            running = true;
            listenerThread = new Thread(ListenLoop) { IsBackground = true };
            listenerThread.Start();
            Debug.Log($"[MissionHttpTrigger] Listening for POST on {prefix} " +
                      $"(reachable via localhost:{port}, 127.0.0.1:{port}, or {ipAddress}:{port})");
        }
        catch (Exception e)
        {
            Debug.LogError(
                $"[MissionHttpTrigger] Failed to start listener on {prefix}: {e.Message}");
        }
    }

    // Runs on a background thread - HttpListener.GetContext() blocks until a request
    // arrives. Never touch Unity APIs/objects directly from here; only set the volatile
    // flag and let Update() (main thread) act on it.
    private void ListenLoop()
    {
        while (running && listener != null && listener.IsListening)
        {
            HttpListenerContext ctx;
            try
            {
                ctx = listener.GetContext();
            }
            catch (Exception)
            {
                break; // listener was stopped/disposed (e.g. on scene exit)
            }

            if (ctx.Request.HttpMethod == "POST")
            {
                triggerReceived = true;
                if (logRequests) Debug.Log("[MissionHttpTrigger] POST received - mission start triggered");
            }
            else if (logRequests)
            {
                Debug.Log($"[MissionHttpTrigger] Ignored {ctx.Request.HttpMethod} request (only POST triggers the mission)");
            }

            ctx.Response.StatusCode = 200;
            ctx.Response.Close();
        }
    }

    private void Update()
    {
        if (triggerReceived)
        {
            triggerReceived = false;
            if (robotBrain != null) robotBrain.StartMission();
        }
    }

    private void OnDestroy()
    {
        running = false;
        try
        {
            listener?.Stop();
            listener?.Close();
        }
        catch (Exception)
        {
            // Listener may already be in a bad state if Start() failed - safe to ignore.
        }
    }
}
