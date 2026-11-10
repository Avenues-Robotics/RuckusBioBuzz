package org.firstinspires.ftc.teamcode.diagnostics.tuning;

import android.content.Context;

import com.qualcomm.ftccommon.CommandList;
import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.teamcode.diagnostics.swerve.SwerveBench;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import fi.iki.elonen.NanoHTTPD;

/**
 * Serves the mechanism tuning dashboard at <b>http://192.168.43.1:8080/tune</b>.
 *
 * <p>Routes are registered at app start, so the notification board ({@code /tune/notify},
 * {@code /tune/notes}, {@code /tune/ack}) works with no OpMode running - including while the
 * Swerve Bring-Up OpMode owns the robot, whose page loads {@code /tune/notify.js}.
 *
 * <p>The SDK dispatches with an exact map lookup, so every route is registered individually.
 */
public class TuningWebApp {
    private static final String TAG = "MechanismTuner";
    private static final String MIME_JSON = "application/json";

    private static volatile String cachedPage;
    private static volatile String cachedNotifyJs;

    private TuningWebApp() {
    }

    @WebHandlerRegistrar
    public static void attachWebServer(Context context, WebHandlerManager manager) {
        try {
            WebHandler page = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    String body = cachedPage == null ? (cachedPage = load("tuning.html")) : cachedPage;
                    return text("text/html", body == null
                            ? "<h1>tuning.html was not bundled into the APK</h1>" : body);
                }
            };
            WebHandler notifyJs = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    String body = cachedNotifyJs == null
                            ? (cachedNotifyJs = load("notify.js")) : cachedNotifyJs;
                    return text("application/javascript", body == null ? "" : body);
                }
            };
            WebHandler state = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    TuningHub.INSTANCE.touchClient();
                    String body = TuningHub.INSTANCE.isLive()
                            ? TuningHub.INSTANCE.state()
                            : "{\"live\":false,\"message\":\"Run the Mechanism Tuner OpMode.\"}";
                    // Swerve liveness rides along so the page's swerve tab needs no second poll.
                    body = body.substring(0, body.length() - 1)
                            + ",\"swerveLive\":" + SwerveBench.INSTANCE.isLive() + "}";
                    return text(MIME_JSON, body);
                }
            };
            WebHandler cmd = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    TuningHub.INSTANCE.touchClient();
                    long seq = TuningHub.INSTANCE.submit(params(session));
                    return text(MIME_JSON, "{\"ok\":true,\"seq\":" + seq
                            + ",\"live\":" + TuningHub.INSTANCE.isLive() + "}");
                }
            };
            WebHandler notify = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    Map<String, String> p = params(session);
                    if (p.containsKey("clear")) {
                        int n = TuningHub.INSTANCE.dismissAll("cleared");
                        return text(MIME_JSON, "{\"ok\":true,\"cleared\":" + n + "}");
                    }
                    String body = p.get("text");
                    if (body == null || body.trim().isEmpty()) {
                        return text(MIME_JSON, "{\"ok\":false,\"message\":\"text= is required\"}");
                    }
                    long id = TuningHub.INSTANCE.notify(body.trim(),
                            orDefault(p.get("level"), "action"),
                            orDefault(p.get("await"), "ack"),
                            orDefault(p.get("from"), "claude"));
                    return text(MIME_JSON, "{\"ok\":true,\"id\":" + id + "}");
                }
            };
            WebHandler notes = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    long since = parseLong(params(session).get("since"), 0);
                    return text(MIME_JSON, TuningHub.INSTANCE.notesJson(since));
                }
            };
            WebHandler ack = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    Map<String, String> p = params(session);
                    boolean ok = TuningHub.INSTANCE.ack(parseLong(p.get("id"), -1),
                            p.get("reply"), orDefault(p.get("resolution"), "done"));
                    return text(MIME_JSON, "{\"ok\":" + ok + "}");
                }
            };
            WebHandler rec = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    FlyRecorder r = TuningHub.INSTANCE.recorder();
                    return text("text/csv", r == null
                            ? "# no recorder; run the Mechanism Tuner OpMode\n" : r.toCsv());
                }
            };
            WebHandler color = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    return text("text/csv", TuningHub.INSTANCE.colorCsv());
                }
            };

            WebHandler regCsv = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    RegressionSession r = TuningHub.INSTANCE.regression();
                    return text("text/csv", r == null
                            ? "# no regression session; run the Mechanism Tuner OpMode\n" : r.csv());
                }
            };

            WebHandler configs = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    return text(MIME_JSON, configsJson());
                }
            };
            WebHandler configActivate = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    return text(MIME_JSON, activateConfig(params(session).get("name")));
                }
            };

            manager.register("/tune", page);
            manager.register("/tune/", page);
            manager.register("/tune/notify.js", notifyJs);
            manager.register("/tune/state", state);
            manager.register("/tune/cmd", cmd);
            manager.register("/tune/notify", notify);
            manager.register("/tune/notes", notes);
            manager.register("/tune/ack", ack);
            manager.register("/tune/rec.csv", rec);
            manager.register("/tune/color.csv", color);
            manager.register("/tune/regression.csv", regCsv);
            manager.register("/tune/configs", configs);
            manager.register("/tune/configActivate", configActivate);
            RobotLog.ii(TAG, "Mechanism tuning dashboard registered at /tune");
        } catch (RuntimeException e) {
            // A failure here must never prevent the Robot Controller from starting.
            RobotLog.ee(TAG, e, "Failed to register mechanism tuning dashboard");
        }
    }

    // ---------------------------------------------------------------- robot configuration

    /** Every robot configuration the Robot Controller knows (built-in XML resources and hub files). */
    private static String configsJson() {
        try {
            RobotConfigFileManager mgr = new RobotConfigFileManager();
            RobotConfigFile active = mgr.getActiveConfig();
            StringBuilder sb = new StringBuilder(512);
            sb.append("{\"ok\":true,\"active\":\"").append(TuningHub.esc(active.getName()))
                    .append("\",\"activeDirty\":").append(active.isDirty())
                    .append(",\"files\":[");
            boolean first = true;
            for (RobotConfigFile f : mgr.getXMLFiles()) {
                if (!first) sb.append(',');
                first = false;
                sb.append("{\"name\":\"").append(TuningHub.esc(f.getName()))
                        .append("\",\"builtIn\":").append(f.isReadOnly()).append('}');
            }
            return sb.append("]}").toString();
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "Could not list robot configurations");
            return "{\"ok\":false,\"message\":\"" + TuningHub.esc(e.toString()) + "\"}";
        }
    }

    /**
     * Activates a configuration and restarts the robot, by injecting the same two commands the
     * Driver Station sends (Configure Robot → Activate, then Restart Robot), so the SDK's own
     * handlers do the work. The hardware map is only built at robot start, hence the restart: it
     * stops any running OpMode and drops this page's connection for a few seconds.
     */
    private static String activateConfig(String name) {
        if (name == null || name.trim().isEmpty()) {
            return "{\"ok\":false,\"message\":\"name= is required\"}";
        }
        try {
            RobotConfigFileManager mgr = new RobotConfigFileManager();
            RobotConfigFile target = null;
            for (RobotConfigFile f : mgr.getXMLFiles()) {
                if (f.getName().equals(name.trim())) {
                    target = f;
                    break;
                }
            }
            if (target == null) {
                return "{\"ok\":false,\"message\":\"no configuration named " + TuningHub.esc(name) + "\"}";
            }
            NetworkConnectionHandler net = NetworkConnectionHandler.getInstance();
            net.injectReceivedCommand(new Command(CommandList.CMD_ACTIVATE_CONFIGURATION,
                    RobotConfigFileManager.serializeConfig(target)));
            net.injectReceivedCommand(new Command(CommandList.CMD_RESTART_ROBOT));
            RobotLog.ii(TAG, "Activated robot configuration %s from /tune; restarting robot", target.getName());
            return "{\"ok\":true,\"message\":\"Activated " + TuningHub.esc(target.getName())
                    + ". Robot restarting.\"}";
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "Could not activate robot configuration %s", name);
            return "{\"ok\":false,\"message\":\"" + TuningHub.esc(e.toString()) + "\"}";
        }
    }

    private static Map<String, String> params(NanoHTTPD.IHTTPSession session) {
        Map<String, String> out = new HashMap<>();
        for (Map.Entry<String, List<String>> e : session.getParameters().entrySet()) {
            List<String> values = e.getValue();
            if (values != null && !values.isEmpty()) {
                out.put(e.getKey(), values.get(0));
            }
        }
        return out;
    }

    private static String orDefault(String value, String fallback) {
        return value == null || value.trim().isEmpty() ? fallback : value.trim();
    }

    private static long parseLong(String value, long fallback) {
        try {
            return value == null ? fallback : Long.parseLong(value.trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static NanoHTTPD.Response text(String mime, String body) {
        NanoHTTPD.Response response =
                NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, mime, body);
        response.addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
        // The swerve dashboard and any laptop tool may load notify.js or post notes.
        response.addHeader("Access-Control-Allow-Origin", "*");
        return response;
    }

    private static String load(String name) {
        try (InputStream in = TuningWebApp.class.getResourceAsStream(name)) {
            if (in == null) {
                return null;
            }
            ByteArrayOutputStream out = new ByteArrayOutputStream(64 * 1024);
            byte[] buffer = new byte[8192];
            int read;
            while ((read = in.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
            return out.toString("UTF-8");
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "Could not read %s", name);
            return null;
        }
    }
}
