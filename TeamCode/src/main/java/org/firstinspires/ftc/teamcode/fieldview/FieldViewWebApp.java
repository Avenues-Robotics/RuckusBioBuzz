package org.firstinspires.ftc.teamcode.fieldview;

import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

import fi.iki.elonen.NanoHTTPD;

/** Serves the field view at http://192.168.43.1:8080/field on the Robot Controller's web server. */
public class FieldViewWebApp {
    private static final String TAG = "FieldView";

    private static volatile byte[] page;
    private static volatile byte[] fieldImage;

    private FieldViewWebApp() {}

    @WebHandlerRegistrar
    public static void attachWebServer(Context context, WebHandlerManager manager) {
        try {
            WebHandler pageHandler = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    return bytes("text/html", page == null ? (page = load("fieldview.html")) : page);
                }
            };
            WebHandler stateHandler = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    return noCache(NanoHTTPD.newFixedLengthResponse(
                        NanoHTTPD.Response.Status.OK, "application/json", FieldView.state()));
                }
            };
            WebHandler imageHandler = new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    return bytes("image/webp",
                        fieldImage == null ? (fieldImage = load("field.webp")) : fieldImage);
                }
            };

            manager.register("/field", pageHandler);
            manager.register("/field/", pageHandler);
            manager.register("/field/state", stateHandler);
            manager.register("/field/field.webp", imageHandler);
            manager.register("/field/camera.jpg", new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    byte[] frame = CameraFeed.latestJpeg();
                    if (frame == null) {
                        return noCache(NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                            "text/plain", "No camera frame - run TeleOp with SystemConstants.Camera.ENABLED"));
                    }
                    return bytes("image/jpeg", frame);
                }
            });
            manager.register("/field/camera", new WebHandler() {
                @Override
                public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) {
                    return noCache(NanoHTTPD.newFixedLengthResponse(
                        NanoHTTPD.Response.Status.OK, "application/json", CameraFeed.statsState()));
                }
            });
            RobotLog.ii(TAG, "Field view registered at /field");
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "Failed to register field view");
        }
    }

    private static NanoHTTPD.Response bytes(String mime, byte[] body) {
        if (body == null) {
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                "text/plain", "Not bundled into the APK");
        }
        return noCache(NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, mime,
            new ByteArrayInputStream(body), body.length));
    }

    private static NanoHTTPD.Response noCache(NanoHTTPD.Response response) {
        response.addHeader("Cache-Control", "no-store");
        return response;
    }

    private static byte[] load(String name) {
        try (InputStream in = FieldViewWebApp.class.getResourceAsStream(name)) {
            if (in == null) {
                return null;
            }
            ByteArrayOutputStream out = new ByteArrayOutputStream(32 * 1024);
            byte[] buffer = new byte[8192];
            int read;
            while ((read = in.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
            return out.toByteArray();
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "Could not read %s", name);
            return null;
        }
    }
}
