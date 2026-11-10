package org.firstinspires.ftc.teamcode.diagnostics.tuning;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Thread-safe hand-off between the {@link MechanismTuner} OpMode and the {@code /tune} web routes,
 * plus the operator notification board.
 *
 * <p>The notification board lives at app scope, not OpMode scope: a host session can ask the
 * operator for something ("start the Mechanism Tuner", "feed a pollen") whether or not any OpMode
 * is running, and the swerve dashboard shows the same board through {@code /tune/notify.js}.
 */
public final class TuningHub {
    public static final TuningHub INSTANCE = new TuningHub();

    /** A snapshot is considered live if the OpMode published within this window. */
    private static final long LIVE_WINDOW_MS = 1500;
    private static final int MAX_PENDING_COMMANDS = 64;
    private static final int MAX_NOTES = 40;

    private final AtomicReference<String> stateJson = new AtomicReference<>("{\"live\":false}");
    private final ConcurrentLinkedQueue<Map<String, String>> commands = new ConcurrentLinkedQueue<>();
    private final AtomicReference<FlyRecorder> recorder = new AtomicReference<>();
    private final AtomicReference<RegressionSession> regression = new AtomicReference<>();
    private final AtomicReference<String> colorCsv = new AtomicReference<>("label,r,g,b,a,dist\n");
    private volatile long lastPublishMs;
    private volatile long lastClientMs;
    private long commandSeq;

    private final List<Note> notes = new ArrayList<>();
    private long noteSeq;

    private TuningHub() {
    }

    // ---------------------------------------------------------------- OpMode state and commands

    public void publish(String json) {
        stateJson.set(json);
        lastPublishMs = System.currentTimeMillis();
    }

    public String state() {
        return stateJson.get();
    }

    public boolean isLive() {
        return System.currentTimeMillis() - lastPublishMs < LIVE_WINDOW_MS;
    }

    /** Queues a command and returns its sequence number; the OpMode echoes it back as cmdSeq. */
    public synchronized long submit(Map<String, String> command) {
        long seq = ++commandSeq;
        command.put("_seq", Long.toString(seq));
        if (commands.size() >= MAX_PENDING_COMMANDS) {
            commands.poll();
        }
        commands.add(command);
        return seq;
    }

    public Map<String, String> poll() {
        return commands.poll();
    }

    public void clearCommands() {
        commands.clear();
    }

    public void markStopped() {
        lastPublishMs = 0;
        commands.clear();
    }

    /** Any web client reading state or sending a command counts as someone watching. */
    public void touchClient() {
        lastClientMs = System.currentTimeMillis();
    }

    public long msSinceClient() {
        return System.currentTimeMillis() - lastClientMs;
    }

    public void setRecorder(FlyRecorder r) {
        recorder.set(r);
    }

    public FlyRecorder recorder() {
        return recorder.get();
    }

    void setRegression(RegressionSession r) {
        regression.set(r);
    }

    RegressionSession regression() {
        return regression.get();
    }

    public void setColorCsv(String csv) {
        colorCsv.set(csv);
    }

    public String colorCsv() {
        return colorCsv.get();
    }

    // ---------------------------------------------------------------- notifications

    /** One message to the operator. Immutable apart from its resolution. */
    static final class Note {
        final long id;
        final String text;
        final String level;   // info | action | warn
        final String await;   // ack | none | shot:pollen | shot:nectar
        final String from;
        final long postedMs;
        boolean resolved;
        String reply = "";
        String resolution = "";
        long resolvedMs;

        Note(long id, String text, String level, String await, String from) {
            this.id = id;
            this.text = text;
            this.level = level;
            this.await = await;
            this.from = from;
            this.postedMs = System.currentTimeMillis();
            // An info note asks nothing of anyone; it is born resolved and only shows in history.
            this.resolved = "none".equals(await);
        }
    }

    public synchronized long notify(String text, String level, String await, String from) {
        Note n = new Note(++noteSeq, text, level, await, from);
        notes.add(n);
        while (notes.size() > MAX_NOTES) {
            notes.remove(0);
        }
        return n.id;
    }

    /** Operator answered from a page. Returns false for an unknown or already-resolved id. */
    public synchronized boolean ack(long id, String reply, String resolution) {
        for (Note n : notes) {
            if (n.id == id && !n.resolved) {
                n.resolved = true;
                n.reply = reply == null ? "" : reply;
                n.resolution = resolution == null ? "done" : resolution;
                n.resolvedMs = System.currentTimeMillis();
                return true;
            }
        }
        return false;
    }

    /** Resolves every open note, e.g. when a session ends. */
    public synchronized int dismissAll(String resolution) {
        int count = 0;
        for (Note n : notes) {
            if (!n.resolved) {
                n.resolved = true;
                n.resolution = resolution;
                n.resolvedMs = System.currentTimeMillis();
                count++;
            }
        }
        return count;
    }

    /**
     * Called by the OpMode when an awaited event happens (a shot dip on a wheel). Resolves the
     * oldest open note waiting on that key, so the operator never has to press anything with a
     * pollen in their hand.
     */
    public synchronized void resolveAwait(String key, String resolution) {
        for (Note n : notes) {
            if (!n.resolved && key.equals(n.await)) {
                n.resolved = true;
                n.resolution = resolution;
                n.resolvedMs = System.currentTimeMillis();
                return;
            }
        }
    }

    /** The open notes the OpMode should mirror onto Driver Station telemetry. */
    public synchronized String openNoteText() {
        StringBuilder sb = new StringBuilder();
        for (Note n : notes) {
            if (!n.resolved) {
                if (sb.length() > 0) {
                    sb.append(" | ");
                }
                sb.append(n.text);
            }
        }
        return sb.toString();
    }

    public synchronized String notesJson(long since) {
        StringBuilder sb = new StringBuilder(512);
        sb.append("{\"now\":").append(System.currentTimeMillis()).append(",\"notes\":[");
        boolean first = true;
        for (Note n : notes) {
            if (n.id <= since) {
                continue;
            }
            if (!first) {
                sb.append(',');
            }
            first = false;
            sb.append("{\"id\":").append(n.id)
                    .append(",\"text\":\"").append(esc(n.text))
                    .append("\",\"level\":\"").append(esc(n.level))
                    .append("\",\"await\":\"").append(esc(n.await))
                    .append("\",\"from\":\"").append(esc(n.from))
                    .append("\",\"postedMs\":").append(n.postedMs)
                    .append(",\"resolved\":").append(n.resolved)
                    .append(",\"reply\":\"").append(esc(n.reply))
                    .append("\",\"resolution\":\"").append(esc(n.resolution))
                    .append("\",\"resolvedMs\":").append(n.resolvedMs)
                    .append('}');
        }
        return sb.append("]}").toString();
    }

    static String esc(String s) {
        if (s == null) {
            return "";
        }
        StringBuilder out = new StringBuilder(s.length() + 8);
        for (int i = 0; i < s.length(); i++) {
            char ch = s.charAt(i);
            switch (ch) {
                case '"': out.append("\\\""); break;
                case '\\': out.append("\\\\"); break;
                case '\n': out.append("\\n"); break;
                case '\r': out.append("\\r"); break;
                case '\t': out.append("\\t"); break;
                default:
                    if (ch < 0x20) {
                        out.append(' ');
                    } else {
                        out.append(ch);
                    }
            }
        }
        return out.toString();
    }
}
