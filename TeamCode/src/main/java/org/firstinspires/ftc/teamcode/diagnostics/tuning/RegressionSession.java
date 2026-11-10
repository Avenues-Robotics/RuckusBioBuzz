package org.firstinspires.ftc.teamcode.diagnostics.tuning;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.modules.ShootingRegression;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;

/**
 * Shooting-regression samples (flywheel velocity that scored, at a turret-to-hive distance) and
 * their least-squares polynomial fit, one per wheel. DIAGNOSTIC ONLY - the Mechanism Tuner's
 * Regression tab. A saved fit reaches {@link ShootingRegression} only through
 * {@code mechtune.py export --only regression --write}.
 *
 * <p>Samples persist to {@code FIRST/mechtune_regression.csv} on the hub on every change and are
 * reloaded at init, so a session survives an OpMode restart or a redeploy. Every method is
 * synchronized: the web thread reads the CSV while the OpMode loop edits.
 */
final class RegressionSession {
    static final String[] WHEELS = {"pollen", "nectar"};
    static final int MAX_DEGREE = 3;
    static final File FILE = new File(AppUtil.FIRST_FOLDER, "mechtune_regression.csv");
    private static final String TAG = "MechanismTuner";
    private static final String HEADER =
            "id,wheel,distIn,velTps,measVelTps,x,y,headingDeg,alliance,hive,volts,timeMs";

    static final class Sample {
        int id;
        int wheel;
        double dist, vel, measVel, x, y, headingDeg, volts;
        String alliance, hive;
        long timeMs;
    }

    static final class Fit {
        double[] coeffs;   // raw: v = c0 + c1 d + c2 d^2 ...
        double r2 = Double.NaN, rmse = Double.NaN, dmin = Double.NaN, dmax = Double.NaN;
        int n;
        String error = "";
    }

    private final List<Sample> samples = new ArrayList<>();
    private final int[] degree = {1, 1};
    private final Fit[] fits = new Fit[2];
    /** Fit the operator saved for export, or null: {coeffs, {dmin, dmax}}. */
    private final double[][][] saved = new double[2][][];
    /** ShootingRegression's values at init, for the export "in the code" column and the chart. */
    final double[][][] shipped = {
            {ShootingRegression.POLLEN_COEFFS.clone(), ShootingRegression.POLLEN_RANGE.clone()},
            {ShootingRegression.NECTAR_COEFFS.clone(), ShootingRegression.NECTAR_RANGE.clone()}};
    private int nextId = 1;
    private String fileError = "";
    private String json;

    // ------------------------------------------------------------------------- edits

    synchronized Sample add(int wheel, double dist, double vel, double measVel, double x, double y,
                            double headingDeg, String alliance, String hive, double volts) {
        Sample s = new Sample();
        s.id = nextId++;
        s.wheel = wheel;
        s.dist = dist;
        s.vel = vel;
        s.measVel = measVel;
        s.x = x;
        s.y = y;
        s.headingDeg = headingDeg;
        s.alliance = alliance;
        s.hive = hive;
        s.volts = volts;
        s.timeMs = System.currentTimeMillis();
        samples.add(s);
        changed(wheel);
        return s;
    }

    synchronized boolean delete(int id) {
        for (Iterator<Sample> it = samples.iterator(); it.hasNext(); ) {
            Sample s = it.next();
            if (s.id == id) {
                it.remove();
                changed(s.wheel);
                return true;
            }
        }
        return false;
    }

    synchronized int clear(int wheel) {
        int n = 0;
        for (Iterator<Sample> it = samples.iterator(); it.hasNext(); ) {
            if (it.next().wheel == wheel) {
                it.remove();
                n++;
            }
        }
        changed(wheel);
        return n;
    }

    synchronized void setDegree(int wheel, int d) {
        degree[wheel] = Math.max(1, Math.min(MAX_DEGREE, d));
        changed(wheel);
    }

    /** Saves the current fit for export. Returns null on success, else why not. */
    synchronized String save(int wheel) {
        Fit f = fits[wheel];
        if (f == null || f.coeffs == null) {
            return f == null ? "no fit" : f.error;
        }
        saved[wheel] = new double[][]{round6(f.coeffs), {roundTo(f.dmin, 1), roundTo(f.dmax, 1)}};
        changed(wheel);
        return null;
    }

    synchronized void unsave(int wheel) {
        saved[wheel] = null;
        changed(wheel);
    }

    synchronized Fit fit(int wheel) {
        return fits[wheel];
    }

    /** Velocity the live fit gives at d (distance clamped to the data range), or NaN with no fit. */
    synchronized double fitVelocity(int wheel, double d) {
        Fit f = fits[wheel];
        if (f == null || f.coeffs == null) return Double.NaN;
        return ShootingRegression.evaluate(f.coeffs, new double[]{f.dmin, f.dmax}, d);
    }

    /** Export right-hand sides: the saved fit, or the shipped value when nothing was saved. */
    synchronized String coeffSource(int wheel) {
        return arraySource(saved[wheel] != null ? saved[wheel][0] : shipped[wheel][0]);
    }

    synchronized String rangeSource(int wheel) {
        return arraySource(saved[wheel] != null ? saved[wheel][1] : shipped[wheel][1]);
    }

    String shippedCoeffSource(int wheel) {
        return arraySource(shipped[wheel][0]);
    }

    String shippedRangeSource(int wheel) {
        return arraySource(shipped[wheel][1]);
    }

    private void changed(int wheel) {
        refit(wheel);
        json = null;
        persist();
    }

    private void refit(int wheel) {
        List<double[]> pts = new ArrayList<>();
        for (Sample s : samples) {
            if (s.wheel == wheel) pts.add(new double[]{s.dist, s.vel});
        }
        fits[wheel] = fitPolynomial(pts, degree[wheel]);
    }

    // ------------------------------------------------------------------------- fit

    /**
     * Ordinary least squares polynomial through (distance, velocity). Distances are scaled by
     * their largest magnitude before the normal equations are formed (d^3 at 150 in would
     * otherwise be 3e6 against 1), then the coefficients are scaled back to raw inches.
     */
    static Fit fitPolynomial(List<double[]> pts, int degree) {
        Fit f = new Fit();
        f.n = pts.size();
        int m = degree + 1;
        List<Double> distinct = new ArrayList<>();
        double scale = 0;
        f.dmin = Double.POSITIVE_INFINITY;
        f.dmax = Double.NEGATIVE_INFINITY;
        for (double[] p : pts) {
            scale = Math.max(scale, Math.abs(p[0]));
            f.dmin = Math.min(f.dmin, p[0]);
            f.dmax = Math.max(f.dmax, p[0]);
            boolean seen = false;
            for (double d : distinct) {
                if (Math.abs(d - p[0]) < 0.5) {
                    seen = true;
                    break;
                }
            }
            if (!seen) distinct.add(p[0]);
        }
        if (distinct.size() < m) {
            f.error = "need samples at " + m + " different distances (0.5 in apart) for degree " + degree
                    + "; have " + distinct.size();
            if (pts.isEmpty()) {
                f.dmin = f.dmax = Double.NaN;
            }
            return f;
        }
        double[][] a = new double[m][m + 1];
        for (double[] p : pts) {
            double x = p[0] / scale;
            double[] pw = new double[2 * m];
            pw[0] = 1;
            for (int k = 1; k < pw.length; k++) pw[k] = pw[k - 1] * x;
            for (int r = 0; r < m; r++) {
                for (int c = 0; c < m; c++) a[r][c] += pw[r + c];
                a[r][m] += pw[r] * p[1];
            }
        }
        double[] sol = solve(a);
        if (sol == null) {
            f.error = "singular fit (distances too close together for this degree)";
            return f;
        }
        f.coeffs = new double[m];
        for (int k = 0; k < m; k++) f.coeffs[k] = sol[k] / Math.pow(scale, k);

        double mean = 0;
        for (double[] p : pts) mean += p[1];
        mean /= pts.size();
        double ssRes = 0, ssTot = 0;
        for (double[] p : pts) {
            double e = p[1] - poly(f.coeffs, p[0]);
            ssRes += e * e;
            ssTot += (p[1] - mean) * (p[1] - mean);
        }
        f.rmse = Math.sqrt(ssRes / pts.size());
        f.r2 = ssTot > 0 ? 1 - ssRes / ssTot : Double.NaN;
        return f;
    }

    static double poly(double[] c, double d) {
        double v = 0;
        for (int i = c.length - 1; i >= 0; i--) v = v * d + c[i];
        return v;
    }

    /** Gaussian elimination with partial pivoting on an augmented m x (m+1) matrix. */
    private static double[] solve(double[][] a) {
        int m = a.length;
        for (int col = 0; col < m; col++) {
            int piv = col;
            for (int r = col + 1; r < m; r++) {
                if (Math.abs(a[r][col]) > Math.abs(a[piv][col])) piv = r;
            }
            if (Math.abs(a[piv][col]) < 1e-12) return null;
            double[] t = a[col];
            a[col] = a[piv];
            a[piv] = t;
            for (int r = 0; r < m; r++) {
                if (r == col) continue;
                double k = a[r][col] / a[col][col];
                for (int c = col; c <= m; c++) a[r][c] -= k * a[col][c];
            }
        }
        double[] x = new double[m];
        for (int i = 0; i < m; i++) x[i] = a[i][m] / a[i][i];
        return x;
    }

    // ------------------------------------------------------------------------- persistence

    synchronized void load() {
        samples.clear();
        if (!FILE.exists()) {
            for (int w = 0; w < 2; w++) refit(w);
            return;
        }
        try (BufferedReader in = new BufferedReader(new FileReader(FILE))) {
            String line;
            while ((line = in.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("id,")) continue;
                String[] p = line.split(",");
                if (line.startsWith("#degree")) {          // #degree,pollen,2
                    degree[wheelIndex(p[1])] = Integer.parseInt(p[2].trim());
                } else if (line.startsWith("#saved")) {    // #saved,pollen,dmin,dmax,c0,c1,...
                    int w = wheelIndex(p[1]);
                    double[] c = new double[p.length - 4];
                    for (int i = 0; i < c.length; i++) c[i] = Double.parseDouble(p[4 + i]);
                    saved[w] = new double[][]{c, {Double.parseDouble(p[2]), Double.parseDouble(p[3])}};
                } else if (!line.startsWith("#") && p.length >= 12) {
                    Sample s = new Sample();
                    s.id = Integer.parseInt(p[0]);
                    s.wheel = wheelIndex(p[1]);
                    s.dist = Double.parseDouble(p[2]);
                    s.vel = Double.parseDouble(p[3]);
                    s.measVel = Double.parseDouble(p[4]);
                    s.x = Double.parseDouble(p[5]);
                    s.y = Double.parseDouble(p[6]);
                    s.headingDeg = Double.parseDouble(p[7]);
                    s.alliance = p[8];
                    s.hive = p[9];
                    s.volts = Double.parseDouble(p[10]);
                    s.timeMs = Long.parseLong(p[11]);
                    samples.add(s);
                    nextId = Math.max(nextId, s.id + 1);
                }
            }
            fileError = "";
        } catch (IOException | RuntimeException e) {
            fileError = "could not read " + FILE.getName() + ": " + e.getMessage();
            RobotLog.ee(TAG, e, "Regression samples unreadable");
        }
        for (int w = 0; w < 2; w++) refit(w);
        json = null;
    }

    private void persist() {
        try (Writer out = new FileWriter(FILE)) {
            out.write(csvLocked());
            fileError = "";
        } catch (IOException e) {
            fileError = "could not write " + FILE.getName() + ": " + e.getMessage();
        }
    }

    synchronized String csv() {
        return csvLocked();
    }

    private String csvLocked() {
        StringBuilder sb = new StringBuilder(256 + 96 * samples.size());
        for (int w = 0; w < 2; w++) {
            sb.append("#degree,").append(WHEELS[w]).append(',').append(degree[w]).append('\n');
            if (saved[w] != null) {
                sb.append("#saved,").append(WHEELS[w]).append(',').append(saved[w][1][0]).append(',')
                        .append(saved[w][1][1]);
                for (double c : saved[w][0]) sb.append(',').append(c);
                sb.append('\n');
            }
        }
        sb.append(HEADER).append('\n');
        for (Sample s : samples) {
            sb.append(s.id).append(',').append(WHEELS[s.wheel]).append(',').append(s.dist).append(',')
                    .append(s.vel).append(',').append(s.measVel).append(',').append(s.x).append(',')
                    .append(s.y).append(',').append(s.headingDeg).append(',').append(s.alliance)
                    .append(',').append(s.hive).append(',').append(s.volts).append(',')
                    .append(s.timeMs).append('\n');
        }
        return sb.toString();
    }

    // ------------------------------------------------------------------------- state JSON

    /** Rebuilt only after a change; the OpMode publishes at 20 Hz. */
    synchronized String json() {
        if (json != null) return json;
        StringBuilder sb = new StringBuilder(1024 + 160 * samples.size());
        sb.append("{\"file\":\"").append(TuningHub.esc(FILE.getPath()))
                .append("\",\"fileError\":\"").append(TuningHub.esc(fileError))
                .append("\",\"maxDegree\":").append(MAX_DEGREE)
                .append(",\"wheels\":[");
        for (int w = 0; w < 2; w++) {
            if (w > 0) sb.append(',');
            Fit f = fits[w];
            sb.append("{\"name\":\"").append(WHEELS[w])
                    .append("\",\"degree\":").append(degree[w])
                    .append(",\"fit\":{\"coeffs\":").append(f == null ? "null" : arrayJson(f.coeffs))
                    .append(",\"n\":").append(f == null ? 0 : f.n)
                    .append(",\"r2\":").append(f == null ? "null" : num(f.r2))
                    .append(",\"rmse\":").append(f == null ? "null" : num(f.rmse))
                    .append(",\"dmin\":").append(f == null ? "null" : num(f.dmin))
                    .append(",\"dmax\":").append(f == null ? "null" : num(f.dmax))
                    .append(",\"error\":\"").append(f == null ? "" : TuningHub.esc(f.error))
                    .append("\"},\"saved\":");
            if (saved[w] == null) {
                sb.append("null");
            } else {
                sb.append("{\"coeffs\":").append(arrayJson(saved[w][0]))
                        .append(",\"range\":").append(arrayJson(saved[w][1])).append('}');
            }
            sb.append(",\"shipped\":{\"coeffs\":").append(arrayJson(shipped[w][0]))
                    .append(",\"range\":").append(arrayJson(shipped[w][1])).append("}}");
        }
        sb.append("],\"samples\":[");
        boolean first = true;
        for (Sample s : samples) {
            if (!first) sb.append(',');
            first = false;
            Fit f = fits[s.wheel];
            double resid = f != null && f.coeffs != null ? s.vel - poly(f.coeffs, s.dist) : Double.NaN;
            sb.append("{\"id\":").append(s.id)
                    .append(",\"wheel\":\"").append(WHEELS[s.wheel])
                    .append("\",\"dist\":").append(num(roundTo(s.dist, 2)))
                    .append(",\"vel\":").append(num(roundTo(s.vel, 1)))
                    .append(",\"meas\":").append(num(roundTo(s.measVel, 1)))
                    .append(",\"resid\":").append(num(roundTo(resid, 1)))
                    .append(",\"x\":").append(num(roundTo(s.x, 2)))
                    .append(",\"y\":").append(num(roundTo(s.y, 2)))
                    .append(",\"h\":").append(num(roundTo(s.headingDeg, 1)))
                    .append(",\"alliance\":\"").append(TuningHub.esc(s.alliance))
                    .append("\",\"hive\":\"").append(TuningHub.esc(s.hive))
                    .append("\",\"volts\":").append(num(roundTo(s.volts, 2)))
                    .append(",\"t\":").append(s.timeMs).append('}');
        }
        json = sb.append("]}").toString();
        return json;
    }

    // ------------------------------------------------------------------------- helpers

    static int wheelIndex(String name) {
        if ("pollen".equals(name)) return 0;
        if ("nectar".equals(name)) return 1;
        throw new IllegalArgumentException("wheel= must be pollen or nectar");
    }

    private static String num(double v) {
        return Double.isNaN(v) || Double.isInfinite(v) ? "null" : Double.toString(v);
    }

    private static String arrayJson(double[] v) {
        if (v == null) return "null";
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < v.length; i++) {
            if (i > 0) sb.append(',');
            sb.append(num(v[i]));
        }
        return sb.append(']').toString();
    }

    /** Java array initializer, whole numbers without ".0" so the splicer's comparison stays quiet. */
    static String arraySource(double[] v) {
        StringBuilder sb = new StringBuilder("{");
        for (int i = 0; i < v.length; i++) {
            if (i > 0) sb.append(", ");
            double x = v[i];
            sb.append(x == Math.rint(x) && Math.abs(x) < 1e9 ? Long.toString((long) x) : Double.toString(x));
        }
        return sb.append('}').toString();
    }

    private static double roundTo(double v, int digits) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return v;
        double s = Math.pow(10, digits);
        return Math.round(v * s) / s;
    }

    /** Six significant figures: enough for a ticks/s curve, short enough to read in a diff. */
    private static double[] round6(double[] v) {
        double[] out = new double[v.length];
        for (int i = 0; i < v.length; i++) {
            out[i] = v[i] == 0 ? 0 : Double.parseDouble(String.format(Locale.US, "%.6g", v[i]));
        }
        return out;
    }
}
