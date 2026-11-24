#!/usr/bin/env python3
"""Record a REAL auto run from the host (DIAGNOSTIC, 2026-11-24).

Polls the field page's /field/state (the autos publish the follower pose there since de1d2b9e) at 20 Hz into
runs/auto_<label>/pose.csv, saves /field/camera.jpg every 0.5 s into frames/, and tees `adb logcat` for the
autos' step lines (tag = the auto's class name) plus CameraPick / PollenCamera / RobotCore into logcat.txt.
Runs until --sec elapses or Ctrl-C. It never commands the robot.

  autorecord.py --label blue1 --sec 35 [--tags BlueSoloAuto,CameraPick,PollenCamera]
"""
import argparse, json, os, subprocess, time, urllib.request

BASE = "http://192.168.43.1:8080"
HERE = os.path.dirname(os.path.abspath(__file__))


def get(path, timeout=1.0):
    with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
        return r.read()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True)
    ap.add_argument("--sec", type=float, default=35)
    ap.add_argument("--tags", default="BlueSoloAuto,RedSoloAuto,CameraPick,PollenCamera")
    a = ap.parse_args()
    out = os.path.join(HERE, "runs", "auto_" + a.label)
    os.makedirs(os.path.join(out, "frames"), exist_ok=True)
    subprocess.run(["adb", "-s", "192.168.43.1:5555", "logcat", "-c"], capture_output=True)
    tags = [t + ":I" for t in a.tags.split(",")] + ["*:S"]
    lc = open(os.path.join(out, "logcat.txt"), "w")
    logcat = subprocess.Popen(["adb", "-s", "192.168.43.1:5555", "logcat", "-v", "time"] + tags, stdout=lc,
                              stderr=subprocess.STDOUT)
    pose = open(os.path.join(out, "pose.csv"), "w")
    pose.write("t,x,y,h,live,inZone,predInZone,ageMs\n")
    t0 = time.time()
    next_frame = 0.0
    n = 0
    try:
        while time.time() - t0 < a.sec:
            t = time.time() - t0
            try:
                s = json.loads(get("/field/state"))
                if "x" in s:
                    pose.write("%.3f,%.2f,%.2f,%.4f,%s,%s,%s,%s\n" % (t, s["x"], s["y"], s["h"], s.get("live"),
                               s.get("inZone"), s.get("predInZone"), s.get("ageMs")))
                    n += 1
            except Exception:
                pass
            if t >= next_frame:
                next_frame = t + 0.5
                try:
                    img = get("/field/camera.jpg", timeout=1.0)
                    with open(os.path.join(out, "frames", "%06.2f.jpg" % t), "wb") as f:
                        f.write(img)
                except Exception:
                    pass
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        pose.close()
        logcat.terminate()
        lc.close()
    print(f"recorded {n} poses, {len(os.listdir(os.path.join(out, 'frames')))} frames -> {out}")


if __name__ == "__main__":
    main()
