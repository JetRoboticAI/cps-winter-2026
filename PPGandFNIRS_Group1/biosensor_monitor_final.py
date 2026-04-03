import matplotlib
matplotlib.use("TkAgg")

import serial
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from scipy.signal import butter, filtfilt, find_peaks
import csv
import time

# ===============================
# MODE SELECTION
# ===============================
print("\nSelect sensor mode:")
print("1 = Finger (BPM + SpO2)")
print("2 = Forehead (fNIRS brain monitor)")

choice = input("Enter mode (1 or 2): ").strip()

if choice == "1":
    MODE = "finger"
elif choice == "2":
    MODE = "forehead"
else:
    print("Invalid choice, defaulting to finger mode")
    MODE = "finger"

print("Running in", MODE, "mode")

# ===============================
# SERIAL
# ===============================
PORT = "COM3"     # change if needed
BAUD = 115200
WINDOW = 500

ser = serial.Serial(PORT, BAUD, timeout=1)

# ===============================
# DATA BUFFERS
# ===============================
t_ms = deque(maxlen=WINDOW)
det730 = deque(maxlen=WINDOW)   # DET2 730
det850 = deque(maxlen=WINDOW)   # DET2 850
pulse730 = deque(maxlen=WINDOW) # pulse730_det2
pulse850 = deque(maxlen=WINDOW) # pulse850_det2

# ===============================
# LOGGING
# ===============================
filename = f"biosensor_{MODE}_{int(time.time())}.csv"
log = open(filename, "w", newline="")
writer = csv.writer(log)
writer.writerow([
    "time_ms",
    "det730",
    "det850",
    "pulse730",
    "pulse850",
    "contact",
    "bpm",
    "spo2",
    "hbo",
    "hbr",
    "hbt",
    "state",
    "mode"
])

# ===============================
# MBLL MATRIX
# ===============================
E = np.array([
    [0.80, 1.55],   # 730 nm: [HbO, HbR]
    [1.40, 0.75]    # 850 nm: [HbO, HbR]
], dtype=float)

E_INV = np.linalg.inv(E)

# ===============================
# PLOT
# ===============================
plt.style.use("dark_background")
plt.ion()

fig = plt.figure(figsize=(12, 8))
ax_wave = plt.subplot2grid((3, 1), (0, 0))
ax_fnirs = plt.subplot2grid((3, 1), (1, 0), rowspan=2)

pulse_line, = ax_wave.plot([], color="lime", lw=2)
ax_wave.set_xlim(0, WINDOW)
ax_wave.set_ylabel("Pulse")
ax_wave.grid(True, alpha=0.3)

hbo_line, = ax_fnirs.plot([], color="red", label="HbO")
hbr_line, = ax_fnirs.plot([], color="blue", label="HbR")
hbt_line, = ax_fnirs.plot([], color="white", label="HbT")
ax_fnirs.legend()
ax_fnirs.set_title("fNIRS Hemoglobin")
ax_fnirs.set_xlabel("Samples")
ax_fnirs.set_ylabel("Relative concentration change")
ax_fnirs.grid(True, alpha=0.3)

bpm_text = ax_wave.text(
    0.02, 0.85, "BPM --",
    transform=ax_wave.transAxes,
    fontsize=22,
    color="cyan"
)

spo2_text = ax_wave.text(
    0.25, 0.85, "SpO2 --",
    transform=ax_wave.transAxes,
    fontsize=22,
    color="yellow"
)

brain_text = ax_wave.text(
    0.52, 0.85, "STATE",
    transform=ax_wave.transAxes,
    fontsize=20,
    color="orange"
)

contact_text = ax_wave.text(
    0.80, 0.85, "MODE",
    transform=ax_wave.transAxes,
    fontsize=18,
    color="lime"
)

# ===============================
# FUNCTIONS
# ===============================
def moving_avg(x, w=7):
    x = np.array(x, dtype=float)
    if len(x) < w:
        return x
    return np.convolve(x, np.ones(w) / w, mode="same")

def estimate_fs():
    if len(t_ms) < 50:
        return 50.0
    dt = (t_ms[-1] - t_ms[-50]) / 1000.0
    if dt <= 0:
        return 50.0
    return 50.0 / dt

def bandpass(sig, fs):
    nyq = 0.5 * fs
    low = 0.8 / nyq
    high = 2.5 / nyq

    if low <= 0:
        low = 0.001
    if high >= 1.0:
        high = 0.99

    b, a = butter(3, [low, high], btype="band")
    return filtfilt(b, a, sig)

def estimate_bpm(signal, fs):
    peaks, _ = find_peaks(
        signal,
        distance=max(1, int(fs * 0.4)),
        height=0.3
    )

    if len(peaks) < 2:
        return None

    intervals = np.diff(peaks) / fs
    bpm = 60.0 / np.mean(intervals)

    if 40 < bpm < 180:
        return bpm
    return None

def compute_hbo_hbr_hbt(sig730, sig850):
    baseline_window = 150

    I0_730 = np.mean(sig730[-baseline_window:])
    I0_850 = np.mean(sig850[-baseline_window:])

    I0_730 = max(I0_730, 1e-6)
    I0_850 = max(I0_850, 1e-6)

    od730 = -np.log(sig730 / I0_730)
    od850 = -np.log(sig850 / I0_850)

    od730 = moving_avg(od730, 35)
    od850 = moving_avg(od850, 35)

    hbo = []
    hbr = []

    for a730, a850 in zip(od730, od850):
        A = np.array([a730, a850], dtype=float)
        c = E_INV @ A
        hbo.append(c[0])
        hbr.append(c[1])

    hbo = moving_avg(np.array(hbo), 35)
    hbr = moving_avg(np.array(hbr), 35)
    hbt = moving_avg(hbo + hbr, 35)

    return hbo, hbr, hbt

# ===============================
# MAIN LOOP
# ===============================
last_bpm = None
last_spo2 = None

print("Close Arduino Serial Monitor first.")
print(f"Logging to: {filename}")

try:
    while True:
        raw = ser.readline().decode(errors="ignore").strip()

        if not raw:
            plt.pause(0.02)
            continue

        if raw.startswith("ms"):
            plt.pause(0.02)
            continue

        parts = raw.split(",")

        # expected from fnirs_final_auto_gain.ino
        # ms,det1_730,det2_730,det1_850,det2_850,pulse730_det2,pulse850_det2
        if len(parts) != 7:
            plt.pause(0.02)
            continue

        try:
            ms = float(parts[0])
            d730 = float(parts[2])   # det2_730
            d850 = float(parts[4])   # det2_850
            p730 = float(parts[5])   # pulse730_det2
            p850 = float(parts[6])   # pulse850_det2
        except ValueError:
            plt.pause(0.02)
            continue

        t_ms.append(ms)
        det730.append(max(d730, 1e-6))
        det850.append(max(d850, 1e-6))
        pulse730.append(p730)
        pulse850.append(p850)

        fs = estimate_fs()
        s730 = np.array(det730, dtype=float)
        s850 = np.array(det850, dtype=float)

        hbo_last = None
        hbr_last = None
        hbt_last = None
        state = "REST"
        contact_flag = None

        # --------------------------------------------
        # FINGER MODE
        # --------------------------------------------
        if MODE == "finger":
            sig_raw = np.array(pulse730, dtype=float)
            signal_strength = np.std(sig_raw[-50:]) if len(sig_raw) > 50 else 0.0

            finger_present = signal_strength > 0.5
            contact_flag = finger_present

            if finger_present:
                contact_text.set_text("CONTACT")
                contact_text.set_color("lime")
            else:
                contact_text.set_text("NO CONTACT")
                contact_text.set_color("red")

            if finger_present and len(sig_raw) > 30 and np.std(sig_raw) > 1e-9:
                try:
                    sig = bandpass(sig_raw, fs)
                    sig = moving_avg(sig, 7)
                    sig = sig - moving_avg(sig, 40)

                    sig_std = np.std(sig)
                    if sig_std > 1e-9:
                        sig = sig / sig_std

                    bpm = estimate_bpm(sig, fs)
                    if bpm is not None:
                        last_bpm = bpm

                    pulse_line.set_data(range(len(sig)), sig)

                    y_min = np.min(sig)
                    y_max = np.max(sig)
                    if abs(y_max - y_min) < 0.2:
                        ax_wave.set_ylim(y_min - 1.0, y_max + 1.0)
                    else:
                        ax_wave.set_ylim(y_min * 1.5, y_max * 1.5)

                except Exception:
                    pass
            else:
                pulse_line.set_data(range(len(sig_raw)), np.zeros(len(sig_raw)))
                ax_wave.set_ylim(-1, 1)

            if finger_present and len(s730) >= 100:
                dc730 = np.mean(s730[-100:])
                dc850 = np.mean(s850[-100:])

                # Use pulse channels for AC estimate
                ac730 = np.std(np.array(pulse730, dtype=float)[-100:])
                ac850 = np.std(np.array(pulse850, dtype=float)[-100:])

                if dc730 > 0 and dc850 > 0 and ac850 > 0:
                    R = (ac730 / dc730) / (ac850 / dc850)
                    spo2 = 100.0 - 15.0 * (R - 1.0)
                    spo2 = max(70.0, min(100.0, spo2))
                    last_spo2 = spo2

            if len(s730) >= 150:
                hbo, hbr, hbt = compute_hbo_hbr_hbt(s730, s850)
                hbo_last = float(hbo[-1])
                hbr_last = float(hbr[-1])
                hbt_last = float(hbt[-1])

                hbo_line.set_data(range(len(hbo)), hbo)
                hbr_line.set_data(range(len(hbr)), hbr)
                hbt_line.set_data(range(len(hbt)), hbt)

                ax_fnirs.relim()
                ax_fnirs.autoscale_view()

            if finger_present and last_bpm is not None:
                bpm_text.set_text(f"BPM {last_bpm:.0f}")
            else:
                bpm_text.set_text("BPM --")

            if finger_present and last_spo2 is not None:
                spo2_text.set_text(f"SpO2 {last_spo2:.1f}%")
            else:
                spo2_text.set_text("SpO2 --")

            brain_text.set_text("STATE: FINGER MODE")
            brain_text.set_color("orange")

        # --------------------------------------------
        # FOREHEAD MODE
        # --------------------------------------------
        elif MODE == "forehead":
            contact_text.set_text("FOREHEAD MODE")
            contact_text.set_color("cyan")
            contact_flag = None

            sig_raw = np.array(pulse730, dtype=float)

            if len(sig_raw) > 30 and np.std(sig_raw) > 1e-9:
                try:
                    sig = bandpass(sig_raw, fs)
                    sig = moving_avg(sig, 7)
                    sig = sig - moving_avg(sig, 40)

                    sig_std = np.std(sig)
                    if sig_std > 1e-9:
                        sig = sig / sig_std

                    pulse_line.set_data(range(len(sig)), sig)

                    y_min = np.min(sig)
                    y_max = np.max(sig)
                    if abs(y_max - y_min) < 0.2:
                        ax_wave.set_ylim(y_min - 1.0, y_max + 1.0)
                    else:
                        ax_wave.set_ylim(y_min * 1.5, y_max * 1.5)

                except Exception:
                    pass
            else:
                pulse_line.set_data(range(len(sig_raw)), np.zeros(len(sig_raw)))
                ax_wave.set_ylim(-1, 1)

            bpm_text.set_text("BPM --")
            spo2_text.set_text("SpO2 --")

            if len(s730) >= 150:
                hbo, hbr, hbt = compute_hbo_hbr_hbt(s730, s850)

                hbo_last = float(hbo[-1])
                hbr_last = float(hbr[-1])
                hbt_last = float(hbt[-1])

                hbo_line.set_data(range(len(hbo)), hbo)
                hbr_line.set_data(range(len(hbr)), hbr)
                hbt_line.set_data(range(len(hbt)), hbt)

                ax_fnirs.relim()
                ax_fnirs.autoscale_view()

                if len(hbo) >= 100 and len(hbr) >= 100:
                    baseline_hbo = np.mean(hbo[:100])
                    baseline_hbr = np.mean(hbr[:100])

                    delta_hbo = np.mean(hbo[-40:]) - baseline_hbo
                    delta_hbr = np.mean(hbr[-40:]) - baseline_hbr

                    state = "REST"
                    if delta_hbo > 0.015 and delta_hbr < -0.005:
                        state = "FOCUS"
                    if delta_hbo > 0.035 and delta_hbr < -0.015:
                        state = "HIGH WORKLOAD"

            brain_text.set_text(f"STATE: {state}")
            if state == "REST":
                brain_text.set_color("orange")
            elif state == "FOCUS":
                brain_text.set_color("lime")
            elif state == "HIGH WORKLOAD":
                brain_text.set_color("red")

        # --------------------------------------------
        # LOG
        # --------------------------------------------
        writer.writerow([
            ms,
            d730,
            d850,
            p730,
            p850,
            contact_flag,
            last_bpm if MODE == "finger" and contact_flag else None,
            last_spo2 if MODE == "finger" and contact_flag else None,
            hbo_last,
            hbr_last,
            hbt_last,
            state,
            MODE
        ])

        fig.canvas.draw_idle()
        plt.pause(0.03)

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    try:
        log.close()
    except Exception:
        pass

    try:
        ser.close()
    except Exception:
        pass

    plt.ioff()
    plt.show()