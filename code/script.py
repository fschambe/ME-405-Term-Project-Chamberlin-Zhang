import os
import time
import serial
import matplotlib.pyplot as plt

# =========================
# USER SETTINGS
# =========================
PORT = "COM8"
BAUD = 115200
SER_TIMEOUT_S = 0.2

LOG_DIR = os.path.join(os.path.expanduser("~"), "Downloads", "Collection Log")

# One or more tests: (setpoint_mm_s, Kp, Ki)
TESTS = [
    (50.0, 0.2, 0.4),
]

# How long to wait for BEGIN/END markers from firmware
WAIT_BEGIN_S = 20.0
WAIT_END_S = 20.0

# =========================
# SERIAL HELPERS
# =========================
def open_serial(port: str, baud: int, timeout_s: float):
    print(f"Opening serial: {port} @ {baud} ...")
    ser = serial.Serial(port, baud, timeout=timeout_s)
    # Give board time to reboot/settle after opening port
    time.sleep(1.5)
    ser.reset_input_buffer()
    return ser

def read_line(ser) -> str:
    """Read one ASCII line (stripped). Returns '' on timeout."""
    try:
        raw = ser.readline()
        if not raw:
            return ""
        return raw.decode("utf-8", errors="ignore").strip()
    except Exception:
        return ""

def write_char(ser, ch: str):
    """Send a single-letter command with NO newline."""
    ser.write(ch.encode("utf-8"))

def write_number(ser, value):
    """Send a numeric value terminated by newline (ends digit-entry mode)."""
    ser.write((str(value) + "\n").encode("utf-8"))

def sync_prompt(ser, seconds=2.0):
    """
    Optional: ping the UI so it prints something.
    Your UI treats newline as 'reprint prompt', so this is safe.
    """
    end = time.time() + seconds
    ser.write(b"\n")
    while time.time() < end:
        line = read_line(ser)
        if line:
            # Uncomment to see boot/menu text
            # print("NUCLEO:", line)
            pass

# =========================
# PROTOCOL (matches your firmware)
# =========================
def send_test_commands(ser, setpoint, kp, ki):
    """
    IMPORTANT:
    - send 's','p','i','g' as single characters (NO newline)
    - send numeric values with newline
    """
    # Set setpoint
    write_char(ser, "s")
    write_number(ser, setpoint)

    # Set Kp
    write_char(ser, "p")
    write_number(ser, kp)

    # Set Ki
    write_char(ser, "i")
    write_number(ser, ki)

    # Start test
    write_char(ser, "g")

def collect_dataset(ser):
    """
    Waits for:
      BEGIN
      <header line like: Time, Velocity (mm/s)>
      <data lines like: 0.05, 12.3>
      END
    Returns: headers(list[str]), rows(list[list[float]])
    """
    # Wait for BEGIN
    t0 = time.time()
    while True:
        if time.time() - t0 > WAIT_BEGIN_S:
            raise RuntimeError("Timed out waiting for BEGIN from Nucleo. Check firmware output/protocol.")
        line = read_line(ser)
        if not line:
            continue

        # Uncomment for debug:
        # print("NUCLEO:", line)

        if line.startswith("BEGIN"):
            break

    # Read header line
    header = ""
    while header == "":
        header = read_line(ser)

    headers = [h.strip() for h in header.split(",")]

    # Read data until END
    rows = []
    t1 = time.time()
    while True:
        if time.time() - t1 > WAIT_END_S:
            raise RuntimeError("Timed out waiting for END from Nucleo. Check firmware output/protocol.")

        line = read_line(ser)
        if not line:
            continue

        if line.startswith("END"):
            break

        # Parse CSV row: "t, vel"
        parts = [p.strip() for p in line.split(",")]
        if len(parts) != 2:
            # Ignore unexpected lines
            continue

        try:
            rows.append([float(parts[0]), float(parts[1])])
        except ValueError:
            # Ignore non-numeric lines
            continue

    return headers, rows

# =========================
# SAVE + PLOT
# =========================
def save_csv(headers, rows, csv_path):
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write(",".join(headers) + "\n")
        for r in rows:
            f.write(f"{r[0]},{r[1]}\n")

def plot_step_response(setpoint, kp, ki, headers, rows, png_path):
    t_raw = [r[0] for r in rows]
    v_raw = [r[1] for r in rows]

    # Break the line when time resets (second dataset starts)
    t = []
    v = []
    last_t = -1e9
    for ti, vi in zip(t_raw, v_raw):
        if ti < last_t:      # time went backwards -> new segment
            t.append(float("nan"))
            v.append(float("nan"))
        t.append(ti)
        v.append(vi)
        last_t = ti

    plt.figure(figsize=(8, 6))
    plt.plot(t, v, label="Measured velocity")
    plt.axhline(setpoint, linestyle="--", label="Setpoint")
    plt.title(f"Step Response (sp={setpoint} mm/s, Kp={kp}, Ki={ki})")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed (mm/s)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    os.makedirs(os.path.dirname(png_path), exist_ok=True)
    plt.savefig(png_path, dpi=150)
    plt.close()

def run_one_test(ser, setpoint, kp, ki):
    tag = time.strftime("%m_%d_%H_%M_%S")
    csv_path = os.path.join(LOG_DIR, f"{tag}_sp{setpoint}_kp{kp}_ki{ki}.csv")
    png_path = os.path.join(LOG_DIR, f"{tag}_sp{setpoint}_kp{kp}_ki{ki}.png")

    print(f"\n--- Running test: setpoint={setpoint}, Kp={kp}, Ki={ki} ---")

    # Send commands
    send_test_commands(ser, setpoint, kp, ki)

    # Collect dataset from BEGIN..END
    headers, rows = collect_dataset(ser)

    print("Headers:", headers)
    print("First row:", rows[0] if rows else "NO DATA")
    print("Rows collected:", len(rows))

    # Save and plot
    save_csv(headers, rows, csv_path)
    plot_step_response(setpoint, kp, ki, headers, rows, png_path)

    print("Saved CSV:", os.path.abspath(csv_path))
    print("Saved PNG:", os.path.abspath(png_path))

def main():
    with open_serial(PORT, BAUD, SER_TIMEOUT_S) as ser:
        sync_prompt(ser, seconds=1.0)
        for (sp, kp, ki) in TESTS:
            run_one_test(ser, sp, kp, ki)

    print("\nAll tests complete.")

if __name__ == "__main__":
    main()
