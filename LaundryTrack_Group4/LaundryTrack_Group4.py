import smbus
import time
import math
import sqlite3
import json
from datetime import datetime
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

# Hardware config

DEVICE_ADDRESS = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
bus = smbus.SMBus(1)

# LED config

LED_PIN = 17

# MQTT config (HiveMQ public broker)

BROKER = "broker.hivemq.com"
PORT = 1883
TOPIC_STATUS = "laundry/machine1/status"
TOPIC_EVENT = "laundry/machine1/event"
TOPIC_SNAPSHOT = "laundry/machine1/snapshot"

mqtt_client = mqtt.Client()


# Database config

DB_FILE = "laundry_monitor.db"

# Detection parameters need to be tuned based on actual machine vibration patterns

THRESHOLD = 0.05
START_CONFIRM = 10
STOP_CONFIRM = 120
SAMPLE_INTERVAL = 0.5
SNAPSHOT_INTERVAL = 1800


# Global state

current_state = "IDLE"
candidate_state = None
candidate_since = None
run_start_time = None


# Setup 

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_PIN, GPIO.OUT)
    set_led("IDLE")


def setup_mqtt():
    mqtt_client.connect(BROKER, PORT, 60)
    mqtt_client.loop_start()


def setup_db():
    conn = sqlite3.connect(DB_FILE)
    cursor = conn.cursor()

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS machine_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            event_type TEXT,
            event_time TEXT
        )
    """)

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS snapshots (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT,
            state TEXT
        )
    """)

    conn.commit()
    return conn


# Utility

def now():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def publish(topic, payload):
    mqtt_client.publish(topic, json.dumps(payload))


def set_led(state):
    GPIO.output(LED_PIN, state == "IDLE")


def save_event(conn, event, t):
    conn.execute(
        "INSERT INTO machine_events (event_type, event_time) VALUES (?, ?)",
        (event, t)
    )
    conn.commit()


def save_snapshot(conn, t, state):
    conn.execute(
        "INSERT INTO snapshots (timestamp, state) VALUES (?, ?)",
        (t, state)
    )
    conn.commit()


# MPU9250 accelerometer

def mpu_init():
    bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0)
    time.sleep(0.1)


def read_word(reg):
    high = bus.read_byte_data(DEVICE_ADDRESS, reg)
    low = bus.read_byte_data(DEVICE_ADDRESS, reg + 1)
    val = (high << 8) | low
    return val - 65536 if val >= 0x8000 else val


def get_vibration():
    ax = read_word(ACCEL_XOUT_H) / 16384.0
    ay = read_word(ACCEL_XOUT_H + 2) / 16384.0
    az = read_word(ACCEL_XOUT_H + 4) / 16384.0

    total = math.sqrt(ax**2 + ay**2 + az**2)
    return abs(total - 1.0)


# Main program

if __name__ == "__main__":
    conn = setup_db()
    setup_gpio()
    setup_mqtt()
    mpu_init()

    last_snapshot = time.time()

    try:
        while True:
            vib = get_vibration()
            raw_state = "RUNNING" if vib > THRESHOLD else "IDLE"

            t_now = time.time()
            t_str = now()

            # State transition with debounce
            if raw_state != current_state:
                if candidate_state != raw_state:
                    candidate_state = raw_state
                    candidate_since = t_now
                else:
                    elapsed = t_now - candidate_since

                    if current_state == "IDLE" and raw_state == "RUNNING" and elapsed >= START_CONFIRM:
                        current_state = "RUNNING"
                        run_start_time = t_str

                        print(f"[START] {t_str}")
                        save_event(conn, "START", t_str)
                        publish(TOPIC_EVENT, {"event": "START", "time": t_str})
                        publish(TOPIC_STATUS, {"state": current_state, "time": t_str})
                        set_led(current_state)

                        candidate_state = None

                    elif current_state == "RUNNING" and raw_state == "IDLE" and elapsed >= STOP_CONFIRM:
                        current_state = "IDLE"

                        print(f"[STOP] {t_str}")
                        save_event(conn, "STOP", t_str)
                        publish(TOPIC_EVENT, {"event": "STOP", "time": t_str})
                        publish(TOPIC_STATUS, {"state": current_state, "time": t_str})
                        set_led(current_state)

                        candidate_state = None
            else:
                candidate_state = None

            # Periodic snapshot
            if t_now - last_snapshot >= SNAPSHOT_INTERVAL:
                save_snapshot(conn, t_str, current_state)
                publish(TOPIC_SNAPSHOT, {"state": current_state, "time": t_str})
                print(f"[SNAPSHOT] {t_str} {current_state}")
                last_snapshot = t_now

            print(f"vib={vib:.4f} state={current_state}")
            time.sleep(SAMPLE_INTERVAL)

    except KeyboardInterrupt:
        pass

    finally:
        GPIO.cleanup()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        conn.close()
        bus.close()