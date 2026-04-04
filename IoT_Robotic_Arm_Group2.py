#!/usr/bin/env python3
# Logic: Button click triggers a pick-and-place sequence + MQTT status updates

import warnings
warnings.filterwarnings("ignore")

from gpiozero import AngularServo, Button
from time import sleep
import paho.mqtt.client as mqtt
import sys



# --- Basic Config ---
BUTTON_PIN = 24
MOVE_DELAY = 0.02 # Seconds between each degree (smaller is faster)



# MQTT Broker settings (using HiveMQ public broker)
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_TOPIC = "mcmaster/cps/group2"



# Joint setup: [GPIO Pin, Name, Home Angle, Is Reversed?]
JOINT_CONFIG = {
    '0': [12, "Base (J0)", 90,  False],
    '1': [13, "Main Arm (J1)", 120, True],
    '2': [18, "Forearm (J2)", 180, False],
    '3': [19, "Wrist (J3)", 70,  False],
    '4': [21, "Pitch (J4)", 120, False],
    '5': [26, "Gripper (J5)", 50,  False]
}



# Standard pulse widths for servos
MIN_PW = 0.5 / 1000
MAX_PW = 2.5 / 1000



servos = {}
current_angles = {}
is_running = False # Flag to prevent overlapping movements



# --- MQTT Functions ---

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] Successfully connected to broker")
    else:
        print(f"[MQTT] Connection failed. Error code: {rc}")

# Setup the client
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
mqtt_client.on_connect = on_connect

def mqtt_publish(message):
    # Simple helper to send messages and log them locally
    mqtt_client.publish(MQTT_TOPIC, message)
    print(f"[MQTT] Sent: {message}")



# --- Mechanical Control ---

def set_servo_angle(idx, logical_angle):
    # Sets a specific joint to an angle, handling reversed servos automatically
    is_reverse = JOINT_CONFIG[idx][3]
    actual_angle = (180 - logical_angle) if is_reverse else logical_angle
    servos[idx].angle = actual_angle
    current_angles[idx] = logical_angle

def smooth_move(idx, target_angle):
    # Moves a joint degree-by-degree to avoid jerky movements
    start = int(current_angles[idx])
    target = int(max(0, min(180, target_angle)))
    
    if start == target:
        return
        
    step = 1 if target > start else -1
    for angle in range(start, target + step, step):
        set_servo_angle(idx, angle)
        sleep(MOVE_DELAY)

def cleanup_and_exit():
    # Safety first: release servos and disconnect MQTT before quitting
    for s in servos.values():
        s.value = None # Cut power to the motor
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    print("\n[System] Servos released. MQTT disconnected. Goodbye!")
    sys.exit(0)



# --- The Main Routine ---

def run_action():
    global is_running
    if is_running:
        print("[Ignored] Already busy with a task!")
        mqtt_publish("Busy: Request ignored as arm is currently active.")
        return

    is_running = True

    print("\n[Action] Starting the pick-and-place sequence...")
    mqtt_publish("Alert: Sequence started by button press.")

    # 1. Lower the arm
    print("  Step 1: Moving to pick position...")
    smooth_move('0', 125)
    smooth_move('1', 125)

    # 2. Close the gripper
    print("  Step 2: Grabbing...")
    smooth_move('5', 100)
    sleep(0.5)
    mqtt_publish("Status: Object grabbed.")

    # 3. Lift and move
    print("  Step 3: Transporting...")
    smooth_move('1', 90)
    smooth_move('3', 45)
    smooth_move('3', 100)
    smooth_move('3', 70)
    smooth_move('0', 75)
    smooth_move('1', 120)

    # 4. Release
    print("  Step 4: Placing...")
    smooth_move('5', 50)
    sleep(0.5)
    mqtt_publish("Success: Object placed successfully.")

    # Return to neutral
    smooth_move('1', 90)
    smooth_move('0', 90)

    print("[Done] Sequence finished. Standing by.")
    is_running = False



# --- Entry Point ---

if __name__ == '__main__':
    print("----------------------------------------")
    print(" Robotic Arm")
    print("----------------------------------------")

    # Connect to the cloud
    print("[MQTT] Connecting...")
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()
    sleep(2) # Give it a moment to stabilize

    # Wake up the servos
    print("Initializing joints...")
    for idx, (pin, name, init_angle, is_reverse) in JOINT_CONFIG.items():
        servos[idx] = AngularServo(pin, min_angle=0, max_angle=180,
                                  min_pulse_width=MIN_PW, max_pulse_width=MAX_PW)
        set_servo_angle(idx, init_angle)
        print(f"  {name} initialized at {init_angle}°")
        sleep(0.3)

    # Listen for button press on GPIO 24
    button = Button(BUTTON_PIN, pull_up=True, bounce_time=0.05)
    button.when_pressed = run_action

    print("\n[Ready] Press the button to start. Type 'q' to exit.")

    try:
        while True:
            cmd = input().strip().lower()
            if cmd == 'q':
                cleanup_and_exit()
    except KeyboardInterrupt:
        cleanup_and_exit()