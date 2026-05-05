import serial
import json
import time

# Function to load waypoints from a JSON file

def load_waypoints(file_path):
    with open(file_path, 'r') as f:
        waypoints = json.load(f)
    return waypoints

# Function to send waypoints to Arduino

def send_waypoints(port, waypoints):
    try:
        with serial.Serial(port, 9600, timeout=1) as ser:
            time.sleep(2)  # Wait for Arduino to reset
            for waypoint in waypoints:
                ser.write(f"{waypoint['lat']},{waypoint['lon']}\n".encode())
                print(f"Sent waypoint: {waypoint}")
                response = ser.readline().decode().strip()
                print(f"Response from Arduino: {response}")
    except serial.SerialException as e:
        print(f"Serial exception: {e}")

# Main execution logic
if __name__ == '__main__':
    waypoints_file = 'waypoints.json'
    waypoints = load_waypoints(waypoints_file)
    arduino_port = 'COM3'  # Change this to your Arduino's port
    send_waypoints(arduino_port, waypoints)
