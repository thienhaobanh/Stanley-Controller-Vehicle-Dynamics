import math
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. KINEMATIC BICYCLE MODEL
# ==========================================
class Vehicle:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        # Initialize vehicle state
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        
        # Vehicle physical parameters
        self.L = 2.9  # Wheelbase (meters)
        self.max_steer = np.radians(35.0)  # Maximum steering angle limit

    def update(self, v, delta, dt):
        # Update longitudinal velocity
        self.v = v
        
        # Limit steering angle to simulate mechanical constraints
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        # State update equations (Kinematic Bicycle Model)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / self.L * math.tan(delta) * dt
        
        # Normalize yaw angle
        self.yaw = self.normalize_angle(self.yaw)

    def normalize_angle(self, angle):
        # Ensure angle remains within [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

# ==========================================
# 2. STANLEY CONTROLLER ALGORITHM
# ==========================================
def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def stanley_control(x, y, yaw, v, cx, cy, cyaw, k=0.5):
    # Find the index of the closest point on the reference path
    dx = [x - icx for icx in cx]
    dy = [y - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)
    
    # Calculate cross-track error
    front_axle_vec = [-np.cos(yaw + np.pi/2), -np.sin(yaw + np.pi/2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
    
    # Calculate heading error
    theta_e = normalize_angle(cyaw[target_idx] - yaw)
    
    # Core Stanley Controller formula
    theta_d = np.arctan2(k * error_front_axle, v)
    delta = theta_e + theta_d
    
    return delta, target_idx, error_front_axle

# ==========================================
# 3. SIMULATION SCENARIO & DATA COLLECTION
# ==========================================
def main():
    # Initialize target trajectory (S-Curve)
    cx = np.arange(0, 50, 0.5)
    cy = [np.sin(ix / 5.0) * 5.0 for ix in cx]
    cyaw = [np.arctan2(np.cos(ix / 5.0), 1.0) for ix in cx]

    # Setup simulation parameters
    target_speed = 3.0  # Vehicle speed (m/s)
    k_gain = 5.0         # Stanley controller gain
    dt = 0.1             # Time step (seconds)
    time_max = 20.0      # Maximum simulation time

    # Initialize vehicle at an offset position to test error correction
    car = Vehicle(x=0.0, y=0.0, yaw=cyaw[0], v=target_speed)

    # Arrays for telemetry data storage
    x_history, y_history, yaw_history, e_history, time_history, delta_history = [], [], [], [], [], []

    time = 0.0
    target_idx = 0

    print("Running simulation...")
    while time < time_max and target_idx < len(cx) - 1:
        # 1. Error Computation & Control Command
        di, target_idx, error = stanley_control(car.x, car.y, car.yaw, car.v, cx, cy, cyaw, k=k_gain)
        
        # 2. Kinematic Update
        car.update(v=target_speed, delta=di, dt=dt) 
        
        x_history.append(car.x)
        y_history.append(car.y)
        yaw_history.append(car.yaw)

        # 3. Telemetry Logging (snippet)
        e_history.append(error)
        delta_history.append(np.degrees(di))
        time_history.append(time)
        
        time += dt

    # ==========================================
    # 4. QUANTITATIVE RESULTS & PLOTTING
    # ==========================================
    max_error = np.max(np.abs(e_history))
    mean_error = np.mean(np.abs(e_history))
    variance_error = np.var(e_history)

    print("\n--- STATISTICAL RESULTS ---")
    print(f"Simulation Speed: {target_speed} m/s | Gain k: {k_gain}")
    print(f"Max Cross-track Error: {max_error:.4f} m")
    print(f"Mean Error: {mean_error:.4f} m")
    print(f"Error Variance: {variance_error:.4f} m^2")

    # Plot 1: 2D Trajectory
    plt.figure(figsize=(12, 10))
    
    plt.subplot(3, 1, 1)
    plt.plot(cx, cy, "-r", label="Reference Path")
    plt.plot(x_history, y_history, "-b", label="Vehicle Trajectory")
    plt.legend()
    plt.title("Stanley Controller Trajectory Tracking")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.grid(True)
    plt.axis("equal")

    # Plot 2: Error Analysis (Cross-track error)
    plt.subplot(3, 1, 2)
    plt.plot(time_history, e_history, "-g", label="Cross-track Error")
    plt.legend()
    plt.title("Tracking Error over Time")
    plt.xlabel("Time [s]")
    plt.ylabel("Error [m]")
    plt.grid(True)

    # Plot 3: Steering Angle Analysis
    plt.subplot(3, 1, 3)
    plt.plot(time_history, delta_history, "-m", label="Steering Angle")
    plt.legend()
    plt.title("Steering Angle over Time")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()