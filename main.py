#!/usr/bin/env python3
import cv2
import numpy as np
import mediapipe as mp
import time
import threading
import math
from math import copysign
from gpiozero import Robot
from multiprocessing import Process, Value
import ctypes

# ==========================================
# 1. Hardware Configuration
# ==========================================
my_robot = Robot(left=(17, 27), right=(22, 23))

# ==========================================
# 2. Tuning Constants
# ==========================================
MAX_SPEED = 0.50
MIN_SPEED = 0.20
MAX_TURN_SPEED = 0.35
MOTOR_DEADBAND = 0.16
MAX_SPEED_CHANGE = 0.05

K_TURN = 0.008             # Proportional gain for turning (degrees to speed)
K_DRIVE = 0.60             # Forward/backward proportional gain

PROC_W = 320
PROC_H = 240
PROC_CENTER_X = PROC_W // 2
DISP_W = 640
DISP_H = 480

TARGET_DIST = 1.00         # Maintain 1.0 meter from person
DIST_DEADBAND = 0.20       # +/- 20cm deadband (0.80m to 1.20m stops)

COLOR_MATCH_THRESHOLD = 0.55
COLOR_TRACKING_THRESHOLD = 0.45
COLOR_RECALIB_RATE = 0.06
COLOR_RECALIB_MIN_SIM = 0.75

TARGET_LOST_GRACE = 15     
TARGET_LOST_DECAY = 0.85

# ==========================================
# 3. Initialization
# ==========================================
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    model_complexity=0,
    min_detection_confidence=0.3,
    min_tracking_confidence=0.3
)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, DISP_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DISP_H)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

lidar_out_dist = Value(ctypes.c_double, 0.0)
lidar_out_angle = Value(ctypes.c_double, 0.0)
cam_in_angle = Value(ctypes.c_double, 999.0)

obs_front = Value(ctypes.c_double, 99.0)
obs_left = Value(ctypes.c_double, 99.0)
obs_right = Value(ctypes.c_double, 99.0)
obs_back = Value(ctypes.c_double, 99.0)

# ==========================================
# 4. Helper Functions
# ==========================================
def clamp(value, min_val=-1.0, max_val=1.0):
    return max(min_val, min(max_val, value))

def ramp_toward(current, target, max_change):
    diff = target - current
    if abs(diff) <= max_change:
        return target
    return current + max_change * (1.0 if diff > 0 else -1.0)

def apply_deadband(value, deadband):
    if abs(value) < deadband:
        return 0.0
    return value

current_left_speed = 0.0
current_right_speed = 0.0

def set_motors(left_power, right_power):
    global current_left_speed, current_right_speed
    
    # Ramp the logical speeds
    current_left_speed = ramp_toward(current_left_speed, left_power, MAX_SPEED_CHANGE)
    current_right_speed = ramp_toward(current_right_speed, right_power, MAX_SPEED_CHANGE)

    def logical_to_physical(speed):
        if abs(speed) < 0.05:
            return 0.0
        # Smoothly map logical 0.0->1.0 to physical MOTOR_DEADBAND->1.0
        phys = MOTOR_DEADBAND + (abs(speed) * (1.0 - MOTOR_DEADBAND))
        return copysign(phys, speed)

    my_robot.left_motor.value = clamp(logical_to_physical(current_left_speed))
    my_robot.right_motor.value = clamp(logical_to_physical(current_right_speed))

def stop_motors():
    set_motors(0.0, 0.0)

def get_body_center(landmarks):
    tracking_ids = [
        mp_pose.PoseLandmark.LEFT_SHOULDER,
        mp_pose.PoseLandmark.RIGHT_SHOULDER,
        mp_pose.PoseLandmark.LEFT_HIP,
        mp_pose.PoseLandmark.RIGHT_HIP,
    ]
    xs, weights = [], []
    for lid in tracking_ids:
        lm = landmarks[lid]
        vis = getattr(lm, 'visibility', 0.0)
        if vis > 0.4:
            xs.append(lm.x)
            weights.append(vis)
    if len(xs) < 2: return None, 0.0
    weights = np.array(weights)
    xs = np.array(xs)
    center_x = (xs * weights).sum() / weights.sum()
    conf = weights.sum() / len(weights)
    return center_x, conf

def get_torso_histogram(frame, landmarks, w_img, h_img):
    torso_ids = [
        mp_pose.PoseLandmark.LEFT_SHOULDER,
        mp_pose.PoseLandmark.RIGHT_SHOULDER,
        mp_pose.PoseLandmark.LEFT_HIP,
        mp_pose.PoseLandmark.RIGHT_HIP,
    ]
    visible = [landmarks[lid] for lid in torso_ids if getattr(landmarks[lid], 'visibility', 0.0) > 0.3]
    if len(visible) < 3: return None, None

    x_coords = [lm.x for lm in visible]
    y_coords = [lm.y for lm in visible]

    x_min, x_max = int(min(x_coords) * w_img), int(max(x_coords) * w_img)
    y_min, y_max = int(min(y_coords) * h_img), int(max(y_coords) * h_img)

    pad_x, pad_y = int((x_max - x_min) * 0.18), int((y_max - y_min) * 0.18)
    x_min, x_max = max(0, x_min + pad_x), min(w_img, x_max - pad_x)
    y_min, y_max = max(0, y_min + pad_y), min(h_img, y_max - pad_y)

    if (x_max - x_min) > 10 and (y_max - y_min) > 10:
        torso_crop = frame[y_min:y_max, x_min:x_max]
        hsv = cv2.cvtColor(torso_crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [50, 60], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist, (x_min, y_min, x_max, y_max)
    return None, None

# ==========================================
# 5. LiDAR Kalman Filter Process
# ==========================================
class KalmanFilter2D:
    def __init__(self, dt=0.1):
        self.dt = dt
        # State: [x, y, vx, vy]
        self.x = np.zeros((4, 1))
        # Transition matrix
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        # Process noise covariance
        self.Q = np.eye(4) * 0.05
        # Measurement noise covariance
        self.R = np.eye(2) * 0.1
        # Estimate error covariance
        self.P = np.eye(4) * 1000
        
        self.is_initialized = False

    def init_state(self, x, y):
        self.x = np.array([[x], [y], [0], [0]])
        self.P = np.eye(4) * 10
        self.is_initialized = True

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x[:2].flatten()

    def update(self, z):
        z = np.array(z).reshape((2, 1))
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.H.shape[1])
        self.P = np.dot((I - np.dot(K, self.H)), self.P)
        return self.x[:2].flatten()

def polar_to_cartesian(r, a_deg):
    a_rad = math.radians(a_deg)
    return r * math.cos(a_rad), r * math.sin(a_rad)

def cartesian_to_polar(x, y):
    r = math.sqrt(x**2 + y**2)
    a_deg = math.degrees(math.atan2(y, x))
    return r, a_deg

def lidar_process_func(out_dist, out_angle, in_cam_angle, out_obs_f, out_obs_l, out_obs_r, out_obs_b):
    import ydlidar
    ydlidar.os_init()
    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/ttyUSB0")
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 5)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

    if not laser.initialize() or not laser.turnOn():
        print("Warning: LiDAR failed to initialize!")
        return

    scan = ydlidar.LaserScan()
    kf = KalmanFilter2D(dt=0.1) # 10Hz
    frames_lost = 0

    try:
        while True:
            r = laser.doProcessSimple(scan)
            if r:
                valid_points = []
                for p in scan.points:
                    if 0.35 < p.range < 3.0: # Ignore chassis (<35cm) and walls (>3m)
                        valid_points.append((p.range, -p.angle * 180.0 / math.pi))
                
                valid_points.sort(key=lambda x: x[1])

                # Simple DBSCAN-like 1D clustering
                clusters = []
                if valid_points:
                    current_cluster = [valid_points[0]]
                    for i in range(1, len(valid_points)):
                        r1, a1 = valid_points[i-1]
                        r2, a2 = valid_points[i]
                        x1, y1 = polar_to_cartesian(r1, a1)
                        x2, y2 = polar_to_cartesian(r2, a2)
                        
                        dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)
                        if dist < 0.25: # 25cm grouping
                            current_cluster.append(valid_points[i])
                        else:
                            clusters.append(current_cluster)
                            current_cluster = [valid_points[i]]
                            
                    # Check wraparound
                    if len(clusters) > 0:
                        x1, y1 = polar_to_cartesian(current_cluster[-1][0], current_cluster[-1][1])
                        x2, y2 = polar_to_cartesian(clusters[0][0][0], clusters[0][0][1])
                        if math.sqrt((x1-x2)**2 + (y1-y2)**2) < 0.25:
                            clusters[0] = current_cluster + clusters[0]
                        else:
                            clusters.append(current_cluster)
                    else:
                        clusters.append(current_cluster)

                cluster_centers_cart = []
                for c in clusters:
                    if len(c) < 2: continue # Needs at least 2 points
                    
                    x_start, y_start = polar_to_cartesian(c[0][0], c[0][1])
                    x_end, y_end = polar_to_cartesian(c[-1][0], c[-1][1])
                    width = math.sqrt((x_start-x_end)**2 + (y_start-y_end)**2)
                    
                    if width > 0.6: continue # Ignore walls/large furniture
                    
                    # Compute Cartesian center
                    avg_x = sum(polar_to_cartesian(p[0], p[1])[0] for p in c) / len(c)
                    avg_y = sum(polar_to_cartesian(p[0], p[1])[1] for p in c) / len(c)
                    cluster_centers_cart.append((avg_x, avg_y))

                cam_angle = in_cam_angle.value
                best_cluster = None
                
                # If Kalman Filter is initialized, predict next position
                pred_x, pred_y = (0.0, 0.0)
                if kf.is_initialized:
                    pred_x, pred_y = kf.predict()

                if cam_angle != 999.0:
                    # MODE 1: Camera sees person -> Lock/update CLOSEST cluster matching camera angle
                    valid_candidates = []
                    for cx, cy in cluster_centers_cart:
                        cr, ca = cartesian_to_polar(cx, cy)
                        if -60 < ca < 60:
                            if abs(ca - cam_angle) < 20.0:
                                valid_candidates.append((cr, cx, cy))
                    
                    if valid_candidates:
                        valid_candidates.sort(key=lambda x: x[0]) # Closest first
                        best_cluster = (valid_candidates[0][1], valid_candidates[0][2])
                    
                    if best_cluster is not None:
                        if not kf.is_initialized or frames_lost > 15:
                            kf.init_state(best_cluster[0], best_cluster[1])
                        else:
                            kf.update(best_cluster)
                        frames_lost = 0
                    elif kf.is_initialized:
                        frames_lost += 1
                else:
                    # MODE 2: Camera missed/cropped -> Use KF prediction or Auto-Initialize on front legs
                    if kf.is_initialized:
                        min_dist = 999.0
                        for cx, cy in cluster_centers_cart:
                            dist = math.sqrt((cx - pred_x)**2 + (cy - pred_y)**2)
                            if dist < 0.7: # Search radius around predicted position
                                if dist < min_dist:
                                    min_dist = dist
                                    best_cluster = (cx, cy)
                        
                        if best_cluster is not None:
                            kf.update(best_cluster)
                            frames_lost = 0
                        else:
                            frames_lost += 1
                    else:
                        # AUTO-INITIALIZE: Pick closest cluster in front of robot (-45 to +45 deg, 0.3m to 2.2m)
                        min_r = 999.0
                        for cx, cy in cluster_centers_cart:
                            cr, ca = cartesian_to_polar(cx, cy)
                            if -45 < ca < 45 and 0.3 < cr < 2.2:
                                if cr < min_r:
                                    min_r = cr
                                    best_cluster = (cx, cy)
                        
                        if best_cluster is not None:
                            kf.init_state(best_cluster[0], best_cluster[1])
                            frames_lost = 0
                
                # Output target state to main process
                if kf.is_initialized and frames_lost < 15:
                    smooth_r, smooth_a = cartesian_to_polar(kf.x[0,0], kf.x[1,0])
                    out_dist.value = smooth_r
                    out_angle.value = smooth_a
                else:
                    out_dist.value = 0.0
                    out_angle.value = 0.0
                    kf.is_initialized = False

                # ------------------------------------------
                # Obstacle Avoidance Sectors Calculation
                # ------------------------------------------
                target_x, target_y = (99.0, 99.0)
                if kf.is_initialized:
                    target_x, target_y = kf.x[0,0], kf.x[1,0]

                obs_f_min, obs_l_min, obs_r_min, obs_b_min = 99.0, 99.0, 99.0, 99.0
                for r_pt, a_pt in valid_points:
                    px, py = polar_to_cartesian(r_pt, a_pt)
                    # Ignore points belonging to the tracked target person's legs (within 0.50m)
                    if math.sqrt((px - target_x)**2 + (py - target_y)**2) < 0.50:
                        continue

                    if -35 < a_pt < 35:
                        if r_pt < obs_f_min: obs_f_min = r_pt
                    elif 35 <= a_pt < 75:
                        if r_pt < obs_l_min: obs_l_min = r_pt
                    elif -75 < a_pt <= -35:
                        if r_pt < obs_r_min: obs_r_min = r_pt
                    elif abs(a_pt) > 135:
                        if r_pt < obs_b_min: obs_b_min = r_pt

                out_obs_f.value = obs_f_min
                out_obs_l.value = obs_l_min
                out_obs_r.value = obs_r_min
                out_obs_b.value = obs_b_min

            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        laser.turnOff()
        laser.disconnecting()

# ==========================================
# 6. Main Tracking Loop
# ==========================================
print("=" * 50)
print("  PERSON FOLLOWING (Kalman Filter + Sensor Fusion)")
print("=" * 50)
print("Tracking started! Stand in front of the camera.")
l_process = Process(target=lidar_process_func, args=(lidar_out_dist, lidar_out_angle, cam_in_angle, obs_front, obs_left, obs_right, obs_back), daemon=True)
l_process.start()

frames_since_target = 999
tracking_active = False
last_left_cmd = 0.0
last_right_cmd = 0.0

try:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            time.sleep(0.01)
            continue

        small = cv2.resize(image, (PROC_W, PROC_H))
        small_rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        results = pose.process(small_rgb)

        target_found_cam = False
        cam_angle_raw = 999.0
        
        # Vision processing
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            target_found_cam = True
            tracking_active = True
            frames_since_target = 0

            center_x_norm, _ = get_body_center(landmarks)
            if center_x_norm is None:
                l_sh = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
                r_sh = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
                center_x_norm = (l_sh.x + r_sh.x) / 2.0
                
            cam_angle_raw = -(center_x_norm - 0.5) * 60.0

        # Send camera angle to LiDAR thread for sensor fusion mapping
        cam_in_angle.value = cam_angle_raw if target_found_cam else 999.0
        
        # Read the SMOOTHED mapped LiDAR target & Obstacle sectors
        target_dist = lidar_out_dist.value
        target_angle = lidar_out_angle.value

        obs_f = obs_front.value
        obs_l = obs_left.value
        obs_r = obs_right.value
        obs_b = obs_back.value

        # ==========================================
        # SENSOR FUSION MOTOR CONTROL
        # ==========================================
        drive_speed = 0.0
        turn_speed = 0.0

        if target_dist > 0.0:
            # We have a valid Kalman Filter object!
            
            # 1. Proportional Steering
            if abs(target_angle) > 5.0:
                turn_speed = target_angle * K_TURN
                turn_speed = clamp(turn_speed, -MAX_TURN_SPEED, MAX_TURN_SPEED)
                
            # 2. Proportional Drive (Maintain 1.0m distance)
            dist_err = target_dist - TARGET_DIST
            if abs(dist_err) > DIST_DEADBAND:
                drive_speed = dist_err * K_DRIVE
                drive_speed = clamp(drive_speed, -MAX_SPEED, MAX_SPEED)
            
            # 3. SMOOTH OBSTACLE AVOIDANCE OVERRIDES
            if obs_f < 0.60 and drive_speed > 0:
                # Smoothly brake as obstacle approaches from 0.6m to 0.35m
                brake_factor = (obs_f - 0.35) / 0.25 # 1.0 at 0.6m, 0.0 at 0.35m
                brake_factor = clamp(brake_factor, 0.0, 1.0)
                drive_speed *= brake_factor
                if brake_factor < 0.2:
                    print(f"[OBSTACLE AVOID] Front obstacle {obs_f:.2f}m! Braking.")
            
            if obs_b < 0.60 and drive_speed < 0:
                # Smoothly brake as obstacle approaches from rear when reversing
                brake_factor = (obs_b - 0.35) / 0.25
                brake_factor = clamp(brake_factor, 0.0, 1.0)
                drive_speed *= brake_factor
                if brake_factor < 0.2:
                    print(f"[OBSTACLE AVOID] Rear obstacle {obs_b:.2f}m! Braking reverse.")

            if obs_l < 0.50:
                # Smoothly steer right away from left obstacle
                repulsion = (0.50 - obs_l) * 1.5 # max 0.30 at 0.3m
                turn_speed -= clamp(repulsion, 0.0, 0.30)
            elif obs_r < 0.50:
                # Smoothly steer left away from right obstacle
                repulsion = (0.50 - obs_r) * 1.5
                turn_speed += clamp(repulsion, 0.0, 0.30)
                
            turn_speed = clamp(turn_speed, -MAX_TURN_SPEED, MAX_TURN_SPEED)

            # 4. HARD SOFTWARE DEADBAND FOR JITTER ELIMINATION
            if abs(drive_speed) < 0.05: drive_speed = 0.0
            if abs(turn_speed) < 0.05: turn_speed = 0.0

            src = "CAM+LiDAR" if target_found_cam else "LiDAR ONLY"
            print(f"[{src}] Dist:{target_dist:.2f}m Ang:{target_angle:.0f}° | Drive:{drive_speed:.2f} Turn:{turn_speed:.2f} | ObsF:{obs_f:.2f}m ObsB:{obs_b:.2f}m")
            
        else:
            frames_since_target += 1
            if frames_since_target <= TARGET_LOST_GRACE:
                decay = TARGET_LOST_DECAY ** frames_since_target
                drive_speed = last_left_cmd * decay
                turn_speed = last_right_cmd * decay
                print("[LOST] Decaying speed...")
            else:
                print("[LOST] Waiting for target...")
                drive_speed = 0.0
                turn_speed = 0.0
                tracking_active = False

        left_power = clamp(drive_speed - turn_speed, -MAX_SPEED, MAX_SPEED)
        right_power = clamp(drive_speed + turn_speed, -MAX_SPEED, MAX_SPEED)
        
        last_left_cmd = left_power
        last_right_cmd = right_power
        set_motors(left_power, right_power)
        
except KeyboardInterrupt:
    print("\nProgram stopped by user.")
finally:
    my_robot.stop()
    l_process.terminate()
    cap.release()
    cv2.destroyAllWindows()
    print("Motors stopped safely.")
