import socket
import math
import random
import matplotlib.pyplot as plt
import msvcrt
from collections import defaultdict

HOST = "192.168.0.44"
PORT = 5000

EXPLORE_STEPS = 50 #Value set for the last run, adjust as necessary
RETURN_STEPS = 60
ATTEMPT_PHYSICAL_RTO = True

LEFT_TOO_CLOSE_CM = 30.0
LEFT_LOST_CM = 95.0
FRONT_BLOCKED_CM = 20.0
FRONT_LEFT_BLOCKED_CM = 22.0
RETURN_FRONT_BLOCKED_CM = 18.0
RETURN_FRONT_LEFT_BLOCKED_CM = 20.0

RETURN_STOP_CM = 8.0
RETURN_ANGLE_THRESH_DEG = 25.0
RETURN_ABORT_IF_WORSE_COUNT = 5
STEP_DISTANCE_EST = 2.0
TURN_ANGLE_EST = 15.0
NUM_PARTICLES = 120

SCAN_MATCH_SIGMA = 8.0
MOTION_POS_SIGMA = 0.35
MOTION_YAW_SIGMA = 1.5
MAP_POINT_STRIDE = 3
MIN_DIST_CM = 20.0
MAX_DIST_CM = 180.0
GRID_SIZE_CM = 5.0
MIN_CELL_HITS = 3
DBSCAN_EPS_CM = 14.0

DBSCAN_MIN_SAMPLES = 4
RANSAC_ITERATIONS = 80
RANSAC_DIST_THRESH_CM = 5.0
RANSAC_MIN_INLIERS = 6

LANDMARK_ANGLE_THRESH_DEG = 18.0
LANDMARK_MIDPOINT_THRESH_CM = 25.0
LANDMARK_WEIGHT_BLEND = 0.0
MIN_STABLE_LANDMARK_SEEN = 3

stop_flag = False

class Particle:
    def __init__(self, x=0.0, y=0.0, yaw_deg=0.0, weight=1.0):
        self.x = x
        self.y = y
        self.yaw_deg = yaw_deg
        self.weight = weight
        self.path = [(x, y, yaw_deg)]
    def copy(self):
        p = Particle(self.x, self.y, self.yaw_deg, self.weight)
        p.path = self.path.copy()
        return p
    
def wrap_angle_deg(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a

def commanded_motion_update(p, action, raw_dyaw=0.0):
    if action == "MOVE":
        p.yaw_deg = wrap_angle_deg(
            p.yaw_deg + raw_dyaw + random.gauss(0, MOTION_YAW_SIGMA)
        )
        dist = STEP_DISTANCE_EST + random.gauss(0, MOTION_POS_SIGMA)
        yaw_rad = math.radians(p.yaw_deg)
        p.x += dist * math.sin(yaw_rad)
        p.y += dist * math.cos(yaw_rad)
    elif action == "TURNL":
        p.yaw_deg = wrap_angle_deg(
            p.yaw_deg - TURN_ANGLE_EST + random.gauss(0, MOTION_YAW_SIGMA)
        )
    elif action == "TURNR":
        p.yaw_deg = wrap_angle_deg(
            p.yaw_deg + TURN_ANGLE_EST + random.gauss(0, MOTION_YAW_SIGMA)
        )
    p.path.append((p.x, p.y, p.yaw_deg))
    
def scan_to_world(scan, pose):
    px, py, yaw_deg = pose
    yaw_rad = math.radians(yaw_deg)
    pts = []
    for angle_deg, dist_cm in scan:
        theta = math.radians(angle_deg)
        lx = dist_cm * math.cos(theta)
        ly = dist_cm * math.sin(theta)
        wx = px + lx * math.cos(yaw_rad) - ly * math.sin(yaw_rad)
        wy = py + lx * math.sin(yaw_rad) + ly * math.cos(yaw_rad)
        pts.append((wx, wy))
    return pts

def consolidate_map(points):
    cells = defaultdict(list)
    for x, y in points:
        gx = round(x / GRID_SIZE_CM)
        gy = round(y / GRID_SIZE_CM)
        cells[(gx, gy)].append((x, y))
    consolidated = []
    for _, pts in cells.items():
        if len(pts) >= MIN_CELL_HITS:
            avg_x = sum(p[0] for p in pts) / len(pts)
            avg_y = sum(p[1] for p in pts) / len(pts)
            consolidated.append((avg_x, avg_y, len(pts)))
    return consolidated

def dbscan_clusters(clean_map, eps=DBSCAN_EPS_CM, min_samples=DBSCAN_MIN_SAMPLES):
    pts = [(x, y) for x, y, hits in clean_map]
    clusters = []
    visited = set()
    assigned = set()
    def neighbors(i):
        x, y = pts[i]
        return [
            j for j, (x2, y2) in enumerate(pts)
            if math.hypot(x - x2, y - y2) <= eps
        ]
    for i in range(len(pts)):
        if i in visited:
            continue
        visited.add(i)
        nbs = neighbors(i)
        if len(nbs) < min_samples:
            continue
        cluster = []
        queue = list(nbs)
        while queue:
            j = queue.pop()
            if j not in visited:
                visited.add(j)
                j_nbs = neighbors(j)
                if len(j_nbs) >= min_samples:
                    queue.extend(j_nbs)
            if j not in assigned:
                assigned.add(j)
                cluster.append(pts[j])
        if len(cluster) >= min_samples:
            clusters.append(cluster)
    return clusters

def get_clustered_points(clusters):
    clustered = set()
    for cluster in clusters:
        for x, y in cluster:
            clustered.add((round(x, 2), round(y, 2)))
    return clustered

def point_line_distance(point, line):
    x, y = point
    x1, y1, x2, y2 = line
    a = y2 - y1
    b = x1 - x2
    c = x2 * y1 - x1 * y2
    denom = math.sqrt(a * a + b * b)
    if denom == 0:
        return float("inf")
    return abs(a * x + b * y + c) / denom

def least_squares_line(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)
    if y_range > x_range:
        n = len(ys)
        mean_y = sum(ys) / n
        mean_x = sum(xs) / n
        denom = sum((y - mean_y) ** 2 for y in ys)
        if denom == 0:
            return None
        m = sum((ys[i] - mean_y) * (xs[i] - mean_x) for i in range(n)) / denom
        c = mean_x - m * mean_y
        y1, y2 = min(ys), max(ys)
        x1 = m * y1 + c
        x2 = m * y2 + c
        return x1, y1, x2, y2
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    denom = sum((x - mean_x) ** 2 for x in xs)
    if denom == 0:
        return None
    m = sum((xs[i] - mean_x) * (ys[i] - mean_y) for i in range(n)) / denom
    c = mean_y - m * mean_x
    x1, x2 = min(xs), max(xs)
    y1 = m * x1 + c
    y2 = m * x2 + c
    return x1, y1, x2, y2

def ransac_line_to_cluster(cluster):
    if len(cluster) < RANSAC_MIN_INLIERS:
        return None
    best_inliers = []
    for _ in range(RANSAC_ITERATIONS):
        p1, p2 = random.sample(cluster, 2)
        if p1 == p2:
            continue
        candidate = (p1[0], p1[1], p2[0], p2[1])
        inliers = [
            p for p in cluster
            if point_line_distance(p, candidate) <= RANSAC_DIST_THRESH_CM
        ]
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
    if len(best_inliers) < RANSAC_MIN_INLIERS:
        return None
    return least_squares_line(best_inliers)

def line_angle(line):
    x1, y1, x2, y2 = line
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

def line_midpoint(line):
    x1, y1, x2, y2 = line
    return 0.5 * (x1 + x2), 0.5 * (y1 + y2)

def line_similarity(line_a, line_b):
    angle_a = line_angle(line_a)
    angle_b = line_angle(line_b)
    angle_diff = abs(wrap_angle_deg(angle_a - angle_b))
    angle_diff = min(angle_diff, abs(180 - angle_diff))
    ax, ay = line_midpoint(line_a)
    bx, by = line_midpoint(line_b)
    mid_dist = math.hypot(ax - bx, ay - by)
    return angle_diff, mid_dist

def update_landmark_memory(landmarks, detected_lines):
    next_id = max([lm["id"] for lm in landmarks], default=-1) + 1
    for line in detected_lines:
        best_idx = None
        best_score = None
        for i, lm in enumerate(landmarks):
            old_line = (lm["x1"], lm["y1"], lm["x2"], lm["y2"])
            angle_diff, mid_dist = line_similarity(line, old_line)
            if (
                angle_diff <= LANDMARK_ANGLE_THRESH_DEG
                and mid_dist <= LANDMARK_MIDPOINT_THRESH_CM
            ):
                score = angle_diff + 0.01 * mid_dist
                if best_score is None or score < best_score:
                    best_score = score
                    best_idx = i
        x1, y1, x2, y2 = line
        if best_idx is None or best_score is None or best_score > 25:
            landmarks.append({
                "id": next_id,
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "seen": 1
            })
            next_id += 1
        else:
            lm = landmarks[best_idx]
            alpha = 0.2
            lm["x1"] = (1 - alpha) * lm["x1"] + alpha * x1
            lm["y1"] = (1 - alpha) * lm["y1"] + alpha * y1
            lm["x2"] = (1 - alpha) * lm["x2"] + alpha * x2
            lm["y2"] = (1 - alpha) * lm["y2"] + alpha * y2
            lm["seen"] += 1
            
def nearest_dist_sq(pt, map_points):
    if not map_points:
        return None
    x, y = pt
    best = None
    for mx, my in map_points[::MAP_POINT_STRIDE]:
        d2 = (x - mx) ** 2 + (y - my) ** 2
        if best is None or d2 < best:
            best = d2
    return best

def measurement_update(particles, scan, global_map):
    if not global_map:
        for p in particles:
            p.weight = 1.0 / len(particles)
        return
    sigma2 = SCAN_MATCH_SIGMA ** 2
    for p in particles:
        pts = scan_to_world(scan, (p.x, p.y, p.yaw_deg))
        score = 0.0
        used = 0
        for pt in pts:
            d2 = nearest_dist_sq(pt, global_map)
            if d2 is not None:
                score += math.exp(-0.5 * d2 / sigma2)
                used += 1
        p.weight = max(score / max(used, 1), 1e-12)
    total = sum(p.weight for p in particles)
    if total == 0:
        for p in particles:
            p.weight = 1.0 / len(particles)
    else:
        for p in particles:
            p.weight /= total
            
def systematic_resample(particles):
    n = len(particles)
    positions = [(random.random() + i) / n for i in range(n)]
    cumulative = []
    s = 0.0
    for p in particles:
        s += p.weight
        cumulative.append(s)
    new_particles = []
    i = 0
    for pos in positions:
        while pos > cumulative[i]:
            i += 1
        new_p = particles[i].copy()
        new_p.weight = 1.0 / n
        new_particles.append(new_p)
    return new_particles

def best_particle(particles):
    return max(particles, key=lambda p: p.weight)

class RobotClient:
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(30)
        self.buffer = ""
        print("Python: attempting connect...")
        self.sock.connect((host, port))
        print("Python: socket connected")
    def send(self, cmd):
        self.sock.sendall((cmd + "\n").encode())
    def read_line(self):
        while "\n" not in self.buffer:
            try:
                data = self.sock.recv(4096).decode(errors="ignore")
            except ConnectionResetError:
                raise RuntimeError("Arduino connection reset")
            if not data:
                raise RuntimeError("Connection closed by Arduino")
            self.buffer += data
        line, self.buffer = self.buffer.split("\n", 1)
        return line.strip()
    def reset(self):
        print("Python: sending RESET")
        self.send("RESET")
        while True:
            line = self.read_line()
            print("Python reset got:", repr(line))
            if line == "RESET_DONE":
                return
    def do_action(self, cmd):
        print(f"Python: sending {cmd}")
        self.send(cmd)
        while True:
            line = self.read_line()
            if not line or line == "CONNECTED":
                continue
            if line.startswith("ODOM,"):
                parts = line.split(",")
                return float(parts[1]), float(parts[2]), float(parts[3])
    def scan(self):
        print("Python: sending SCAN")
        self.send("SCAN")
        scan = []
        while True:
            line = self.read_line()
            if not line or line == "CONNECTED":
                continue
            if line == "SWEEP_START":
                scan = []
                continue
            if line == "SWEEP_END":
                continue
            if line.startswith("POSE,"):
                parts = line.split(",")
                return scan, (float(parts[1]), float(parts[2]), float(parts[3]))
            if "," in line:
                parts = line.split(",")
                if len(parts) == 2:
                    try:
                        angle = float(parts[0])
                        dist = parts[1]
                        if dist != "INVALID":
                            d = float(dist)
                            if MIN_DIST_CM <= d <= MAX_DIST_CM:
                                scan.append((angle, d))
                    except ValueError:
                        pass
    def stop(self):
        try:
            self.send("STOP")
        except Exception:
            pass
    def close(self):
        try:
            self.stop()
            self.sock.close()
        except Exception:
            pass
def sector_min(scan, a0, a1):
    vals = [d for a, d in scan if a0 <= a <= a1]
    return min(vals) if vals else 999.0

def choose_explore_action(scan):
    front_min = sector_min(scan, 70, 110)
    front_left_min = sector_min(scan, 100, 140)
    left_min = sector_min(scan, 120, 170)
    print(
        f"EXPLORE front={front_min:.1f} front_left={front_left_min:.1f} left={left_min:.1f}",
        end=" -> "
    )
    if front_min < FRONT_BLOCKED_CM or front_left_min < FRONT_LEFT_BLOCKED_CM:
        print("TURNR")
        return "TURNR"
    if left_min > LEFT_LOST_CM:
        print("TURNL")
        return "TURNL"
    if left_min < LEFT_TOO_CLOSE_CM:
        print("TURNR")
        return "TURNR"
    print("MOVE")
    return "MOVE"

def choose_return_action(scan, pose):
    x, y, yaw = pose
    dx = -x
    dy = -y
    dist = math.hypot(dx, dy)
    if dist <= RETURN_STOP_CM:
        return "DONE", dist, 0.0
    target_angle = math.degrees(math.atan2(dx, dy))
    angle_error = wrap_angle_deg(target_angle - yaw)
    front_min = sector_min(scan, 75, 105)
    front_left_min = sector_min(scan, 105, 135)
    if front_min < RETURN_FRONT_BLOCKED_CM or front_left_min < RETURN_FRONT_LEFT_BLOCKED_CM:
        return "TURNR", dist, angle_error
    if abs(angle_error) > RETURN_ANGLE_THRESH_DEG:
        if angle_error < 0:
            return "TURNL", dist, angle_error
        else:
            return "TURNR", dist, angle_error
    return "MOVE", dist, angle_error

def check_keyboard_stop():
    global stop_flag
    if msvcrt.kbhit():
        key = msvcrt.getch()
        if key == b"q":
            stop_flag = True
            print(">>> STOP KEY PRESSED")
            
robot = RobotClient(HOST, PORT)
robot.reset()
particles = [Particle() for _ in range(NUM_PARTICLES)]
global_map = []
landmarks = []
explore_path = []
return_path = []
computed_rto_error_before_return = None
try:
    scan, pose = robot.scan()
    global_map.extend(scan_to_world(scan, (0.0, 0.0, 0.0)))
    for step_idx in range(EXPLORE_STEPS):
        check_keyboard_stop()
        if stop_flag:
            break
        action = choose_explore_action(scan)
        dx_raw, dy_raw, dyaw_raw = robot.do_action(action)
        for p in particles:
            commanded_motion_update(p, action, dyaw_raw)
        scan, _ = robot.scan()
        measurement_update(particles, scan, global_map)
        particles = systematic_resample(particles)
        bp = best_particle(particles)
        explore_path.append((bp.x, bp.y, bp.yaw_deg))
        best_scan_pts = scan_to_world(scan, (bp.x, bp.y, bp.yaw_deg))
        global_map.extend(best_scan_pts)
        clean_map = consolidate_map(global_map)
        clusters = dbscan_clusters(clean_map)
        detected_lines = []
        for cluster in clusters:
            line = ransac_line_to_cluster(cluster)
            if line is not None:
                detected_lines.append(line)
        update_landmark_memory(landmarks, detected_lines)
        print(
            f"EXPLORE {step_idx + 1}/{EXPLORE_STEPS}: "
            f"action={action}, pose=({bp.x:.2f},{bp.y:.2f},{bp.yaw_deg:.2f}), "
            f"map_cells={len(clean_map)}, clusters={len(clusters)}"
        )
    bp = best_particle(particles)
    computed_rto_error_before_return = math.hypot(bp.x, bp.y)
    print("\nCOMPUTED RETURN-TO-ORIGIN METRIC")
    print(f"Estimated displacement from origin before physical RTO: {computed_rto_error_before_return:.2f} cm")
    if ATTEMPT_PHYSICAL_RTO:
        print("\nPHYSICAL RETURN-TO-ORIGIN ATTEMPT START")
        best_return_dist = computed_rto_error_before_return
        worse_count = 0
        for step_idx in range(RETURN_STEPS):
            check_keyboard_stop()
            if stop_flag:
                break
            bp = best_particle(particles)
            action, dist, angle_error = choose_return_action(scan, (bp.x, bp.y, bp.yaw_deg))
            print(
                f"RETURN {step_idx + 1}/{RETURN_STEPS}: "
                f"dist={dist:.2f}cm angle_error={angle_error:.2f} -> {action}"
            )
            if action == "DONE":
                print("Return-to-origin complete")
                break
            dx_raw, dy_raw, dyaw_raw = robot.do_action(action)
            for p in particles:
                commanded_motion_update(p, action, dyaw_raw)
            scan, _ = robot.scan()
            measurement_update(particles, scan, global_map)
            particles = systematic_resample(particles)
            bp = best_particle(particles)
            return_path.append((bp.x, bp.y, bp.yaw_deg))
            current_dist = math.hypot(bp.x, bp.y)
            if action == "MOVE":
                if current_dist < best_return_dist:
                    best_return_dist = current_dist
                    worse_count = 0
                else:
                    worse_count += 1
                if worse_count >= RETURN_ABORT_IF_WORSE_COUNT:
                    print("Return attempt aborted: estimated distance stopped improving after moves")
                    break
                
except KeyboardInterrupt:
    print("KeyboardInterrupt detected")
    
except (ConnectionResetError, TimeoutError, OSError, RuntimeError) as e:
    print("ERROR:", e)
    print("Saving whatever data was collected.")
    
finally:
    robot.close()
    final_bp = best_particle(particles)
    final_x, final_y, final_yaw = final_bp.path[-1]
    final_error = math.hypot(final_x, final_y)
    clean_map = consolidate_map(global_map)
    clusters = dbscan_clusters(clean_map)
    stable_landmarks = [lm for lm in landmarks if lm["seen"] >= MIN_STABLE_LANDMARK_SEEN]
    print("\nFINAL ESTIMATE")
    print(f"Computed RTO error before physical return: {computed_rto_error_before_return:.2f} cm")
    print(f"Final estimated x: {final_x:.2f} cm")
    print(f"Final estimated y: {final_y:.2f} cm")
    print(f"Final estimated yaw: {final_yaw:.2f} deg")
    print(f"Final estimated displacement from origin: {final_error:.2f} cm")
    print(f"Final map cells: {len(clean_map)}")
    print(f"Final DBSCAN clusters: {len(clusters)}")
    print(f"Stable landmarks: {len(stable_landmarks)}")
    fig, ax = plt.subplots(figsize=(9, 8))
    clustered_points = get_clustered_points(clusters)
    out_x = []
    out_y = []
    for x, y, hits in clean_map:
        key = (round(x, 2), round(y, 2))
        if key not in clustered_points:
            out_x.append(x)
            out_y.append(y)
    ax.scatter(out_x, out_y, s=35, alpha=0.35, label="Unclustered map points")
    for i, cluster in enumerate(clusters):
        cx = [p[0] for p in cluster]
        cy = [p[1] for p in cluster]
        ax.scatter(cx, cy, s=65, marker="x", label="DBSCAN cluster" if i == 0 else None)
    for i, lm in enumerate(stable_landmarks):
        ax.plot(
            [lm["x1"], lm["x2"]],
            [lm["y1"], lm["y2"]],
            linewidth=4,
            color="purple",
            label="Stable RANSAC landmark" if i == 0 else None
        )
    if explore_path:
        ex = [p[0] for p in explore_path]
        ey = [p[1] for p in explore_path]
        ax.plot(ex, ey, marker="o", linewidth=2, color="red", label="Explore path")
    if return_path:
        rx = [p[0] for p in return_path]
        ry = [p[1] for p in return_path]
        ax.plot(rx, ry, marker="o", linewidth=2, color="blue", label="Physical RTO attempt")
    ax.scatter([0], [0], s=160, c="black", label="Origin")
    ax.scatter([final_x], [final_y], s=120, c="magenta", label="Final estimate")
    ax.set_title("Mapping and Physical Return-to-Origin")
    ax.set_xlabel("cm")
    ax.set_ylabel("cm")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys())
    plt.savefig("slam_computed_and_physical_rto.png", dpi=300, bbox_inches="tight")
    plt.show()
