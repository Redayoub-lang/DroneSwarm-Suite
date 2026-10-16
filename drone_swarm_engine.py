import numpy as np
import cv2
import time
import math
import csv


class UAVAgent:
    """Represents a single autonomous drone within the swarm network."""
    def __init__(self, uav_id, initial_pos, is_leader=False):
        self.id = uav_id
        self.pos = np.array(initial_pos, dtype=np.float64)  # [x, y, z]
        self.vel = np.zeros(3, dtype=np.float64)            # [vx, vy, vz]
        self.acc = np.zeros(3, dtype=np.float64)            # [ax, ay, az]
        self.is_leader = is_leader
        
        # Physical constraints
        self.max_speed = 3.5     # m/s
        self.max_force = 8.0     # m/s^2

    def update_kinematics(self, dt):
        """Integrate acceleration and velocity using Euler method."""
        # Speed saturation limit
        speed = np.linalg.norm(self.vel)
        if speed > self.max_speed:
            self.vel = (self.vel / speed) * self.max_speed

        self.pos += self.vel * dt + 0.5 * self.acc * (dt ** 2)
        self.vel += self.acc * dt
        self.acc = np.zeros(3)  # Reset force accumulator


class SwarmController:
    """Swarm Orchestrator running Consensus Protocols and APF Obstacle Avoidance."""
    def __init__(self, num_followers=4):
        self.agents = []
        
        # Instantiate 1 Leader (ID 0) and N Followers
        self.agents.append(UAVAgent(uav_id=0, initial_pos=[200.0, 300.0, 10.0], is_leader=True))
        for i in range(1, num_followers + 1):
            start_x = 100.0 + (i * 30)
            start_y = 300.0 + (i * 20)
            self.agents.append(UAVAgent(uav_id=i, initial_pos=[start_x, start_y, 10.0], is_leader=False))

        # Control Gains
        self.kp_formation = 2.5
        self.kv_consensus = 1.2
        self.k_repulsion = 8000.0
        self.d_safe_obstacle = 60.0  # Safe boundary distance in pixels/meters

        # Active Formations (Relative offsets relative to Leader)
        self.current_formation_name = "V_SHAPE"
        self.formation_offsets = self._generate_offsets("V_SHAPE")

        # Obstacles list [x, y, radius]
        self.obstacles = [
            {"pos": np.array([450.0, 280.0, 10.0]), "radius": 35.0},
            {"pos": np.array([700.0, 360.0, 10.0]), "radius": 45.0}
        ]

    def _generate_offsets(self, shape_name):
        """Pre-calculate relative offset vectors d_ij for followers."""
        offsets = {}
        if shape_name == "V_SHAPE":
            offsets[1] = np.array([-40.0, -40.0, 0.0])
            offsets[2] = np.array([-40.0,  40.0, 0.0])
            offsets[3] = np.array([-80.0, -80.0, 0.0])
            offsets[4] = np.array([-80.0,  80.0, 0.0])
        elif shape_name == "LINE":
            offsets[1] = np.array([0.0, -50.0, 0.0])
            offsets[2] = np.array([0.0,  50.0, 0.0])
            offsets[3] = np.array([0.0, -100.0, 0.0])
            offsets[4] = np.array([0.0,  100.0, 0.0])
        elif shape_name == "DIAMOND":
            offsets[1] = np.array([-40.0, -40.0, 0.0])
            offsets[2] = np.array([-40.0,  40.0, 0.0])
            offsets[3] = np.array([-80.0,   0.0, 0.0])
            offsets[4] = np.array([-120.0,  0.0, 0.0])
        return offsets

    def set_formation(self, shape_name):
        """Switch active swarm shape dynamically."""
        if shape_name in ["V_SHAPE", "LINE", "DIAMOND"]:
            self.current_formation_name = shape_name
            self.formation_offsets = self._generate_offsets(shape_name)

    def compute_apf_repulsion(self, agent_pos):
        """Calculate Artificial Potential Field repulsive forces against obstacles."""
        f_rep = np.zeros(3)
        for obs in self.obstacles:
            dist_vec = agent_pos - obs["pos"]
            dist = np.linalg.norm(dist_vec) - obs["radius"]
            
            if 0 < dist < self.d_safe_obstacle:
                gain = self.k_repulsion * ((1.0 / dist) - (1.0 / self.d_safe_obstacle)) * (1.0 / (dist ** 2))
                unit_vec = dist_vec / (np.linalg.norm(dist_vec) + 1e-6)
                f_rep += gain * unit_vec
        return f_rep

    def step_simulation(self, dt, target_leader_vel):
        """Execute one control loop iteration across graph topology."""
        leader = self.agents[0]
        leader.vel = np.array(target_leader_vel, dtype=np.float64)
        leader.update_kinematics(dt)

        # Compute Consensus and Formation Forces for Followers
        for agent in self.agents[1:]:
            # Target position relative to leader
            target_pos = leader.pos + self.formation_offsets[agent.id]
            
            # Position error and Consensus velocity matching
            pos_error = target_pos - agent.pos
            vel_error = leader.vel - agent.vel
            
            f_formation = self.kp_formation * pos_error
            f_consensus = self.kv_consensus * vel_error
            f_repulsion = self.compute_apf_repulsion(agent.pos)

            # Total Force Accumulation
            agent.acc = f_formation + f_consensus + f_repulsion
            agent.update_kinematics(dt)


class SwarmDashboard:
    """Tactical Visualizer rendering real-time Swarm Telemetry & Radar Grid."""
    def __init__(self, width=1000, height=650):
        self.width = width
        self.height = height

    def render(self, controller):
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # 1. Draw Radar Grid Lines
        for x in range(0, self.width, 50):
            cv2.line(canvas, (x, 0), (x, self.height), (20, 25, 25), 1)
        for y in range(0, self.height, 50):
            cv2.line(canvas, (0, y), (self.width, y), (20, 25, 25), 1)

        # 2. Render Obstacles with Repulsion Field Boundaries
        for obs in controller.obstacles:
            cx, cy = int(obs["pos"][0]), int(obs["pos"][1])
            rad = int(obs["radius"])
            safe_rad = int(rad + controller.d_safe_obstacle)
            
            cv2.circle(canvas, (cx, cy), safe_rad, (0, 0, 100), 1, cv2.LINE_AA)  # Danger Zone
            cv2.circle(canvas, (cx, cy), rad, (0, 0, 255), -1, cv2.LINE_AA)       # Obstacle Core
            cv2.putText(canvas, "OBSTACLE", (cx - 25, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # 3. Render Topology Communication Links
        leader_pos = (int(controller.agents[0].pos[0]), int(controller.agents[0].pos[1]))
        for agent in controller.agents[1:]:
            follower_pos = (int(agent.pos[0]), int(agent.pos[1]))
            cv2.line(canvas, leader_pos, follower_pos, (0, 150, 255), 1, cv2.LINE_AA)

        # 4. Render UAV Agents
        for agent in controller.agents:
            px, py = int(agent.pos[0]), int(agent.pos[1])
            if agent.is_leader:
                cv2.circle(canvas, (px, py), 9, (0, 255, 255), -1, cv2.LINE_AA)
                cv2.putText(canvas, "LEADER (UAV-0)", (px + 12, py - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            else:
                cv2.circle(canvas, (px, py), 6, (0, 255, 0), -1, cv2.LINE_AA)
                cv2.putText(canvas, f"UAV-{agent.id}", (px + 10, py + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        # 5. Render HUD Control Overlay
        cv2.rectangle(canvas, (15, 15), (320, 125), (10, 10, 10), -1)
        cv2.rectangle(canvas, (15, 15), (320, 125), (0, 255, 0), 1)
        
        cv2.putText(canvas, f"SWARM MODE: {controller.current_formation_name}", (25, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(canvas, f"ACTIVE AGENTS: {len(controller.agents)} (1 Leader, {len(controller.agents)-1} Followers)",
                    (25, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)
        cv2.putText(canvas, "CONTROLS: [1] V-Shape | [2] Line | [3] Diamond", (25, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)
        cv2.putText(canvas, "PRESS [Q] TO EXIT", (25, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 255), 1)

        return canvas


def main():
    controller = SwarmController(num_followers=4)
    dashboard = SwarmDashboard(width=1000, height=650)
    
    # Telemetry CSV Logger Setup
    log_file = open("swarm_telemetry.csv", "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(["Timestamp", "UAV_ID", "Pos_X", "Pos_Y", "Vel_X", "Vel_Y", "Formation"])

    dt = 0.05
    start_time = time.time()

    print("[INFO] Starting Multi-UAV Swarm Simulation Engine...")
    print("[INFO] Use keys '1', '2', '3' to change formations dynamically.")

    while True:
        elapsed_time = time.time() - start_time
        
        # Leader Path Dynamics (Moving across screen)
        target_vx = 25.0
        target_vy = 8.0 * math.sin(elapsed_time * 0.8)
        
        # Controller Update Step
        controller.step_simulation(dt, target_leader_vel=[target_vx, target_vy, 0.0])

        # Log Telemetry
        for agent in controller.agents:
            writer.writerow([
                round(elapsed_time, 2), agent.id,
                round(agent.pos[0], 2), round(agent.pos[1], 2),
                round(agent.vel[0], 2), round(agent.vel[1], 2),
                controller.current_formation_name
            ])

        # Render Dashboard
        frame = dashboard.render(controller)
        cv2.imshow("Multi-UAV Swarm Coordination & Formation Engine", frame)

        # Keyboard Interactivity
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            controller.set_formation("V_SHAPE")
        elif key == ord('2'):
            controller.set_formation("LINE")
        elif key == ord('3'):
            controller.set_formation("DIAMOND")

    log_file.close()
    cv2.destroyAllWindows()
    print("[INFO] Swarm Simulation terminated. Telemetry saved to 'swarm_telemetry.csv'.")


if __name__ == "__main__":
    main()