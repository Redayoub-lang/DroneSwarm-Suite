import numpy as np
import cv2
import time
import math
import csv


class UAVAgent3D:
    """Industrial 3D Autonomous UAV Agent with SE(3) State Kinematics."""
    def __init__(self, uav_id, initial_pos, is_leader=False):
        self.id = uav_id
        self.pos = np.array(initial_pos, dtype=np.float64)  # [x, y, z]
        self.vel = np.zeros(3, dtype=np.float64)            # [vx, vy, vz]
        self.acc = np.zeros(3, dtype=np.float64)            # [ax, ay, az]
        self.is_leader = is_leader
        self.is_active = True                               # Health state for fault tolerance
        
        # Physical Constraints
        self.max_speed = 4.5    # m/s
        self.max_force = 12.0   # m/s^2

    def update_kinematics(self, dt):
        """3D Numerical Integration with Speed Saturation."""
        if not self.is_active:
            self.vel = np.zeros(3)
            self.acc = np.zeros(3)
            return

        speed = np.linalg.norm(self.vel)
        if speed > self.max_speed:
            self.vel = (self.vel / speed) * self.max_speed

        self.pos += self.vel * dt + 0.5 * self.acc * (dt ** 2)
        # Ensure floor collision protection (z >= 2.0m)
        if self.pos[2] < 2.0:
            self.pos[2] = 2.0
            self.vel[2] = max(0.0, self.vel[2])

        self.vel += self.acc * dt
        self.acc = np.zeros(3)


class SwarmController3D:
    """3D Swarm Orchestrator with Self-Healing Topology & Dynamic 3D Formations."""
    def __init__(self, num_followers=5):
        self.agents = []
        
        # Instantiate 1 Leader (ID 0) and N Followers in 3D
        self.agents.append(UAVAgent3D(uav_id=0, initial_pos=[150.0, 250.0, 20.0], is_leader=True))
        for i in range(1, num_followers + 1):
            start_x = 100.0 + (i * 25)
            start_y = 200.0 + (i * 15)
            start_z = 10.0 + (i * 5)
            self.agents.append(UAVAgent3D(uav_id=i, initial_pos=[start_x, start_y, start_z]))

        # Control Gains
        self.kp_formation = 3.2
        self.kv_consensus = 1.5
        self.k_repulsion = 12000.0
        self.d_safe_obstacle = 75.0
        self.r_comm_range = 300.0

        # Active 3D Formations
        self.current_formation_name = "3D_CUBE"
        self.formation_offsets = self._generate_3d_offsets("3D_CUBE")

        # 3D Spherical Obstacles [x, y, z, radius]
        self.obstacles = [
            {"pos": np.array([450.0, 260.0, 25.0]), "radius": 40.0},
            {"pos": np.array([720.0, 320.0, 35.0]), "radius": 50.0}
        ]

    def _generate_3d_offsets(self, shape_name):
        """Generates 3D Relative Displacement Offsets d_ij in R^3."""
        offsets = {}
        if shape_name == "3D_CUBE":
            offsets[1] = np.array([-40.0, -40.0,  15.0])
            offsets[2] = np.array([-40.0,  40.0,  15.0])
            offsets[3] = np.array([-40.0, -40.0, -15.0])
            offsets[4] = np.array([-40.0,  40.0, -15.0])
            offsets[5] = np.array([-80.0,   0.0,   0.0])
        elif shape_name == "SPHERE":
            radius = 50.0
            for i in range(1, 6):
                phi = math.acos(-1 + (2 * i) / 6)
                theta = math.sqrt(6 * math.pi) * phi
                offsets[i] = np.array([
                    radius * math.cos(theta) * math.sin(phi) - 40.0,
                    radius * math.sin(theta) * math.sin(phi),
                    radius * math.cos(phi)
                ])
        elif shape_name == "HELIX":
            for i in range(1, 6):
                angle = i * (2 * math.pi / 5)
                offsets[i] = np.array([-30.0 * i, 40.0 * math.sin(angle), 20.0 * math.cos(angle)])
        elif shape_name == "3D_V_SHAPE":
            offsets[1] = np.array([-40.0, -40.0,  10.0])
            offsets[2] = np.array([-40.0,  40.0,  10.0])
            offsets[3] = np.array([-80.0, -80.0,  20.0])
            offsets[4] = np.array([-80.0,  80.0,  20.0])
            offsets[5] = np.array([-120.0,  0.0,  30.0])
        return offsets

    def set_formation(self, shape_name):
        """Switch 3D formation geometry."""
        if shape_name in ["3D_CUBE", "SPHERE", "HELIX", "3D_V_SHAPE"]:
            self.current_formation_name = shape_name
            self.formation_offsets = self._generate_3d_offsets(shape_name)

    def trigger_fault_injection(self, agent_id):
        """Simulate hardware failure on an agent to trigger self-healing topology."""
        for agent in self.agents:
            if agent.id == agent_id and not agent.is_leader:
                agent.is_active = False
                print(f"[ALERT] FAULT INJECTED: UAV-{agent_id} Failure! Self-Healing Graph activated.")

    def compute_3d_apf_repulsion(self, agent_pos):
        """Calculates 3D Artificial Potential Field repulsion vector."""
        f_rep = np.zeros(3)
        for obs in self.obstacles:
            dist_vec = agent_pos - obs["pos"]
            dist_3d = np.linalg.norm(dist_vec) - obs["radius"]
            
            if 0 < dist_3d < self.d_safe_obstacle:
                gain = self.k_repulsion * ((1.0 / dist_3d) - (1.0 / self.d_safe_obstacle)) * (1.0 / (dist_3d ** 2))
                unit_vec = dist_vec / (np.linalg.norm(dist_vec) + 1e-6)
                f_rep += gain * unit_vec
        return f_rep

    def step_simulation(self, dt, target_leader_vel):
        """Executes one iteration of Self-Healing Consensus Control in R^3."""
        leader = self.agents[0]
        if leader.is_active:
            leader.vel = np.array(target_leader_vel, dtype=np.float64)
            leader.update_kinematics(dt)

        # Dynamic Adjacency Reconstruction for Active Agents
        active_agents = [a for a in self.agents if a.is_active]

        for agent in active_agents:
            if agent.is_leader:
                continue

            target_pos = leader.pos + self.formation_offsets[agent.id]
            pos_error = target_pos - agent.pos
            vel_error = leader.vel - agent.vel

            f_formation = self.kp_formation * pos_error
            f_consensus = self.kv_consensus * vel_error
            f_repulsion = self.compute_3d_apf_repulsion(agent.pos)

            # Inter-agent collision avoidance (Separation force)
            f_inter_uav = np.zeros(3)
            for other in active_agents:
                if other.id != agent.id:
                    uav_dist_vec = agent.pos - other.pos
                    uav_dist = np.linalg.norm(uav_dist_vec)
                    if 0 < uav_dist < 35.0:  # Safety bubble
                        f_inter_uav += (uav_dist_vec / uav_dist) * (35.0 - uav_dist) * 15.0

            agent.acc = f_formation + f_consensus + f_repulsion + f_inter_uav
            agent.update_kinematics(dt)


class Swarm3DDashboard:
    """Tactical Dual-Projection (2D Top-Down + 3D Height Map) Visualization Overlay."""
    def __init__(self, width=1050, height=680):
        self.w = width
        self.h = height

    def render(self, controller):
        canvas = np.zeros((self.h, self.w, 3), dtype=np.uint8)

        # 1. Spatial Grid Layout
        for x in range(0, self.w, 50):
            cv2.line(canvas, (x, 0), (x, self.h), (20, 25, 25), 1)
        for y in range(0, self.h, 50):
            cv2.line(canvas, (0, y), (self.w, y), (20, 25, 25), 1)

        # 2. Render 3D Obstacles (Projection with Altitude Ring)
        for obs in controller.obstacles:
            cx, cy = int(obs["pos"][0]), int(obs["pos"][1])
            rad = int(obs["radius"])
            safe_rad = int(rad + controller.d_safe_obstacle)
            
            cv2.circle(canvas, (cx, cy), safe_rad, (0, 0, 80), 1, cv2.LINE_AA)
            cv2.circle(canvas, (cx, cy), rad, (0, 0, 220), -1, cv2.LINE_AA)
            cv2.putText(canvas, f"3D OBS [Z:{obs['pos'][2]:.0f}m]", (cx - 35, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # 3. Communication Links & 3D Topology Lines
        active_agents = [a for a in controller.agents if a.is_active]
        leader = controller.agents[0]
        l_pos = (int(leader.pos[0]), int(leader.pos[1]))

        for agent in active_agents:
            if not agent.is_leader:
                f_pos = (int(agent.pos[0]), int(agent.pos[1]))
                cv2.line(canvas, l_pos, f_pos, (0, 160, 255), 1, cv2.LINE_AA)

        # 4. Render UAV Agents with Height-based Color Coding (Z-axis)
        for agent in controller.agents:
            px, py = int(agent.pos[0]), int(agent.pos[1])
            pz = agent.pos[2]

            if not agent.is_active:
                cv2.drawMarker(canvas, (px, py), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 12, 2)
                cv2.putText(canvas, f"UAV-{agent.id} [OFFLINE]", (px + 10, py + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
                continue

            # Altitude-based dynamic color (Green -> Cyan -> Magenta)
            z_color = (int(np.clip(255 - pz * 4, 50, 255)),
                       int(np.clip(pz * 6, 100, 255)),
                       int(np.clip(pz * 8, 50, 255)))

            if agent.is_leader:
                cv2.circle(canvas, (px, py), 10, (0, 255, 255), -1, cv2.LINE_AA)
                cv2.putText(canvas, f"LEADER (Z:{pz:.1f}m)", (px + 14, py - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
            else:
                cv2.circle(canvas, (px, py), 7, z_color, -1, cv2.LINE_AA)
                cv2.putText(canvas, f"UAV-{agent.id} (Z:{pz:.1f}m)", (px + 10, py + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1)

        # 5. Render HUD Tactical Panel
        cv2.rectangle(canvas, (15, 15), (380, 145), (10, 10, 10), -1)
        cv2.rectangle(canvas, (15, 15), (380, 145), (0, 255, 0), 1)

        active_count = len(active_agents)
        cv2.putText(canvas, f"3D SWARM ENGINE v2.0 [{controller.current_formation_name}]", (25, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.putText(canvas, f"ACTIVE TOPOLOGY: {active_count}/{len(controller.agents)} Agents Online", (25, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0) if active_count == len(controller.agents) else (0, 165, 255), 1)
        cv2.putText(canvas, "FORMATIONS: [1] Cube | [2] Sphere | [3] Helix | [4] V-Shape", (25, 82),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (200, 200, 200), 1)
        cv2.putText(canvas, "FAULT INJECTION: Press [F] to disable UAV-3 (Self-Healing)", (25, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 150, 255), 1)
        cv2.putText(canvas, "PRESS [Q] TO EXIT", (25, 128),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

        return canvas


def main():
    controller = SwarmController3D(num_followers=5)
    dashboard = Swarm3DDashboard(width=1050, height=680)

    log_file = open("swarm_telemetry_3d.csv", "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(["Timestamp", "UAV_ID", "Pos_X", "Pos_Y", "Pos_Z", "Vel_X", "Vel_Y", "Vel_Z", "Formation", "Is_Active"])

    dt = 0.05
    start_time = time.time()

    print("[INFO] Launching Industrial 3D Multi-UAV Swarm Engine (v2.0)...")

    while True:
        elapsed_time = time.time() - start_time

        # Leader 3D Spatial Trajectory
        target_vx = 26.0
        target_vy = 10.0 * math.sin(elapsed_time * 0.7)
        target_vz = 3.0 * math.cos(elapsed_time * 0.5)

        controller.step_simulation(dt, target_leader_vel=[target_vx, target_vy, target_vz])

        # Log 3D Telemetry
        for agent in controller.agents:
            writer.writerow([
                round(elapsed_time, 2), agent.id,
                round(agent.pos[0], 2), round(agent.pos[1], 2), round(agent.pos[2], 2),
                round(agent.vel[0], 2), round(agent.vel[1], 2), round(agent.vel[2], 2),
                controller.current_formation_name, 1 if agent.is_active else 0
            ])

        frame = dashboard.render(controller)
        cv2.imshow("Multi-UAV 3D Swarm Coordination & Self-Healing Engine v2.0", frame)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            controller.set_formation("3D_CUBE")
        elif key == ord('2'):
            controller.set_formation("SPHERE")
        elif key == ord('3'):
            controller.set_formation("HELIX")
        elif key == ord('4'):
            controller.set_formation("3D_V_SHAPE")
        elif key == ord('f'):
            controller.trigger_fault_injection(agent_id=3)

    log_file.close()
    cv2.destroyAllWindows()
    print("[INFO] 3D Swarm Simulation ended. Telemetry exported to 'swarm_telemetry_3d.csv'.")


if __name__ == "__main__":
    main()