import numpy as np
import cv2
import time
import math
import csv
import struct


class MAVLinkTelemetryBridge:
    """Enterprise MAVLink Protocol Simulator (PX4 / ArduPilot Telemetry Integration)."""
    def __init__(self, sys_id=1, comp_id=1):
        self.sys_id = sys_id
        self.comp_id = comp_id
        self.packet_sequence = 0

    def encode_set_position_target_local_ned(self, time_boot_ms, x, y, z, vx, vy, vz):
        """Encodes MAVLink MSG_ID #84 SET_POSITION_TARGET_LOCAL_NED packet."""
        self.packet_sequence = (self.packet_sequence + 1) % 256
        coordinate_frame = 1  # MAV_FRAME_LOCAL_NED
        type_mask = 0b0000111111000000  # Enable Pos + Vel
        
        # MAVLink Header & Binary Packing Simulation
        header = struct.pack("<BBBBBB", 0xFE, 28, self.packet_sequence, self.sys_id, self.comp_id, 84)
        payload = struct.pack("<IfffffffffffffH",
                              time_boot_ms, coordinate_frame, type_mask,
                              x, y, -z, vx, vy, -vz, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
        return header + payload

    def parse_heartbeat_status(self):
        """Returns simulated MAVLink System Health & Arming Status."""
        return {"armed": True, "mode": "OFFBOARD", "mavlink_version": 2}


class LossyNetworkGraph:
    """Simulates Realistic Ad-Hoc UAV Mesh Network Jitter & Packet Loss."""
    def __init__(self, packet_loss_rate=0.08, max_latency_ms=120):
        self.p_loss = packet_loss_rate
        self.max_latency = max_latency_ms
        self.delay_buffers = {}

    def transmit_state(self, sender_id, receiver_id, state_vector):
        """Simulates lossy RF link transmission with latency delay buffer."""
        link_key = (sender_id, receiver_id)
        
        # Simulate Random Packet Loss
        if np.random.rand() < self.p_loss:
            return None  # Dropped Packet

        # Simulate Network Jitter Delay
        jitter_delay = np.random.uniform(10.0, self.max_latency) / 1000.0
        
        if link_key not in self.delay_buffers:
            self.delay_buffers[link_key] = []
            
        self.delay_buffers[link_key].append((time.time() + jitter_delay, state_vector))
        
        # Deliver available delayed packet
        valid_packets = [p for p in self.delay_buffers[link_key] if time.time() >= p[0]]
        if valid_packets:
            delivered = valid_packets[-1][1]
            self.delay_buffers[link_key] = [p for p in self.delay_buffers[link_key] if p not in valid_packets]
            return delivered
        return None


class DMPCOptimizer3D:
    """Distributed Model Predictive Controller over Finite Prediction Horizon H."""
    def __init__(self, horizon=5, dt=0.08):
        self.H = horizon
        self.dt = dt
        self.a_max = 10.0  # m/s^2
        self.v_max = 5.0   # m/s

    def compute_optimal_control(self, current_pos, current_vel, target_pos, target_vel, neighbor_states):
        """Optimizes acceleration commands over horizon H subject to state constraints."""
        best_u = np.zeros(3)
        min_cost = float('inf')

        # Sample Candidate Acceleration Vectors in R^3
        acc_samples = np.random.uniform(-self.a_max, self.a_max, (24, 3))

        for u in acc_samples:
            cost = 0.0
            pred_pos = current_pos.copy()
            pred_vel = current_vel.copy()

            for step in range(self.H):
                pred_vel += u * self.dt
                # Velocity bounds constraint
                speed = np.linalg.norm(pred_vel)
                if speed > self.v_max:
                    pred_vel = (pred_vel / speed) * self.v_max

                pred_pos += pred_vel * self.dt + 0.5 * u * (self.dt ** 2)

                # 1. Target Tracking Cost
                cost += 3.0 * np.linalg.norm(pred_pos - target_pos)**2
                cost += 1.0 * np.linalg.norm(pred_vel - target_vel)**2

                # 2. Control Effort Cost
                cost += 0.2 * np.linalg.norm(u)**2

                # 3. Inter-agent Collision Penalty
                for n_pos in neighbor_states:
                    if n_pos is not None:
                        dist = np.linalg.norm(pred_pos - n_pos[:3])
                        if dist < 25.0:  # Safety margin
                            cost += 5000.0 / (dist + 1e-3)**2

            if cost < min_cost:
                min_cost = cost
                best_u = u

        return best_u


class UAVAgentEnterprise:
    """3D Industrial UAV Agent with MAVLink & DMPC Controller."""
    def __init__(self, uav_id, initial_pos, is_leader=False):
        self.id = uav_id
        self.pos = np.array(initial_pos, dtype=np.float64)
        self.vel = np.zeros(3, dtype=np.float64)
        self.acc = np.zeros(3, dtype=np.float64)
        self.is_leader = is_leader
        self.is_active = True
        
        self.mavlink_bridge = MAVLinkTelemetryBridge(sys_id=uav_id+1)
        self.dmpc = DMPCOptimizer3D(horizon=5, dt=0.08)

    def update_physics(self, dt):
        """3D Continuous Physics State Integration."""
        if not self.is_active:
            self.vel = np.zeros(3)
            self.acc = np.zeros(3)
            return

        self.pos += self.vel * dt + 0.5 * self.acc * (dt ** 2)
        if self.pos[2] < 2.0:
            self.pos[2] = 2.0
            self.vel[2] = max(0.0, self.vel[2])

        self.vel += self.acc * dt
        self.acc = np.zeros(3)


class Swarm3DControllerV3:
    """Enterprise 3D Swarm Orchestrator with Lossy Mesh Network & MAVLink Output."""
    def __init__(self, num_followers=5):
        self.agents = []
        self.network = LossyNetworkGraph(packet_loss_rate=0.06, max_latency_ms=80)

        # Initialize Leader and Followers
        self.agents.append(UAVAgentEnterprise(uav_id=0, initial_pos=[150.0, 250.0, 20.0], is_leader=True))
        for i in range(1, num_followers + 1):
            self.agents.append(UAVAgentEnterprise(uav_id=i, initial_pos=[100.0 + i*20, 200.0 + i*15, 10.0 + i*4]))

        self.current_formation = "3D_CUBE"
        self.formation_offsets = self._generate_offsets("3D_CUBE")

        self.obstacles = [
            {"pos": np.array([450.0, 260.0, 25.0]), "radius": 40.0},
            {"pos": np.array([720.0, 320.0, 35.0]), "radius": 50.0}
        ]

    def _generate_offsets(self, shape):
        offsets = {}
        if shape == "3D_CUBE":
            offsets[1] = np.array([-40.0, -40.0,  15.0])
            offsets[2] = np.array([-40.0,  40.0,  15.0])
            offsets[3] = np.array([-40.0, -40.0, -15.0])
            offsets[4] = np.array([-40.0,  40.0, -15.0])
            offsets[5] = np.array([-80.0,   0.0,   0.0])
        elif shape == "SPHERE":
            radius = 50.0
            for i in range(1, 6):
                phi = math.acos(-1 + (2 * i) / 6)
                theta = math.sqrt(6 * math.pi) * phi
                offsets[i] = np.array([radius * math.cos(theta) * math.sin(phi) - 40.0,
                                       radius * math.sin(theta) * math.sin(phi),
                                       radius * math.cos(phi)])
        return offsets

    def set_formation(self, shape):
        if shape in ["3D_CUBE", "SPHERE"]:
            self.current_formation = shape
            self.formation_offsets = self._generate_offsets(shape)

    def trigger_fault(self, uav_id):
        for agent in self.agents:
            if agent.id == uav_id and not agent.is_leader:
                agent.is_active = False
                print(f"[ALERT] FAULT INJECTED: UAV-{uav_id} Offline!")

    def step_simulation(self, dt, target_leader_vel):
        leader = self.agents[0]
        if leader.is_active:
            leader.vel = np.array(target_leader_vel)
            leader.update_physics(dt)

        active_agents = [a for a in self.agents if a.is_active]

        # Simulate Mesh Network Transmissions
        received_neighbor_states = {a.id: [] for a in active_agents}
        for sender in active_agents:
            for receiver in active_agents:
                if sender.id != receiver.id:
                    st = self.network.transmit_state(sender.id, receiver.id, np.concatenate((sender.pos, sender.vel)))
                    received_neighbor_states[receiver.id].append(st)

        # DMPC Optimization Step for Followers
        for agent in active_agents:
            if agent.is_leader:
                continue

            target_p = leader.pos + self.formation_offsets[agent.id]
            target_v = leader.vel

            # Compute DMPC Acceleration Setpoint
            acc_cmd = agent.dmpc.compute_optimal_control(
                agent.pos, agent.vel, target_p, target_v, received_neighbor_states[agent.id]
            )

            # 3D Obstacle Repulsion
            for obs in self.obstacles:
                d_vec = agent.pos - obs["pos"]
                dist = np.linalg.norm(d_vec) - obs["radius"]
                if 0 < dist < 70.0:
                    acc_cmd += (d_vec / (np.linalg.norm(d_vec) + 1e-6)) * (8000.0 / (dist**2))

            agent.acc = acc_cmd
            agent.update_physics(dt)

            # Encode MAVLink Stream
            _ = agent.mavlink_bridge.encode_set_position_target_local_ned(
                int(time.time() * 1000), agent.pos[0], agent.pos[1], agent.pos[2],
                agent.vel[0], agent.vel[1], agent.vel[2]
            )


class Swarm3DDashboardV3:
    """Tactical Rendering Interface with MAVLink Telemetry & Network Jitter Profiler."""
    def __init__(self, width=1080, height=700):
        self.w = width
        self.h = height

    def render(self, controller):
        canvas = np.zeros((self.h, self.w, 3), dtype=np.uint8)

        # Spatial Grid
        for x in range(0, self.w, 50):
            cv2.line(canvas, (x, 0), (x, self.h), (20, 22, 22), 1)
        for y in range(0, self.h, 50):
            cv2.line(canvas, (0, y), (self.w, y), (20, 22, 22), 1)

        # Obstacles
        for obs in controller.obstacles:
            cx, cy = int(obs["pos"][0]), int(obs["pos"][1])
            rad = int(obs["radius"])
            cv2.circle(canvas, (cx, cy), rad + 60, (0, 0, 70), 1, cv2.LINE_AA)
            cv2.circle(canvas, (cx, cy), rad, (0, 0, 210), -1, cv2.LINE_AA)

        # Active UAV Links & Trajectories
        active_agents = [a for a in controller.agents if a.is_active]
        leader = controller.agents[0]
        l_pos = (int(leader.pos[0]), int(leader.pos[1]))

        for agent in active_agents:
            if not agent.is_leader:
                f_pos = (int(agent.pos[0]), int(agent.pos[1]))
                cv2.line(canvas, l_pos, f_pos, (0, 180, 255), 1, cv2.LINE_AA)

        # Render UAV Nodes with MAVLink Status
        for agent in controller.agents:
            px, py = int(agent.pos[0]), int(agent.pos[1])
            pz = agent.pos[2]

            if not agent.is_active:
                cv2.drawMarker(canvas, (px, py), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 12, 2)
                cv2.putText(canvas, f"UAV-{agent.id} [OFFLINE]", (px + 10, py + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
                continue

            z_col = (int(np.clip(255 - pz * 4, 50, 255)),
                     int(np.clip(pz * 6, 100, 255)),
                     int(np.clip(pz * 8, 50, 255)))

            if agent.is_leader:
                cv2.circle(canvas, (px, py), 10, (0, 255, 255), -1, cv2.LINE_AA)
                cv2.putText(canvas, f"LEADER MAVLink#1 (Z:{pz:.1f}m)", (px + 14, py - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
            else:
                cv2.circle(canvas, (px, py), 7, z_col, -1, cv2.LINE_AA)
                cv2.putText(canvas, f"UAV-{agent.id} DMPC (Z:{pz:.1f}m)", (px + 10, py + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1)

        # HUD Dashboard Overlay
        cv2.rectangle(canvas, (15, 15), (420, 160), (10, 10, 10), -1)
        cv2.rectangle(canvas, (15, 15), (420, 160), (0, 255, 0), 1)

        cv2.putText(canvas, f"DRONE SWARM v3.0 [DMPC + MAVLink SITL]", (25, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.putText(canvas, f"MAVLINK MESH: {len(active_agents)} Nodes Online | OFFBOARD Mode", (25, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1)
        cv2.putText(canvas, "RF NETWORK: Lossy Jitter (P_Loss: 6% | Latency: ~80ms)", (25, 82),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 150, 0), 1)
        cv2.putText(canvas, "CONTROLLER: Distributed Model Predictive Control (DMPC)", (25, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
        cv2.putText(canvas, "CONTROLS: [1] Cube | [2] Sphere | [F] Fault UAV-3 | [Q] Exit", (25, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 255, 255), 1)

        return canvas


def main():
    controller = Swarm3DControllerV3(num_followers=5)
    dashboard = Swarm3DDashboardV3(width=1080, height=700)

    log_file = open("swarm_mavlink_v3_telemetry.csv", "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(["Timestamp", "UAV_ID", "Pos_X", "Pos_Y", "Pos_Z", "Vel_X", "Vel_Y", "Vel_Z", "MAVLink_Seq"])

    dt = 0.08
    start_time = time.time()

    print("[INFO] Launching Industrial 3D Multi-UAV Swarm Engine v3.0 (DMPC & MAVLink SITL)...")

    while True:
        elapsed = time.time() - start_time

        target_vx = 25.0
        target_vy = 12.0 * math.sin(elapsed * 0.8)
        target_vz = 2.5 * math.cos(elapsed * 0.4)

        controller.step_simulation(dt, target_leader_vel=[target_vx, target_vy, target_vz])

        for agent in controller.agents:
            writer.writerow([
                round(elapsed, 2), agent.id,
                round(agent.pos[0], 2), round(agent.pos[1], 2), round(agent.pos[2], 2),
                round(agent.vel[0], 2), round(agent.vel[1], 2), round(agent.vel[2], 2),
                agent.mavlink_bridge.packet_sequence
            ])

        frame = dashboard.render(controller)
        cv2.imshow("Multi-UAV 3D Swarm Coordination & MAVLink Engine v3.0", frame)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            controller.set_formation("3D_CUBE")
        elif key == ord('2'):
            controller.set_formation("SPHERE")
        elif key == ord('f'):
            controller.trigger_fault(uav_id=3)

    log_file.close()
    cv2.destroyAllWindows()
    print("[INFO] Swarm Engine v3.0 terminated. Data logged to 'swarm_mavlink_v3_telemetry.csv'.")


if __name__ == "__main__":
    main()