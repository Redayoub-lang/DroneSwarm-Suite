# DroneSwarm-Suite (v2.0 Industrial Edition)
## Multi-UAV 3D Swarm Coordination & Self-Healing Topology Engine

An industrial-grade 3D Multi-UAV Swarm Coordination and Control Suite built on **Graph Laplacian Consensus Dynamics in $\mathbb{R}^3$**, **Self-Healing Topology Re-indexing**, and **Predictive 3D Artificial Potential Fields (APF)**.

---

## Key Advanced Upgrades in v2.0

* **True 3D Spatial Formations ($\mathbb{R}^3$):** Dynamic mathematically defined offsets for `3D_CUBE`, `SPHERE`, `HELIX`, and `3D_V_SHAPE`.
* **Fault-Tolerant Self-Healing Graph:** Automated Laplacian matrix ($L = D - A$) topology reconstruction upon agent disconnection or hardware failure.
* **3D Predictive APF Avoidance:** Dynamic force field vectors avoiding spherical obstacles and preventing inter-agent collisions in $SE(3)$.
* **Height-Coded Tactical HUD Dashboard:** Real-time dual projection renderer displaying altitude gradients, link graphs, and active node status.
* **Flight Data Logger:** Auto-exporting 3D positional trajectories to `swarm_telemetry_3d.csv`.

---

## Mathematical Mechanics

The position vector $\mathbf{p}_i = [x_i, y_i, z_i]^T \in \mathbb{R}^3$ of each follower agent $i$ evolves according to:

$$\mathbf{a}_i = -k_p \sum_{j \in \mathcal{N}_i(t)} a_{ij}(t) \left( (\mathbf{p}_i - \mathbf{p}_j) - \mathbf{d}_{ij}^*(t) \right) - k_v \sum_{j \in \mathcal{N}_i(t)} a_{ij}(t) (\mathbf{v}_i - \mathbf{v}_j) + \mathbf{F}_{\text{APF}, i}^{\text{3D}}$$

---

## Quick Execution

```bash
# Install dependencies
pip install -r requirements.txt

# Launch 3D Swarm Engine
python drone_swarm_engine.py
Interactive Controls
1: Switch to 3D Cube Formation

2: Switch to Spherical Formation

3: Switch to Helix Formation

4: Switch to 3D V-Shape Formation

F: Inject Fault into UAV-3 to test Self-Healing Graph Topology

Q: Terminate Engine & Export Telemetry