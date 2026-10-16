# DroneSwarm-Suite: Multi-UAV Formation & Coordination Engine

An industrial-grade simulation and control engine for Multi-UAV Autonomous Swarm Coordination using **Consensus Protocols**, **Graph Laplacian Dynamics**, and **Artificial Potential Fields (APF)** for dynamic collision avoidance.

---

## Core Features

* **Decentralized Consensus Protocol:** Graph Laplacian-based state trajectory matching for leader-follower structures.
* **Dynamic Formation Switching:** Instantaneous mathematical transition between `V_SHAPE`, `LINE`, and `DIAMOND` formations.
* **APF Obstacle Avoidance:** Real-time force field calculation pushing swarm agents safely around dynamic hazards.
* **Tactical Swarm Radar HUD:** Real-time visualization of communication links, velocity vectors, and hazard boundaries.
* **Telemetry Data Export:** Automated flight trajectory logger exporting to `swarm_telemetry.csv`.

---

## Mathematical Formulation

The force vector $\mathbf{F}_i$ applied to follower agent $i$ is defined by:

$$\mathbf{F}_i = -k_p \sum_{j \in \mathcal{N}_i} a_{ij} \left( (\mathbf{p}_i - \mathbf{p}_j) - \mathbf{d}_{ij}^* \right) - k_v (\mathbf{v}_i - \mathbf{v}_{\text{leader}}) + \mathbf{F}_{\text{APF}}$$

---

## Execution Guide

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Run Swarm Simulator
python drone_swarm_engine.py