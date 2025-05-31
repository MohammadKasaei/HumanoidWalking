# HumanoidWalking - CPG and DRL
<div align="center">
  <img src="imgs/DRL.gif" alt="Image description" width="720">
</div>


### 🦿 Humanoid Robot Walking Engine

This repository contains the **core component** of our **end-to-end walking pipeline**, developed as part of my [PhD research](https://ria.ua.pt/bitstream/10773/33545/1/Documento_Seyed_Kasaei.pdf). It presents a robust framework for generating walking gaits in humanoid robots.

#### 🔧 Key Features:
- 🚶‍♂️ **Hybrid Walking Engine**: Combines the **Linear Inverted Pendulum Model (LIPM)** with **Central Pattern Generators (CPGs)** for trajectory generation.
- 🧠 **State Machine Control**: A modular finite-state machine orchestrates the phases of the walking cycle.
- 🔄 **Smooth Gait Generation**: Utilizes **Partial Fourier Series (PFS) oscillators** to produce smooth and continuous joint trajectories.
- 🛠️ **Stabilization Controllers**: Includes feedback mechanisms for dynamic stability during locomotion.
- 🤖 **Multi-Platform Deployment**: Successfully deployed on multiple humanoid platforms including:
  - **COMAN**
  - **Talos**
  - **Nao**

This implementation is a cornerstone of my doctoral work and contributes to the broader goal of enabling reliable, dynamic locomotion in humanoid robots.

<div style="display: flex; justify-content: space-between; align-items: center;">
  <img src="imgs/coman.png" alt="Image description" width="320">
  <img src="imgs/talos.png" alt="Image description" width="320">
</div>




# 🧠 Reward Function (High-Level Overview)

The reward function in this environment is crafted to promote **stable, energy-efficient, and goal-directed walking behavior**. It follows a simple but effective structure:

#### 📐 Reward Equation (High-Level)
```python
Reward = 1.0 - EnergyPenalty - CommandPenalty
```

- **Base Reward**: The reward starts at `1.0` each step.
- **Energy Penalty**: Encourages efficient behavior by penalizing the magnitude of the action vector. Actions affecting torso, hips, ankles, and arms are weighted differently based on their physical influence.
- **Command Penalty**: Measures how far the robot deviates from a desired motion command (e.g., forward velocity or step direction). This ensures purposeful walking.

- **Failure Detection**: The episode terminates early if the robot falls or makes undesired contact with the ground (non-foot links), acting as an implicit penalty.



> To ensure learning stability, a minimum reward threshold is enforced:
```python
if reward < 0.3:
    reward = 0.3
```

---

## Observation Space (High-Level Overview)

The observation vector captures both internal (proprioceptive) and external (environmental) state information of the humanoid robot, enabling robust decision-making during locomotion.

### Joint Features (for each controlled joint)
- **Position** (rad)
- **Velocity** (rad/s)
- **Torque** (Nm)
- **Symmetrized**: Left and right limb data are mirrored to exploit bilateral symmetry.

### 🌍 Base and Sensor Readings
- **Base Linear Velocity** (in robot’s local frame):  
  `[vx, vy, vz]` – measures forward, lateral, and vertical movement
- **Base Angular Velocity** (in local frame):  
  `[wx, wy, wz]` – roll, pitch, and yaw rates
- **Base Height**:  
  Vertical position of the robot’s torso

- **Gravity Vector** (in robot’s local frame):  
  `[gx, gy, gz]` – orientation of gravity relative to the robot body

### 🦶 Foot Contact and Pressure
- **Center of Pressure (CoP)** for:
  - Left foot: `[x, y]`
  - Right foot: `[x, y]`
- **Ground Reaction Forces**:
  - Total vertical contact force under each foot

This comprehensive observation vector is essential for training policies that can maintain balance, adapt to disturbances, and execute stable walking on varied terrains.


# Installation and Setup

## Clone the Repository:

git clone https://github.com/MohammadKasaei/HumanoidWalking.git
cd HumanoidWalking
```
## Set Up a Virtual Environment (optional):

```
python3 -m venv walking_env
source walking_env/bin/activate  # On Windows use `walking_env\Scripts\activate`
```
## Install Dependencies:
Before running the script, make sure you have execute permissions. Run the following command:
```
chmod +x install_dependencies.sh
```
To install all the dependencies, simply run:
```
./install_dependencies.sh
```
Wait for the script to complete. Once done, all the required dependencies should be installed in your environment.


## Usage 
Instructions on how to run the code, experiments, and reproduce results.
```
python3 Coman_CPG.py
```
Once everything successfully installed, you'll see the simulated COMAN while **walking forward** within the PyBullet simulator.

<div align="center">
  <img src="imgs/coman.gif" alt="Image description" width="640">
</div>

Additionally, by running the following command, you'll see the simulated Talos robot performing **omnidirectional walking**.
```
python3 Talos_CPG.py
```
<div align="center">
  <img src="imgs/talos.gif" alt="Image description" width="640">
</div>



## Citation
If you find our work useful in your research, please consider citing:
```
@article{kasaei2021robust,
  title={Robust biped locomotion using deep reinforcement learning on top of an analytical control approach},
  author={Kasaei, Mohammadreza and Abreu, Miguel and Lau, Nuno and Pereira, Artur and Reis, Luis Paulo},
  journal={Robotics and Autonomous Systems},
  volume={146},
  pages={103900},
  year={2021},
  publisher={Elsevier}
}

@article{kasaei2023learning,
  title={Learning hybrid locomotion skills—Learn to exploit residual actions and modulate model-based gait control},
  author={Kasaei, Mohammadreza and Abreu, Miguel and Lau, Nuno and Pereira, Artur and Reis, Luis Paulo and Li, Zhibin},
  journal={Frontiers in Robotics and AI},
  volume={10},
  pages={1004490},
  year={2023},
  publisher={Frontiers}
}
```