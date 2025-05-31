# HumanoidWalking - CPG and DRL
<div align="center">
  <img src="imgs/DRL.gif" alt="Image description" width="640">
</div>

<!-- In this repository, you will find one of the key components of my [PhD research](https://ria.ua.pt/bitstream/10773/33545/1/Documento_Seyed_Kasaei.pdf): a straightforward and effective method for creating walking gaits in humanoid robots. Our strategy blends the Linear Inverted Pendulum Model (LIPM) with Central Pattern Generators (CPG) to craft a unique walk engine. This engine utilizes a state machine to oversee the robot's walking cycle, generating smooth walking paths through a combination of Partial Fourier Series (PFS) oscillators and stabilizing controllers. This repository offers a glimpse into a larger body of work dedicated to advancing the field of humanoid robotics, showcasing a practical and efficient approach to robot locomotion as part of my doctoral journey. I have deployed the walking engine on different platforms including COMAN, Talos and Nao. -->

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



# Installation and Setup

## Clone the Repository:

```
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

![Alt](imgs/coman.gif)

Additionally, by running the following command, you'll see the simulated Talos robot performing **omnidirectional walking**.
```
python3 Talos_CPG.py
```
![Alt](imgs/talos.gif)



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