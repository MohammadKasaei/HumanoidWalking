#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Install numpy and matplotlib
pip install numpy scipy matplotlib 

# Install gym
pip install gym

# Install pybullet
pip install pybullet

echo "All dependencies installed successfully!"
