#!/bin/bash

# Ensure the script exits if any command fails
set -e

# Setup conda initialization so we can use 'conda activate' inside the script
eval "$(conda shell.bash hook)"

echo "=========================================="
echo "Creating conda environment: teleop"
echo "=========================================="
conda create -n teleop python=3.10 -y

echo "=========================================="
echo "Activating conda environment: teleop"
echo "=========================================="
conda activate teleop

echo "=========================================="
echo "Installing pip (if not already present)..."
echo "=========================================="
conda install pip -y

echo "=========================================="
echo "Installing project and dependencies..."
echo "=========================================="
# Using pip to read venv/teleop/pyproject.toml
# This installs all dependencies (including git repos) and the project in editable mode
pip install -e ./venv/teleop

echo "=========================================="
echo "Installation completed successfully!"
echo "Please run: conda activate teleop"
echo "=========================================="
