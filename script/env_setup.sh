#!/bin/bash

# cargo3d_env_setup.sh - Create Conda environment for 3D cargo point cloud processing


# Environment configuration
ENV_NAME="cargo3d"
PYTHON_VERSION="3.10"

# Check if environment already exists
if conda env list | grep -q "$ENV_NAME"; then
    echo "Environment '$ENV_NAME' already exists."
    read -p "Recreate environment? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing existing environment..."
        conda env remove -n "$ENV_NAME"
    else
        echo "Using existing environment."
        exit 0
    fi
fi

# Create new environment with Python 3.10
echo "Creating Conda environment: $ENV_NAME (Python $PYTHON_VERSION)"
conda create -n "$ENV_NAME" python="$PYTHON_VERSION" -y

# Activate environment
conda activate "$ENV_NAME"

# Install core scientific packages
echo "Installing core dependencies..."
conda install -y \
    numpy \
    open3d \
    scipy \
    matplotlib \
    pandas 


# conda install -y \
    # scikit-learn \
    # seaborn \
    # tqdm

# Install development tools
# echo "Installing development tools..."
# conda install  -y \
#     jupyterlab \
#     jupyterlab-g

# Create environment info file
echo "Creating environment info file..."
conda env export > cargo3d_open3d_environment.yml