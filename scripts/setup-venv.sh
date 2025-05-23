#!/bin/bash

# setup-venv.sh
# Sets up a Python virtual environment for the ROS2 workspace

setup-venv() {
    # Get the workspace root directory
    local workspace_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
    local venv_dir="$workspace_dir/venv"
    
    echo "Setting up virtual environment in: $venv_dir"
    
    # Create virtual environment if it doesn't exist
    if [[ ! -d "$venv_dir" ]]; then
        echo "Creating new virtual environment..."
        python3 -m venv "$venv_dir"
    fi
    
    # Activate virtual environment
    source "$venv_dir/bin/activate"
    
    # Upgrade pip and install build tools
    echo "Upgrading pip and installing build tools..."
    pip install --upgrade pip wheel setuptools build

    # Install Cython first (needed for numpy)
    echo "Installing Cython..."
    pip install Cython
    
    # Install colcon and dependencies
    echo "Installing colcon and dependencies..."
    pip install -U colcon-common-extensions colcon-core
    
    # Install project requirements
    if [[ -f "$workspace_dir/requirements.txt" ]]; then
        echo "Installing project requirements..."
        pip install --use-pep517 -r "$workspace_dir/requirements.txt"
    fi
    
    # Create COLCON_IGNORE files
    echo "Setting up COLCON_IGNORE files..."
    mkdir -p "$workspace_dir"/{build,install,log}
    touch "$workspace_dir"/{build,install,log}/COLCON_IGNORE
    
    # Set up environment variables
    local python_version=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
    export PYTHONPATH="$venv_dir/lib/python${python_version}/site-packages:$PYTHONPATH"
    
    # Override Python path for colcon
    export PYTHON_PATH="$venv_dir/bin/python3"
    export ROS_PYTHON_VERSION=$python_version
    
    # Create or update the colcon defaults file
    local colcon_defaults="$workspace_dir/.colcon/defaults.yaml"
    mkdir -p "$(dirname "$colcon_defaults")"
    cat > "$colcon_defaults" << EOF
build:
  cmake-args:
    - -DPYTHON_EXECUTABLE=$venv_dir/bin/python3
  symlink-install: true
EOF
    
    # Create a local setup file that can be sourced later
    local setup_local="$workspace_dir/setup.local"
    cat > "$setup_local" << EOF
#!/bin/bash
export PYTHONPATH="$venv_dir/lib/python${python_version}/site-packages:\$PYTHONPATH"
export PYTHON_PATH="$venv_dir/bin/python3"
export ROS_PYTHON_VERSION=$python_version
export VIRTUAL_ENV="$venv_dir"
export PATH="$venv_dir/bin:\$PATH"
EOF
    chmod +x "$setup_local"
    
    echo "Virtual environment setup complete!"
    echo "Python executable: $(which python3)"
    echo "Python version: $python_version"
    echo "Pip executable: $(which pip)"
    echo
    echo "To activate this environment later, run:"
    echo "source $setup_local"
    echo
    echo "To build the workspace, run:"
    echo "colcon build"
}

# Execute the function if the script is being run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    setup-venv
fi 