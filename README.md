# TA-ROVI-ROB: Robotics Vision and Planning in MuJoCo

A minimal MuJoCo-based simulation environment for the ROVI (Robotics Vision) course. This project provides a framework for students to implement computer vision methods and path planning algorithms taught in the lectures, using a simulated UR5e robot with a Hand-E gripper.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

![Demo Video](/media/clip_rrtconnect-ezgif.com-video-to-gif-converter.gif)



## 🏗️ Project Structure
### Key Files Explained:
*   **`exercises/`**: Students will create files (e.g., `exercise_1.py`) in this directory for their assignments. Teaching Assistants can place their corresponding solution files here.
*   **`main.py`**: The core simulation file. To run a specific exercise, change the import (e.g., `from exercises.exercise_1 import program`).
*   **`robot.py`**: Contains the `Robot` class with methods like `move_l()`,`move_j()`, `ur_ctrl_qpos()`, `ur_set_qpos()`, and `get_current_q()`.
*   **`cam.py`**: Contains the `Camera` class with methods to `get_rgb()`, `get_depth()`, and `get_pointcloud()`.



## ⚙️ Prerequisites

Before you begin, ensure you have the following installed on your system:
*   **Python 3.10** (This project specifically requires version 3.10)
*   `pip` (usually comes with Python)
*   `venv` module (usually included in standard Python 3.10+ library)
*   **OMPL (Open Motion Planning Library)**: This is a **system-level library**, not a Python package. It must be installed on your machine. See installation [here](https://ompl.kavrakilab.org/installation.html).
    <!-- *   **Ubuntu/Debian:** `sudo apt install libompl-dev`
    *   **macOS (with Homebrew):** `brew install ompl`
    *   **Windows:** Building from source is recommended. -->
*   **OMPL (alternative):** 
[OMPL](https://ompl.kavrakilab.org/) is the library we use for planning and can be installed using the precompiled binary wheels from [here](https://github.com/ompl/ompl/releases/tag/prerelease). Download the wheel corresponding to your platform and install it using
```bash
# Note: Remember to replace the filename with your downloaded filename
pip install ompl-1.6.0-cp312-cp312-manylinux_2_28_x86_64.whl 
```


## Getting Started

Follow these steps to set up the development environment and install dependencies.

1.  **Clone the repository:**
    ```bash
    git clone https://gitlab.sdu.dk/wilb/rovi2025.git
    cd rovi2025
    ```

2.  **Create a virtual environment:** (Recommended to isolate dependencies)
    ```bash
    python3.10 -m venv venv
    ```

3.  **Activate the virtual environment:**
    *   On Linux/macOS:
        ```bash
        source venv/bin/activate
        ```
    *   On Windows (Command Prompt):
        ```bat
        venv\Scripts\activate.bat
        ```
        Your command prompt should now show `(venv)` at the beginning.

4.  **Install the required Python packages:**
    ```bash
    python3.10 -m pip install -r requirements.txt
    ```

You are now ready to run the project!

## 🎯 Usage

### Running an Exercise
The main simulation is run by executing `main.py`. By default, it may be configured to run a specific exercise or a demo.

1.  **To run a specific exercise (e.g., Exercise 1):**
    *   Open `main.py` in a text editor.
    *   Find the import statement near the top and change it to import your desired program.
    *   **Example:**
        ```python
        # Change this line to the exercise file you want to run, e.g. exercise_2, ..., exercise_8 :
        from exercises.exercise_1 import program
        ```
    *   Run the simulation:
        ```bash
        (venv) $ mjpython main.py
        ```


### Developing Your Solution
1.  Create a new file in the `exercises/` folder (e.g., `exercise_2.py`).
2.  Implement your `program(model, data, robot, cam)` function in that file. This is the entry point that `main.py` will call.
3.  Use the helper functions from `robot.py` and `cam.py` to control the robot and process vision data.
4.  Update the import in `main.py` to test your implementation.

<!-- ## 📚 API Overview

### Robot Class (`robot.py`)
Key methods for controlling the robot:
*   `move_to(target_pose, ...)`: Plans and executes a motion to a target Cartesian pose.
*   `set_joint_positions(q)`: Commands the robot to a specific joint configuration.
*   `get_current_pose()`: Returns the current end-effector pose as a 4x4 transformation matrix.
*   `open_gripper()`, `close_gripper()`: Controls the gripper.

### Camera Class (`cam.py`)
Key methods for perception:
*   `get_rgb_image()`: Returns an RGB image of the current scene.
*   `get_depth_image()`: Returns a depth image.
*   `get_pointcloud()`: Processes the depth image to return a 3D point cloud. -->

## 🆘 Getting Help

If you encounter issues, please check the following:
1.  Ensure all **Prerequisites** are met, especially the system-wide installation of **OMPL**.
2.  Ensure your virtual environment is activated and all packages from `requirements.txt` are installed correctly.
3.  For course-specific questions, please refer to the course material or contact the teaching staff.
