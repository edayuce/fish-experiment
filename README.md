# Fish Experiment GUI

This project is a ROS-based application with a Python GTK interface to control a fish experiment setup.

## Prerequisites

Before you begin, ensure you have the following installed:

*   **ROS (Robot Operating System):** This project was developed with ROS. Please follow the official installation instructions for your Linux distribution.
*   **Python 3 and dependencies:**
    ```bash
    pip install pyserial rospkg PySimpleGUI screeninfo
    ```
*   **GTK and GObject Introspection:**
    ```bash
    sudo apt-get update
    sudo apt-get install python3-gi gir1.2-gtk-3.0
    ```
    ```
*   **SMbus:** For I2C communication.
    ```bash
    sudo apt-get install python3-smbus
    ```

## Building the Project

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/edayuce/fish-experiment.git
    ```

2.  **Navigate to the workspace directory:**
    ```bash
    cd fish-experiment
    ```

3.  **Source your ROS environment:**
    (Replace `noetic` with your installed ROS distribution, e.g., `melodic`, `kinetic`)
    ```bash
    source /opt/ros/noetic/setup.bash
    ```

4.  **Build the workspace:**
    This command will create the `build` and `devel` folders.
    ```bash
    catkin_make
    ```

## Running the Project

1.  **Source the local setup file:**
    From the root of the workspace (`fish-experiment`), run:
    ```bash
    source devel/setup.bash
    ```
2. **Start master:**
   Open a terminal, run: 
   ```bash
   roscore
   ```

3.  **Run the GUI:**
    Click + top-left corner, open second termial and run:
    ```bash
    python3 src/rectrial/scripts/my_gui.py
    ```

This will launch the main control window for the experiment.
