# rosbag_to_csv

Convert ROS 2 bag files into a single, timestamp‑synchronized CSV via a simple GUI.

## Prerequisites
- This package assumes you have a `~/bags` directory (i.e., `/home/<user>/bags`) containing your rosbag2 recordings. Each bag should be in its own folder that contains `metadata.yaml`.
- Ensure you have the proper message packages installed for all topics in your bags so they can be deserialized.
- For the GUIs:
	- `python3-tk` (Tkinter) installed on your system.
	- `matplotlib` for plotting and PDF export: `pip install matplotlib` (or install via your distro).

## Install and Build
1) Clone this repo into your ROS 2 workspace `src` folder:
	 ```bash
	 cd ~/ros2_ws/src
	 git clone https://github.com/diegohmacias/rosbag_to_csv.git
	 ```
2) Source your ROS 2 distro setup:
	 ```bash
	 source /opt/ros/<DISTRO>/setup.bash
	 ```
3) Build from the workspace root:
	 ```bash
	 cd ~/ros2_ws
	 colcon build --symlink-install
	 ```
4) Source the workspace overlay:
	 ```bash
	 source install/setup.bash
	 ```

Also ensure the `~/bags` directory exists and contains your bag folders.

## Convert rosbags to a merged CSV (GUI)
Launch the converter GUI:
```bash
ros2 launch rosbag_to_csv rosbag_to_csv_gui.launch.py
```

In the GUI:
- Choose the bag directory (a folder containing `metadata.yaml`).
- Choose an output directory (defaults to `~/csvs`).
- Pick a timestamp mode (`ns` or `sec`). The output CSV will contain a single timestamp column.
- Load and select the topics you want included.
- Click “Export Selected → CSV”.

Output specifics:
- One single CSV is produced, named after the bag directory (e.g., `my_bag.csv`).
- The CSV is timestamp‑synchronized across selected topics using union‑of‑timestamps with forward‑fill.
- Only one timestamp column is included: `timestamp_ns` or `timestamp_sec`.
- Topic fields are namespaced to avoid collisions, e.g., `/odom:pose.pose.position.x`.

## Plot CSV signals (GUI)
Launch the plot GUI:
```bash
ros2 launch rosbag_to_csv rosbag_to_csv_plot.launch.py
```

In the GUI:
- Pick the CSV directory (defaults to `~/csvs`).
- Select a CSV from the list.
- In “Available signals”, check the signals you want to plot (you can select any columns except the timestamp).
- If your CSV includes latitude/longitude, optionally enable the Derived checkboxes for `pos_x` / `pos_y` (meters).
- Optional: to put specific signals on the same subplot, enter their exact names in the “Overlay signals (comma-separated exact names)” field. Example: `pos_x, pos_y` will have the two signals on the same plot
- Click “Plot”.

What happens:
- The plot is shown and automatically saved next to the CSV as a `.pdf` (same base filename).
- Numeric signals are drawn as lines. Boolean-like signals (e.g., `true/false`, `on/off`, `1/0`) are drawn as stepped traces (0/1), with offsets when overlaid for readability.

## Troubleshooting
- Package not found when launching:
	- Ensure you ran `colcon build --symlink-install` and `source install/setup.bash` in the same shell before launching.
- GUI doesn’t open on Linux:
	- Install Tkinter (`sudo apt install python3-tk`).
- Plot GUI errors about matplotlib:
	- Install matplotlib: `pip install matplotlib` (or via your distro package manager).
- Bag can’t be read:
	- Confirm the bag directory contains `metadata.yaml` and that all required message packages are installed.


