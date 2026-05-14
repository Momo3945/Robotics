````markdown
# SurveillanceBot Robotics Assignment

This folder contains our Honours SurveillanceBot implementation. The robot maps the environment, uses a processed occupancy grid for A* path planning, and navigates to user-entered `(x, y)` coordinates using a proportional waypoint controller.

---

## 1. Where to place this folder

Place the `surveillance_bot` folder inside the `src` directory of the provided robotics workspace:

```text
robot_assignment_ws/
├── startWorld
├── src/
│   └── surveillance_bot/
│       ├── scripts/
│       ├── maps/
│       ├── docs/
│       └── utils/
````

So the final path should be:

```text
robot_assignment_ws/src/surveillance_bot/
```

---

## 2. Compile the workspace

From inside the Docker/Singularity/ROS environment, go to the workspace root:

```bash
cd robot_assignment_ws
catkin_make
```

Then source the workspace:

```bash
source devel/setup.bash
```

---

## 3. Start the simulator

Open a terminal and run:

```bash
cd robot_assignment_ws
source devel/setup.bash
./startWorld
```

Leave this terminal running. This starts Gazebo and the assignment world.

---

## 4. Run the navigation node

Open a second terminal:

```bash
cd robot_assignment_ws
source devel/setup.bash
cd src/surveillance_bot/scripts
python navigate_to_goal.py
```

The program will ask for a goal coordinate:

```text
Enter goal as: x y   or q to quit
>
```

Enter a coordinate, for example:

```text
0 0
```

or:

```text
4 0.5
```

The robot will:

1. Get its current pose from Gazebo.
2. Convert the current pose and goal into map coordinates.
3. Run A* on the processed occupancy map.
4. Save a debug image of the planned path.
5. Follow the path using velocity commands.

To quit:

```text
q
```

---

## 5. Important files

### Main scripts

```text
scripts/astar_planner.py       A* path planner
scripts/controller.py          Waypoint-following controller
scripts/navigate_to_goal.py    Main navigation node
```

### Maps

```text
maps/raw/surveillance_map_091423.pgm
maps/raw/surveillance_map_091423.yaml
```

These are the raw maps produced using `gmapping`.

```text
maps/processed/surveillance_map_preprocessed.pgm
maps/processed/surveillance_map_preprocessed.yaml
```

This is the automatically preprocessed version.

```text
maps/processed/surveillance_map_planning.pgm
maps/processed/surveillance_map_planning.yaml
```

This is the final cleaned planning map used by A*.

### Report/demo images

```text
docs/screenshots/
```

Contains full A* debug path images.

```text
docs/report_figures/
```

Contains cropped images used in the report.

---

## 6. Testing A* only

To test the planner without moving the robot, run from the `scripts` folder:

```bash
python astar_planner.py -1.1 1.9 4.2 1.8 --debug-out ../docs/screenshots/test_path.png
```

This should print the planned waypoints and save a debug image showing the A* path.

Another useful test:

```bash
python astar_planner.py 4.2 1.8 -1.1 1.9 --debug-out ../docs/screenshots/test_path_reverse.png
```

---

## 7. Regenerating the preprocessed map

The preprocessing script is stored with the raw map:

```text
maps/raw/preprocess_map.py
```

Run it from the raw map folder:

```bash
cd robot_assignment_ws/src/surveillance_bot/maps/raw
python preprocess_map.py
```

It reads:

```text
surveillance_map_091423.yaml
surveillance_map_091423.pgm
```

and writes:

```text
../processed/surveillance_map_preprocessed.pgm
../processed/surveillance_map_preprocessed.yaml
```

The final manually cleaned map is:

```text
maps/processed/surveillance_map_planning.pgm
```

---

## 8. Cropping report figures

To crop maps and A* screenshots for the report, run:

```bash
cd robot_assignment_ws/src/surveillance_bot
python utils/crop_maps.py
```

The cropped output is saved in:

```text
docs/report_figures/
```

---

## 9. Some demo coordinates

Some coordinates that I used during testing:

```text
0 0
4 0.5
-8 0.5
-1 2
3 4
4.2 1.8
```
---


