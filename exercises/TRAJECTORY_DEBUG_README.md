# Trajectory Debugging Tools

This document explains how to use the trajectory debugging features added to the project.

## Overview

The trajectory system now automatically saves detailed trajectory data to JSON files for debugging and analysis.

## Features

### 1. Automatic Trajectory Saving

When you run the robot program, it automatically saves the trajectory to a JSON file with:
- **Joint positions** at each waypoint
- **Velocities** (calculated using finite differences)
- **Accelerations** (calculated using second-order finite differences)
- **Gripper states** (0=open, 255=closed)
- **Statistics** (max/mean velocity and acceleration)
- **Timestamp** for easy identification

**Filename format:** `trajectory_<planner>_<timestamp>.json`
- Example: `trajectory_rrt_20251215_143052.json`

### 2. Trajectory Analysis Script

Use `analyze_trajectory.py` to visualize and analyze saved trajectories.

#### Usage

```bash
# Analyze a specific trajectory file
python analyze_trajectory.py trajectory_rrt_20251215_143052.json

# Or just run it without arguments to analyze the most recent trajectory
python analyze_trajectory.py
```

#### What it shows

1. **Statistics**: Number of points, duration, max/mean velocities and accelerations
2. **Plots**: 4-panel visualization showing:
   - Joint positions over time
   - Joint velocities over time  
   - Joint accelerations over time
   - Gripper state over time
3. **Problematic segments**: Identifies points with unusually high velocity or acceleration
4. **PNG export**: Automatically saves plots as PNG files

## JSON File Structure

```json
{
  "metadata": {
    "timestamp": "2025-12-15 14:30:52",
    "num_points": 1500,
    "dt": 0.001,
    "description": "Robot trajectory with joint positions, velocities, and accelerations"
  },
  "trajectory": [
    {
      "index": 0,
      "time": 0.0,
      "position": [q1, q2, q3, q4, q5, q6],
      "velocity": [dq1, dq2, dq3, dq4, dq5, dq6],
      "acceleration": [ddq1, ddq2, ddq3, ddq4, ddq5, ddq6],
      "gripper": 0,
      "velocity_magnitude": 0.0,
      "acceleration_magnitude": 0.0
    },
    ...
  ],
  "statistics": {
    "max_velocity": 5.234,
    "mean_velocity": 1.456,
    "max_acceleration": 45.67,
    "mean_acceleration": 12.34
  }
}
```

## Debugging Tips

### 1. Check for Velocity Spikes
- Look for sudden jumps in the velocity plots
- High velocity magnitudes may indicate discontinuous motion

### 2. Check for Acceleration Spikes
- Large accelerations can cause jerky motion or violate robot limits
- Typical robot acceleration limits: 50-200 rad/s²

### 3. Gripper Timing
- Verify gripper opens/closes at the right times
- Check if gripper state changes during motion (should be stable)

### 4. Joint Limits
- Verify all joint positions stay within [-3.2, 3.2] rad
- Check if any joints hit their limits

### 5. Compare Planners
- Save trajectories from both "points" and "rrt" planners
- Compare smoothness, duration, and peak values

## Example Workflow

```bash
# 1. Run your robot program
mjpython main.py
# Choose "rrt" or "points"
# Trajectory is automatically saved

# 2. Analyze the trajectory
python analyze_trajectory.py
# Review statistics and plots

# 3. If there are issues, adjust parameters in Project.py:
#    - Approach offsets (e.g., Tz(-0.15))
#    - Safe height (Tz(-0.30))
#    - Movement times (EXE_TIME)
#    - RRT planning timeout

# 4. Run again and compare trajectories
```

## Troubleshooting

### Issue: "No solution found" during RRT planning
- **Check**: Is the target pose reachable?
- **Check**: Trajectory JSON to see where it failed
- **Solution**: Adjust frame orientations or approach offsets

### Issue: Robot moves too fast
- **Check**: Max velocity in statistics
- **Solution**: Increase movement times or reduce step sizes

### Issue: Jerky motion
- **Check**: Acceleration plot for spikes
- **Solution**: Use smoother interpolation (increase steps in parabolic interpolation)

### Issue: Collision with walls
- **Check**: Position trajectory and gripper state
- **Solution**: Increase safe height or approach offsets

## Files Created

- `trajectory_<planner>_<timestamp>.json` - Raw trajectory data
- `trajectory_<planner>_<timestamp>.png` - Visualization plots
- `analyze_trajectory.py` - Analysis script
- `Project.py` - Contains `save_trajectory_to_json()` function

---

**Happy Debugging! 🤖**

