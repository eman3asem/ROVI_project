#!/usr/bin/env python3
"""
Trajectory Analysis Tool
Reads trajectory JSON files and provides visualization and analysis
"""

import json
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def load_trajectory(filename):
    """Load trajectory data from JSON file"""
    with open(filename, 'r') as f:
        return json.load(f)

def plot_trajectory(data, save_fig=True):
    """Create comprehensive plots of the trajectory"""
    trajectory = data['trajectory']
    stats = data['statistics']
    metadata = data['metadata']
    
    # Extract data
    times = [p['time'] for p in trajectory]
    positions = np.array([p['position'] for p in trajectory])
    velocities = np.array([p['velocity'] for p in trajectory])
    accelerations = np.array([p['acceleration'] for p in trajectory])
    grippers = [p['gripper'] for p in trajectory]
    
    num_joints = positions.shape[1]
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))
    
    # Plot positions
    ax1 = plt.subplot(4, 1, 1)
    for i in range(num_joints):
        ax1.plot(times, positions[:, i], label=f'Joint {i+1}', linewidth=1.5)
    ax1.set_ylabel('Position (rad)', fontsize=10)
    ax1.set_title(f'Trajectory Analysis - {metadata["timestamp"]}', fontsize=12, fontweight='bold')
    ax1.legend(loc='upper right', ncol=3, fontsize=8)
    ax1.grid(True, alpha=0.3)
    
    # Plot velocities
    ax2 = plt.subplot(4, 1, 2)
    for i in range(num_joints):
        ax2.plot(times, velocities[:, i], label=f'Joint {i+1}', linewidth=1.5)
    ax2.set_ylabel('Velocity (rad/s)', fontsize=10)
    ax2.axhline(y=stats['max_velocity'], color='r', linestyle='--', 
                label=f"Max: {stats['max_velocity']:.2f}", alpha=0.5)
    ax2.legend(loc='upper right', ncol=4, fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    # Plot accelerations
    ax3 = plt.subplot(4, 1, 3)
    for i in range(num_joints):
        ax3.plot(times, accelerations[:, i], label=f'Joint {i+1}', linewidth=1.5)
    ax3.set_ylabel('Acceleration (rad/s²)', fontsize=10)
    ax3.axhline(y=stats['max_acceleration'], color='r', linestyle='--', 
                label=f"Max: {stats['max_acceleration']:.2f}", alpha=0.5)
    ax3.legend(loc='upper right', ncol=4, fontsize=8)
    ax3.grid(True, alpha=0.3)
    
    # Plot gripper state
    ax4 = plt.subplot(4, 1, 4)
    ax4.plot(times, grippers, 'k-', linewidth=2, label='Gripper')
    ax4.set_ylabel('Gripper (0=open, 255=closed)', fontsize=10)
    ax4.set_xlabel('Time (s)', fontsize=10)
    ax4.set_ylim([-10, 265])
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_fig:
        fig_name = os.path.splitext(filename)[0] + '.png'
        plt.savefig(fig_name, dpi=150, bbox_inches='tight')
        print(f"Figure saved as: {fig_name}")
    
    plt.show()

def print_statistics(data):
    """Print trajectory statistics"""
    stats = data['statistics']
    metadata = data['metadata']
    
    print("\n" + "="*60)
    print(f"TRAJECTORY STATISTICS - {metadata['timestamp']}")
    print("="*60)
    print(f"Number of points: {metadata['num_points']}")
    print(f"Time step (dt):   {metadata['dt']} s")
    print(f"Total duration:   {metadata['num_points'] * metadata['dt']:.3f} s")
    print("-"*60)
    print(f"Max velocity:     {stats['max_velocity']:.4f} rad/s")
    print(f"Mean velocity:    {stats['mean_velocity']:.4f} rad/s")
    print(f"Max acceleration: {stats['max_acceleration']:.4f} rad/s²")
    print(f"Mean acceleration:{stats['mean_acceleration']:.4f} rad/s²")
    print("="*60)

def find_problematic_segments(data, vel_threshold=10.0, acc_threshold=100.0):
    """Identify segments with high velocity or acceleration"""
    trajectory = data['trajectory']
    
    print("\n" + "="*60)
    print("PROBLEMATIC SEGMENTS")
    print("="*60)
    
    high_vel = [p for p in trajectory if p['velocity_magnitude'] > vel_threshold]
    high_acc = [p for p in trajectory if p['acceleration_magnitude'] > acc_threshold]
    
    if high_vel:
        print(f"\n⚠ High velocity points (>{vel_threshold} rad/s): {len(high_vel)}")
        for p in high_vel[:5]:  # Show first 5
            print(f"  t={p['time']:.3f}s, v={p['velocity_magnitude']:.2f} rad/s")
        if len(high_vel) > 5:
            print(f"  ... and {len(high_vel)-5} more")
    else:
        print(f"\n✓ No velocity violations (threshold: {vel_threshold} rad/s)")
    
    if high_acc:
        print(f"\n⚠ High acceleration points (>{acc_threshold} rad/s²): {len(high_acc)}")
        for p in high_acc[:5]:  # Show first 5
            print(f"  t={p['time']:.3f}s, a={p['acceleration_magnitude']:.2f} rad/s²")
        if len(high_acc) > 5:
            print(f"  ... and {len(high_acc)-5} more")
    else:
        print(f"\n✓ No acceleration violations (threshold: {acc_threshold} rad/s²)")
    
    print("="*60)

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_trajectory.py <trajectory_file.json>")
        print("\nLooking for trajectory files in current directory...")
        
        # Find all trajectory JSON files
        json_files = [f for f in os.listdir('.') if f.startswith('trajectory_') and f.endswith('.json')]
        
        if not json_files:
            print("No trajectory files found!")
            return
        
        print(f"Found {len(json_files)} file(s):")
        for i, f in enumerate(json_files, 1):
            print(f"  {i}. {f}")
        
        # Use the most recent file
        json_files.sort(reverse=True)
        filename = json_files[0]
        print(f"\nUsing most recent: {filename}")
    else:
        filename = sys.argv[1]
    
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found!")
        return
    
    # Load and analyze trajectory
    print(f"\nLoading: {filename}")
    data = load_trajectory(filename)
    
    # Print statistics
    print_statistics(data)
    
    # Find problematic segments
    find_problematic_segments(data)
    
    # Plot trajectory
    print("\nGenerating plots...")
    plot_trajectory(data, save_fig=True)

if __name__ == "__main__":
    main()

