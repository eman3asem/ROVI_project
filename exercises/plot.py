import matplotlib.pyplot as plt
import json
import numpy as np

def plot_comparison(p2p_file, rrt_file):
    with open(p2p_file) as f: p2p = json.load(f)
    with open(rrt_file) as f: rrt = json.load(f)

    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    fig.suptitle('Joint Trajectories (Q) Comparison: P2P vs RRT')

    for joint in range(6):
        ax = axes[joint//2, joint%2]
        p2p_q = [node['q'][joint] for node in p2p]
        rrt_q = [node['q'][joint] for node in rrt]
        
        ax.plot(p2p_q, label='P2P (Trapezoidal)', color='blue')
        ax.plot(rrt_q, label='RRT', color='orange', linestyle='--')
        ax.set_title(f'Joint {joint+1}')
        ax.legend()

    plt.tight_layout()
    plt.savefig('graphs/trajectory_comparison.png', dpi=300, bbox_inches='tight')
    print("Figure saved as 'trajectory_comparison.png'")
    plt.show()

def main():
    p2p_file = "p2p_results.json"
    rrt_file = "rrt_results.json"
    plot_comparison(p2p_file, rrt_file)

if __name__ == "__main__":
    main()