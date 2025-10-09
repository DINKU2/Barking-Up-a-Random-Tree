#!/usr/bin/env python3
"""
Create a comparison visualization showing robot size impact
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def draw_robot(ax, x, y, theta, size, color='blue', alpha=0.7):
    """Draw a square robot at position (x,y) with rotation theta"""
    robot = patches.Rectangle(
        (x - size/2, y - size/2), size, size,
        angle=np.degrees(theta), rotation_point='center',
        facecolor=color, alpha=alpha, edgecolor='black', linewidth=1
    )
    ax.add_patch(robot)
    
    # Draw direction indicator
    dx = size * 0.4 * np.cos(theta)
    dy = size * 0.4 * np.sin(theta)
    ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.05, fc='red', ec='red')

def create_robot_size_comparison():
    """Create comparison of different robot sizes"""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    # Common settings
    for ax in axes:
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
    
    # Obstacle (same for all)
    obstacle = patches.Rectangle(
        (-0.5, -0.5), 1.0, 1.0,
        facecolor='red', alpha=0.7, edgecolor='black', linewidth=2
    )
    
    # Plot 1: Tiny robot (0.1)
    ax1 = axes[0]
    ax1.add_patch(patches.Rectangle((-0.5, -0.5), 1.0, 1.0, facecolor='red', alpha=0.7, edgecolor='black', linewidth=2))
    draw_robot(ax1, -2.0, -2.0, 0.0, 0.1, color='green', alpha=0.8)
    draw_robot(ax1, 2.0, 2.0, 0.0, 0.1, color='orange', alpha=0.8)
    ax1.set_title('Robot Size: 0.1 units\n(Hard to see rotation)')
    ax1.text(-2.0, -2.3, 'START', ha='center', fontweight='bold', color='green')
    ax1.text(2.0, 2.3, 'GOAL', ha='center', fontweight='bold', color='orange')
    
    # Plot 2: Medium robot (0.5)
    ax2 = axes[1]
    ax2.add_patch(patches.Rectangle((-0.5, -0.5), 1.0, 1.0, facecolor='red', alpha=0.7, edgecolor='black', linewidth=2))
    draw_robot(ax2, -2.0, -2.0, 0.0, 0.5, color='green', alpha=0.8)
    draw_robot(ax2, 2.0, 2.0, 0.0, 0.5, color='orange', alpha=0.8)
    ax2.set_title('Robot Size: 0.5 units\n(Good for visualization)')
    ax2.text(-2.0, -2.3, 'START', ha='center', fontweight='bold', color='green')
    ax2.text(2.0, 2.3, 'GOAL', ha='center', fontweight='bold', color='orange')
    
    # Plot 3: Large robot (1.0)
    ax3 = axes[2]
    ax3.add_patch(patches.Rectangle((-0.5, -0.5), 1.0, 1.0, facecolor='red', alpha=0.7, edgecolor='black', linewidth=2))
    draw_robot(ax3, -2.0, -2.0, 0.0, 1.0, color='green', alpha=0.8)
    draw_robot(ax3, 2.0, 2.0, 0.0, 1.0, color='orange', alpha=0.8)
    ax3.set_title('Robot Size: 1.0 units\n(Too large for this obstacle)')
    ax3.text(-2.0, -2.3, 'START', ha='center', fontweight='bold', color='green')
    ax3.text(2.0, 2.3, 'GOAL', ha='center', fontweight='bold', color='orange')
    
    plt.suptitle('Robot Size Comparison for Box Robot Planning', fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.savefig('robot_size_comparison.png', dpi=300, bbox_inches='tight')
    print("Saved robot size comparison: robot_size_comparison.png")

if __name__ == "__main__":
    create_robot_size_comparison()
