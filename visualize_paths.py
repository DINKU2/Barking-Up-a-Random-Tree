#!/usr/bin/env python3
"""
Visualization script for Project 3 - Exercise 2 (Box Robot Planning)
Creates images of environments and RTP-generated paths
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os

def read_obstacles_from_code():
    """Extract obstacle definitions from the C++ code"""
    obstacles = []
    
    # Environment 2 obstacles (from makeEnvironment2)
    obstacles.append({
        'name': 'Environment 2 - Complex obstacles for box robot',
        'rectangles': [
            (-0.5, -0.5, 1.0, 1.0),   # Center obstacle
            (-1.0, 1.5, 0.8, 0.8),    # Top left
            (1.5, -1.0, 0.8, 0.8),    # Bottom right
            (-1.5, -1.5, 0.6, 0.8),   # Bottom left
            (1.0, 1.0, 0.6, 0.8),     # Top right
            (-2.0, 2.5, 0.6, 0.6),    # Far top left
            (0.0, 2.0, 0.6, 0.6),     # Top center
            (-2.5, -0.5, 0.6, 0.6),   # Left side
            (2.0, -2.0, 0.6, 0.6)     # Far bottom right
        ],
        'start': (-3.5, -3.5, 0.0),  # x, y, theta - bottom left
        'goal': (3.5, 3.5, 0.0),     # x, y, theta - top right
        'robot_size': 0.5  # Larger robot for better visualization
    })
    
    return obstacles

def read_path_file(filename):
    """Read path from the generated text file"""
    if not os.path.exists(filename):
        return None
    
    path = []
    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 3:
                x, y, theta = float(parts[0]), float(parts[1]), float(parts[2])
                path.append((x, y, theta))
    return path

def draw_robot(ax, x, y, theta, size, color='blue', alpha=0.7):
    """Draw a square robot at position (x,y) with rotation theta"""
    # Create square robot
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

def visualize_environment(env, path=None, save_name=None):
    """Visualize environment with obstacles, start, goal, and optional path"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    
    # Set up the plot
    ax.set_xlim(-7, 7)
    ax.set_ylim(-7, 7)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title(f'{env["name"]}\nStart: {env["start"][:2]}, Goal: {env["goal"][:2]}')
    
    # Draw obstacles
    for rect in env['rectangles']:
        x, y, w, h = rect
        obstacle = patches.Rectangle(
            (x, y), w, h,
            facecolor='red', alpha=0.7, edgecolor='black', linewidth=2
        )
        ax.add_patch(obstacle)
    
    # Draw start position
    draw_robot(ax, env['start'][0], env['start'][1], env['start'][2], 
               env['robot_size'], color='green', alpha=0.8)
    ax.text(env['start'][0], env['start'][1] - 0.3, 'START', 
            ha='center', va='top', fontweight='bold', color='green')
    
    # Draw goal position
    draw_robot(ax, env['goal'][0], env['goal'][1], env['goal'][2], 
               env['robot_size'], color='orange', alpha=0.8)
    ax.text(env['goal'][0], env['goal'][1] + 0.3, 'GOAL', 
            ha='center', va='bottom', fontweight='bold', color='orange')
    
    # Draw path if provided
    if path:
        # Draw path line
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        ax.plot(path_x, path_y, 'b-', linewidth=2, alpha=0.7, label='RTP Path')
        
        # Draw robot at key waypoints
        for i, (x, y, theta) in enumerate(path[::max(1, len(path)//10)]):  # Show ~10 waypoints
            draw_robot(ax, x, y, theta, env['robot_size'], 
                      color='blue', alpha=0.5)
        
        # Mark waypoints
        ax.scatter(path_x, path_y, c='blue', s=20, alpha=0.6, zorder=5)
        
        ax.legend()
    
    # Add environment info
    info_text = f'Robot Size: {env["robot_size"]}\n'
    if path:
        info_text += f'Path Length: {len(path)} waypoints\n'
        info_text += f'Path Distance: {calculate_path_length(path):.2f} units'
    
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    
    if save_name:
        plt.savefig(save_name, dpi=300, bbox_inches='tight')
        print(f"Saved visualization: {save_name}")
    
    return fig, ax

def calculate_path_length(path):
    """Calculate total path length"""
    if len(path) < 2:
        return 0
    
    total_length = 0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        total_length += np.sqrt(dx*dx + dy*dy)
    
    return total_length

def create_environment_overview():
    """Create overview of all environments"""
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    
    # Environment 1 (Point Robot - No obstacles)
    ax1 = axes[0]
    ax1.set_xlim(-7, 7)
    ax1.set_ylim(-7, 7)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Exercise 1: Point Robot\nEnvironment 1 - No Obstacles')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    
    # Start and goal for point robot
    ax1.scatter(-5, -5, c='green', s=100, marker='o', label='Start', zorder=5)
    ax1.scatter(5, 5, c='orange', s=100, marker='s', label='Goal', zorder=5)
    ax1.text(-5, -5.5, 'START', ha='center', fontweight='bold', color='green')
    ax1.text(5, 5.5, 'GOAL', ha='center', fontweight='bold', color='orange')
    ax1.legend()
    
    # Environment 2 (Box Robot - With obstacles)
    ax2 = axes[1]
    ax2.set_xlim(-7, 7)
    ax2.set_ylim(-7, 7)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Exercise 2: Box Robot\nEnvironment 2 - With Obstacles')
    ax2.set_xlabel('X Position')
    ax2.set_ylabel('Y Position')
    
    # Draw obstacle
    obstacle = patches.Rectangle(
        (-0.5, -0.5), 1.0, 1.0,
        facecolor='red', alpha=0.7, edgecolor='black', linewidth=2
    )
    ax2.add_patch(obstacle)
    
    # Draw multiple obstacles for box robot
    obstacle_rects = [
        (-0.5, -0.5, 1.0, 1.0), (-1.0, 1.5, 0.8, 0.8), (1.5, -1.0, 0.8, 0.8),
        (-1.5, -1.5, 0.6, 0.8), (1.0, 1.0, 0.6, 0.8), (-2.0, 2.5, 0.6, 0.6),
        (0.0, 2.0, 0.6, 0.6), (-2.5, -0.5, 0.6, 0.6), (2.0, -2.0, 0.6, 0.6)
    ]
    
    for rect in obstacle_rects:
        x, y, w, h = rect
        obstacle = patches.Rectangle(
            (x, y), w, h,
            facecolor='red', alpha=0.7, edgecolor='black', linewidth=1
        )
        ax2.add_patch(obstacle)
    
    # Start and goal for box robot
    draw_robot(ax2, -3.5, -3.5, 0.0, 0.5, color='green', alpha=0.8)
    draw_robot(ax2, 3.5, 3.5, 0.0, 0.5, color='orange', alpha=0.8)
    ax2.text(-3.5, -3.8, 'START', ha='center', fontweight='bold', color='green')
    ax2.text(3.5, 3.8, 'GOAL', ha='center', fontweight='bold', color='orange')
    
    plt.tight_layout()
    plt.savefig('environment_overview.png', dpi=300, bbox_inches='tight')
    print("Saved environment overview: environment_overview.png")

def main():
    """Main function to generate all visualizations"""
    print("Generating visualizations for Project 3 - Exercise 2...")
    
    # Create environment overview
    create_environment_overview()
    
    # Get environments
    environments = read_obstacles_from_code()
    
    # Visualize each environment
    for i, env in enumerate(environments):
        # Environment without path
        fig, ax = visualize_environment(env, save_name=f'environment_{i+1}_no_path.png')
        plt.close(fig)
        
        # Environment with path (if available)
        path_file = 'box_robot_path.txt'
        path = read_path_file(path_file)
        
        if path:
            fig, ax = visualize_environment(env, path=path, save_name=f'environment_{i+1}_with_path.png')
            plt.close(fig)
        else:
            print(f"Warning: Could not find path file {path_file}")
    
    print("\nVisualization complete!")
    print("Generated files:")
    print("- environment_overview.png (overview of both exercises)")
    print("- environment_1_no_path.png (Exercise 2 environment without path)")
    print("- environment_1_with_path.png (Exercise 2 environment with RTP path)")

if __name__ == "__main__":
    main()
