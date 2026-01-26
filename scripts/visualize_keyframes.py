#!/usr/bin/env python3
"""
Visualize keyframes from FAST-LIVO2 output
- Load keyframe poses and point clouds
- Transform point clouds from body frame to global frame
- Merge all point clouds
- Downsample to 0.5m resolution
- Visualize with z-axis coordinate coloring
"""

import numpy as np
import open3d as o3d
import os
import sys
from pathlib import Path


def load_keyframe_poses(poses_file):
    """
    Load keyframe poses from keyframes.txt
    Format: # id timestamp tx ty tz qx qy qz qw
    Returns: list of dicts with id, timestamp, position, quaternion
    """
    keyframes = []
    
    with open(poses_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) < 9:
                continue
            
            kf = {
                'id': int(parts[0]),
                'timestamp': float(parts[1]),
                'position': np.array([float(parts[2]), float(parts[3]), float(parts[4])]),
                'quaternion': np.array([float(parts[5]), float(parts[6]), float(parts[7]), float(parts[8])])  # [qx, qy, qz, qw]
            }
            keyframes.append(kf)
    
    print(f"Loaded {len(keyframes)} keyframe poses")
    return keyframes


def quaternion_to_rotation_matrix(q):
    """
    Convert quaternion [qx, qy, qz, qw] to rotation matrix
    """
    qx, qy, qz, qw = q
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    return R


def transform_point_cloud(pcd, R, t):
    """
    Transform point cloud: p_global = R * p_body + t
    """
    points = np.asarray(pcd.points)
    points_transformed = (R @ points.T).T + t
    pcd_transformed = o3d.geometry.PointCloud()
    pcd_transformed.points = o3d.utility.Vector3dVector(points_transformed)
    if pcd.has_colors():
        pcd_transformed.colors = pcd.colors
    if pcd.has_normals():
        pcd_transformed.normals = pcd.normals
    return pcd_transformed


def load_and_transform_keyframes(keyframes_dir, keyframes):
    """
    Load PCD files for each keyframe and transform to global frame
    """
    all_points = []
    
    for kf in keyframes:
        pcd_path = os.path.join(keyframes_dir, f"{kf['id']}.pcd")
        
        if not os.path.exists(pcd_path):
            print(f"Warning: PCD file not found: {pcd_path}")
            continue
        
        # Load point cloud
        pcd = o3d.io.read_point_cloud(pcd_path)
        if len(pcd.points) == 0:
            print(f"Warning: Empty point cloud: {pcd_path}")
            continue
        
        # Get transformation
        R = quaternion_to_rotation_matrix(kf['quaternion'])
        t = kf['position']
        
        # Transform to global frame
        pcd_global = transform_point_cloud(pcd, R, t)
        
        # Collect points
        points = np.asarray(pcd_global.points)
        all_points.append(points)
        
        print(f"Keyframe {kf['id']}: {len(points)} points")
    
    # Merge all points
    if len(all_points) == 0:
        print("Error: No valid point clouds found!")
        return None
    
    merged_points = np.vstack(all_points)
    print(f"\nTotal points after merging: {len(merged_points)}")
    
    # Create merged point cloud
    merged_pcd = o3d.geometry.PointCloud()
    merged_pcd.points = o3d.utility.Vector3dVector(merged_points)
    
    return merged_pcd


def downsample_point_cloud(pcd, voxel_size=0.2):
    """
    Downsample point cloud using voxel grid
    """
    print(f"\nDownsampling with voxel size: {voxel_size}m")
    pcd_down = pcd.voxel_down_sample(voxel_size)
    print(f"Points after downsampling: {len(pcd_down.points)}")
    return pcd_down


def color_by_z_coordinate(pcd):
    """
    Color point cloud based on z-axis coordinate
    """
    points = np.asarray(pcd.points)
    z_values = points[:, 2]
    
    # Normalize z values to [0, 1]
    z_min = np.min(z_values)
    z_max = np.max(z_values)
    z_normalized = (z_values - z_min) / (z_max - z_min + 1e-6)
    
    # Create colormap (using matplotlib's viridis colormap)
    # Blue (low z) -> Green -> Yellow -> Red (high z)
    colors = np.zeros((len(z_normalized), 3))
    
    # Custom colormap: blue -> cyan -> green -> yellow -> red
    for i, z_norm in enumerate(z_normalized):
        if z_norm < 0.25:
            # Blue to Cyan
            t = z_norm / 0.25
            colors[i] = [0, t, 1]
        elif z_norm < 0.5:
            # Cyan to Green
            t = (z_norm - 0.25) / 0.25
            colors[i] = [0, 1, 1 - t]
        elif z_norm < 0.75:
            # Green to Yellow
            t = (z_norm - 0.5) / 0.25
            colors[i] = [t, 1, 0]
        else:
            # Yellow to Red
            t = (z_norm - 0.75) / 0.25
            colors[i] = [1, 1 - t, 0]
    
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    print(f"Z range: [{z_min:.2f}, {z_max:.2f}] meters")
    print(f"Color mapping: Blue (low) -> Cyan -> Green -> Yellow -> Red (high)")
    
    return pcd


def create_trajectory_visualization(keyframes, line_color=[1.0, 0.0, 0.0], point_color=[1.0, 1.0, 0.0], 
                                    line_radius=0.1, point_size=0.2):
    """
    Create trajectory visualization from keyframe positions
    Returns: List of geometries (LineSet and PointCloud) for trajectory
    
    Args:
        keyframes: List of keyframe dicts with 'position' field
        line_color: RGB color for trajectory line (default: bright red [1,0,0])
        point_color: RGB color for keyframe points (default: bright yellow [1,1,0])
        line_radius: Radius of trajectory line cylinders (default: 0.1m)
        point_size: Radius of keyframe point spheres (default: 0.2m)
    """
    if len(keyframes) < 2:
        print("Warning: Not enough keyframes to create trajectory")
        return None, None
    
    # Extract positions
    positions = np.array([kf['position'] for kf in keyframes])
    
    # Create line set for trajectory (thin lines as base)
    lines = []
    line_colors = []
    for i in range(len(positions) - 1):
        lines.append([i, i + 1])
        line_colors.append(line_color)
    
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(positions)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(line_colors)
    
    # Create thicker trajectory using cylinders (more visible)
    trajectory_meshes = []
    for i in range(len(positions) - 1):
        p1 = positions[i]
        p2 = positions[i + 1]
        
        # Create cylinder between two points
        direction = p2 - p1
        length = np.linalg.norm(direction)
        
        if length > 1e-6:
            # Create cylinder aligned with z-axis first
            cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=line_radius, height=length, resolution=20)
            
            # Rotate cylinder to align with direction
            z_axis = np.array([0, 0, 1])
            direction_norm = direction / length
            
            # Calculate rotation to align z-axis with direction
            if np.abs(np.dot(z_axis, direction_norm)) < 0.999:  # Not parallel
                rot_axis = np.cross(z_axis, direction_norm)
                rot_axis_norm = rot_axis / (np.linalg.norm(rot_axis) + 1e-6)
                rot_angle = np.arccos(np.clip(np.dot(z_axis, direction_norm), -1, 1))
                R = o3d.geometry.get_rotation_matrix_from_axis_angle(rot_axis_norm * rot_angle)
                cylinder.rotate(R, center=[0, 0, 0])
            elif np.dot(z_axis, direction_norm) < -0.999:
                # Opposite direction, rotate 180 degrees around x or y axis
                cylinder.rotate(o3d.geometry.get_rotation_matrix_from_axis_angle([1, 0, 0] * np.pi), center=[0, 0, 0])
            
            # Translate to midpoint (cylinder center is at origin, extends from -length/2 to +length/2 along z)
            midpoint = (p1 + p2) / 2
            cylinder.translate(midpoint)
            cylinder.paint_uniform_color(line_color)
            trajectory_meshes.append(cylinder)
    
    # Combine all cylinder meshes
    if len(trajectory_meshes) > 0:
        trajectory_line = trajectory_meshes[0]
        for mesh in trajectory_meshes[1:]:
            trajectory_line = trajectory_line + mesh
    else:
        trajectory_line = None
    
    # Create point cloud for keyframe positions (spheres)
    trajectory_points = []
    trajectory_colors = []
    
    for pos in positions:
        # Create a sphere at each keyframe position
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=point_size, resolution=20)
        sphere.translate(pos)
        sphere.paint_uniform_color(point_color)
        
        # Convert sphere to point cloud for better performance
        sphere_pcd = sphere.sample_points_uniformly(number_of_points=100)
        trajectory_points.append(np.asarray(sphere_pcd.points))
        trajectory_colors.append(np.tile(point_color, (len(sphere_pcd.points), 1)))
    
    if len(trajectory_points) > 0:
        all_traj_points = np.vstack(trajectory_points)
        all_traj_colors = np.vstack(trajectory_colors)
        traj_pcd = o3d.geometry.PointCloud()
        traj_pcd.points = o3d.utility.Vector3dVector(all_traj_points)
        traj_pcd.colors = o3d.utility.Vector3dVector(all_traj_colors)
    else:
        traj_pcd = None
    
    print(f"Trajectory: {len(positions)} keyframes, {len(lines)} segments")
    print(f"Trajectory color: Line={line_color}, Points={point_color}")
    
    return trajectory_line, traj_pcd


def main():
    if len(sys.argv) < 2:
        print("Usage: python visualize_keyframes.py <output_dir>")
        print("  output_dir: Directory containing keyframes.txt and keyframes/ folder")
        sys.exit(1)
    
    output_dir = sys.argv[1]
    output_path = Path(output_dir)
    
    if not output_path.exists():
        print(f"Error: Output directory does not exist: {output_dir}")
        sys.exit(1)
    
    # File paths
    poses_file = output_path / "keyframes.txt"
    keyframes_dir = output_path / "keyframes"
    trajectory_file = output_path / "poses.txt"  # Optional: full trajectory
    
    if not poses_file.exists():
        print(f"Error: keyframes.txt not found: {poses_file}")
        sys.exit(1)
    
    if not keyframes_dir.exists():
        print(f"Error: keyframes directory not found: {keyframes_dir}")
        sys.exit(1)
    
    print("=" * 60)
    print("FAST-LIVO2 Keyframe Visualization")
    print("=" * 60)
    
    # Load keyframe poses
    print("\n[1/5] Loading keyframe poses...")
    keyframes = load_keyframe_poses(str(poses_file))
    
    if len(keyframes) == 0:
        print("Error: No keyframes found!")
        sys.exit(1)
    
    # Load and transform keyframes
    print("\n[2/5] Loading and transforming keyframes...")
    merged_pcd = load_and_transform_keyframes(str(keyframes_dir), keyframes)
    
    if merged_pcd is None:
        print("Error: Failed to load keyframes!")
        sys.exit(1)
    
    # Downsample
    print("\n[3/5] Downsampling point cloud...")
    pcd_down = downsample_point_cloud(merged_pcd, voxel_size=0.5)
    
    # Color by z-coordinate
    print("\n[4/6] Coloring by z-coordinate...")
    pcd_colored = color_by_z_coordinate(pcd_down)
    
    # Create trajectory visualization
    print("\n[5/6] Creating trajectory visualization...")
    # Use bright colors: bright red line, bright yellow points
    traj_line, traj_points = create_trajectory_visualization(
        keyframes,
        line_color=[1.0, 0.0, 0.0],      # Bright red
        point_color=[1.0, 1.0, 0.0],     # Bright yellow
        line_radius=0.15,                # Thick line (0.15m radius)
        point_size=0.25                  # Large keyframe markers (0.25m radius)
    )
    
    # Prepare visualization list
    geometries = [pcd_colored]
    
    # Add trajectory (prefer thick cylinders, fallback to LineSet)
    if traj_line is not None:
        geometries.append(traj_line)
    elif len(keyframes) >= 2:
        # Fallback: create simple LineSet if cylinder creation failed
        positions = np.array([kf['position'] for kf in keyframes])
        lines = [[i, i+1] for i in range(len(positions)-1)]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(positions)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector([[1.0, 0.0, 0.0]] * len(lines))
        geometries.append(line_set)
        print("Using LineSet for trajectory (cylinder creation may have failed)")
    
    if traj_points is not None:
        geometries.append(traj_points)
    
    # Visualize
    print("\n[6/6] Visualizing...")
    print("Controls:")
    print("  - Mouse: Rotate view")
    print("  - Shift + Mouse: Pan view")
    print("  - Mouse wheel: Zoom")
    print("  - Q or close window: Exit")
    print("\nVisualization includes:")
    print("  - Point cloud (colored by z-axis)")
    print("  - Trajectory line (bright red)")
    print("  - Keyframe positions (bright yellow spheres)")
    print("\nPress any key in the visualization window to continue...")
    
    o3d.visualization.draw_geometries(geometries,
                                      window_name="FAST-LIVO2 Keyframes + Trajectory",
                                      width=1920,
                                      height=1080)
    
    # Optionally save the result
    save_path = output_path / "keyframes_merged_downsampled.pcd"
    print(f"\nSaving merged and downsampled point cloud to: {save_path}")
    o3d.io.write_point_cloud(str(save_path), pcd_colored)
    print("Done!")


if __name__ == "__main__":
    main()
