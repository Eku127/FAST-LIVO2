#!/usr/bin/env python3
"""
Visualize keyframes from FAST-LIVO2 output
- Load keyframe poses and point clouds
- Transform point clouds from body frame to global frame
- Merge all point clouds
- Downsample to 0.5m resolution
- Visualize with z-axis coordinate coloring
- Generate roaming video along trajectory (optional)

Dependencies:
  pip install scipy imageio imageio-ffmpeg
"""

import numpy as np
import open3d as o3d
import open3d.visualization.rendering as rendering
import os
import sys
import argparse
from pathlib import Path

# Optional imports for video generation
try:
    from scipy.interpolate import CubicSpline
    import imageio
    HAS_VIDEO_DEPS = True
except ImportError:
    HAS_VIDEO_DEPS = False


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


# =============================================================================
# Roaming Video Functions
# =============================================================================

def interpolate_trajectory(keyframes, total_frames):
    """
    Interpolate smooth camera path between keyframes using cubic spline.
    
    Args:
        keyframes: List of keyframe dicts with 'position' field
        total_frames: Total number of frames to generate
        
    Returns:
        numpy array of shape (total_frames, 3) with interpolated positions
    """
    if not HAS_VIDEO_DEPS:
        raise ImportError("scipy is required for trajectory interpolation. "
                          "Install with: pip install scipy")
    
    positions = np.array([kf['position'] for kf in keyframes])
    n_keyframes = len(positions)
    
    if n_keyframes < 2:
        raise ValueError("Need at least 2 keyframes for interpolation")
    
    # Parameter t goes from 0 to n_keyframes-1
    t = np.arange(n_keyframes)
    t_interp = np.linspace(0, n_keyframes - 1, total_frames)
    
    # Create cubic spline for each coordinate
    cs_x = CubicSpline(t, positions[:, 0])
    cs_y = CubicSpline(t, positions[:, 1])
    cs_z = CubicSpline(t, positions[:, 2])
    
    # Evaluate at interpolated points
    interp_positions = np.column_stack([
        cs_x(t_interp),
        cs_y(t_interp),
        cs_z(t_interp)
    ])
    
    return interp_positions


def ease_in_out(t):
    """
    Smooth ease-in-out interpolation (smoothstep function).
    
    Args:
        t: Value in range [0, 1]
        
    Returns:
        Smoothly interpolated value in range [0, 1]
    """
    t = np.clip(t, 0, 1)
    return t * t * (3 - 2 * t)


def compute_look_at_point(positions, current_idx, look_ahead=10):
    """
    Compute the look-at point by looking ahead on the trajectory.
    
    Args:
        positions: Array of trajectory positions
        current_idx: Current position index
        look_ahead: Number of frames to look ahead
        
    Returns:
        Look-at point as numpy array
    """
    target_idx = min(current_idx + look_ahead, len(positions) - 1)
    return positions[target_idx]


def create_roaming_video(pcd_colored, traj_line, traj_points, keyframes, 
                         output_path, fps=30, duration=30.0, height_offset=1.0,
                         width=1920, height=1080, fov=60.0, look_down_offset=0.0,
                         near_clip=0.1, far_clip=1000.0):
    """
    Create roaming video along trajectory with transition to overhead view
    using Open3D's official OffscreenRenderer API.
    
    Args:
        pcd_colored: Colored point cloud geometry
        traj_line: Trajectory line mesh (can be None)
        traj_points: Trajectory keyframe points (can be None)
        keyframes: List of keyframe dicts with positions
        output_path: Path to save output video (e.g., 'roaming.mp4')
        fps: Video frame rate (default: 30)
        duration: Total video duration in seconds (default: 30.0)
        height_offset: Camera height above trajectory in meters (default: 1.0)
        width: Video width in pixels (default: 1920)
        height: Video height in pixels (default: 1080)
        fov: Camera field of view in degrees (default: 60.0)
        look_down_offset: How much lower the look-at point is relative to camera (default: 0.0)
                          Positive values make camera look down, showing more ground
        near_clip: Near clipping plane distance in meters (default: 0.1)
        far_clip: Far clipping plane distance in meters (default: 1000.0)
    """
    if not HAS_VIDEO_DEPS:
        raise ImportError("scipy and imageio are required for video generation. "
                          "Install with: pip install scipy imageio imageio-ffmpeg")
    
    print(f"\n{'='*60}")
    print("Creating Roaming Video")
    print(f"{'='*60}")
    print(f"Resolution: {width}x{height}, FPS: {fps}, Duration: {duration}s")
    print(f"Height offset: {height_offset}m, Look-down offset: {look_down_offset}m, FOV: {fov}°")
    print(f"Output: {output_path}")
    
    total_frames = int(fps * duration)
    
    # Phase 1: First-person flythrough (80% of frames)
    # Phase 2: Transition to overhead view (20% of frames)
    flythrough_frames = int(total_frames * 0.8)
    transition_frames = total_frames - flythrough_frames
    
    print(f"Flythrough frames: {flythrough_frames}, Transition frames: {transition_frames}")
    
    # 1. Create OffscreenRenderer
    print("\n[1/6] Initializing OffscreenRenderer...")
    renderer = rendering.OffscreenRenderer(width, height)
    
    # 2. Setup materials
    # Material for point cloud (unlit to preserve vertex colors)
    pcd_material = rendering.MaterialRecord()
    pcd_material.shader = "defaultUnlit"
    pcd_material.point_size = 1.0  # Default point size
    
    # Material for trajectory mesh (lit for better visibility)
    traj_material = rendering.MaterialRecord()
    traj_material.shader = "defaultLit"
    traj_material.base_color = [1.0, 0.0, 0.0, 1.0]  # Red
    
    # Material for trajectory points
    traj_points_material = rendering.MaterialRecord()
    traj_points_material.shader = "defaultUnlit"
    traj_points_material.point_size = 3.0
    
    # 3. Add geometries to scene (disable downsampling to keep all points)
    print("[2/6] Adding geometries to scene...")
    renderer.scene.add_geometry("point_cloud", pcd_colored, pcd_material, 
                                add_downsampled_copy_for_fast_rendering=False)
    
    if traj_line is not None:
        renderer.scene.add_geometry("trajectory_line", traj_line, traj_material,
                                    add_downsampled_copy_for_fast_rendering=False)
    
    if traj_points is not None:
        renderer.scene.add_geometry("trajectory_points", traj_points, traj_points_material,
                                    add_downsampled_copy_for_fast_rendering=False)
    
    # 4. Setup lighting and background
    # Use a near-white background for better visibility
    renderer.scene.set_background([0.95, 0.95, 0.95, 1.0])  # Near white
    renderer.scene.set_lighting(
        rendering.Open3DScene.LightingProfile.NO_SHADOWS,
        np.array([0.577, -0.577, -0.577], dtype=np.float32)
    )
    
    # 5. Interpolate trajectory for smooth camera motion
    print("[3/6] Interpolating trajectory...")
    interp_positions = interpolate_trajectory(keyframes, flythrough_frames)
    
    # Get bounding box for overhead view calculation
    bbox = pcd_colored.get_axis_aligned_bounding_box()
    bbox_center = np.array(bbox.get_center())
    bbox_extent = np.array(bbox.get_max_bound()) - np.array(bbox.get_min_bound())
    
    # Calculate overhead camera position (looking down at center)
    overhead_height = max(bbox_extent[0], bbox_extent[1]) * 1.5 + bbox_center[2]
    overhead_eye = np.array([bbox_center[0], bbox_center[1], overhead_height])
    overhead_center = bbox_center.copy()
    
    # Calculate aspect ratio for camera projection
    aspect_ratio = width / height
    
    # Get the scene's camera for direct projection control
    # This is more reliable than setup_camera for controlling near/far clip
    scene_camera = renderer.scene.camera
    
    print(f"Near clip: {near_clip}m, Far clip: {far_clip}m")

    # 6. Render frames
    frames = []
    
    # Phase 1: First-person flythrough
    print(f"[4/6] Rendering flythrough frames (0-{flythrough_frames})...")
    look_ahead = max(10, flythrough_frames // len(keyframes))  # Dynamic look-ahead
    
    for i in range(flythrough_frames):
        # Camera position: slightly above trajectory
        eye = interp_positions[i] + np.array([0, 0, height_offset])
        
        # Look-at point: ahead on trajectory, lowered by look_down_offset
        # This makes the camera tilt downward to see more ground/points below
        center = compute_look_at_point(interp_positions, i, look_ahead)
        center = center - np.array([0, 0, look_down_offset])  # Lower the look-at point
        
        # Up vector (z-up)
        up = np.array([0, 0, 1], dtype=np.float32)
        
        # Use Camera's set_projection and look_at for reliable near/far clip control
        # set_projection(fov, aspect_ratio, near, far, fov_type)
        scene_camera.set_projection(fov, aspect_ratio, near_clip, far_clip, 
                                    rendering.Camera.FovType.Vertical)
        # look_at(center, eye, up)
        scene_camera.look_at(center, eye, up)
        
        # Render frame
        img = renderer.render_to_image()
        frames.append(np.asarray(img))
        
        # Progress update
        if (i + 1) % 100 == 0 or i == flythrough_frames - 1:
            print(f"  Flythrough: {i + 1}/{flythrough_frames} frames")
    
    # Record last flythrough camera state for smooth transition
    last_eye = interp_positions[-1] + np.array([0, 0, height_offset])
    last_center = interp_positions[-1] + np.array([0, 0, 0])  # Look at current position
    
    # Phase 2: Transition to overhead view
    print(f"[5/6] Rendering transition frames ({flythrough_frames}-{total_frames})...")
    
    for i in range(transition_frames):
        # Interpolation factor with ease-in-out
        t = i / (transition_frames - 1) if transition_frames > 1 else 1.0
        alpha = ease_in_out(t)
        
        # Interpolate camera position
        eye = (1 - alpha) * last_eye + alpha * overhead_eye
        center = (1 - alpha) * last_center + alpha * overhead_center
        
        # Gradually rotate up vector for top-down view
        # Start with z-up, end with looking down (so up becomes -y or similar)
        up = np.array([0, 0, 1], dtype=np.float32)
        if alpha > 0.5:
            # Gradually tilt up vector for overhead view
            tilt_alpha = (alpha - 0.5) * 2
            up = np.array([0, -tilt_alpha, 1 - tilt_alpha * 0.5], dtype=np.float32)
            up = up / np.linalg.norm(up)
        
        # Use Camera's set_projection and look_at for reliable near/far clip control
        scene_camera.set_projection(fov, aspect_ratio, near_clip, far_clip,
                                    rendering.Camera.FovType.Vertical)
        scene_camera.look_at(center, eye, up)
        
        # Render frame
        img = renderer.render_to_image()
        frames.append(np.asarray(img))
        
        # Progress update
        if (i + 1) % 50 == 0 or i == transition_frames - 1:
            print(f"  Transition: {i + 1}/{transition_frames} frames")
    
    # 7. Export to MP4
    print(f"[6/6] Exporting video to {output_path}...")
    imageio.mimwrite(str(output_path), frames, fps=fps)
    
    print(f"\nVideo saved: {output_path}")
    print(f"Total frames: {len(frames)}, Duration: {len(frames)/fps:.1f}s")
    
    return output_path


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Visualize FAST-LIVO2 keyframes and optionally generate roaming video",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive visualization only
  python visualize_keyframes.py /path/to/output

  # Generate roaming video
  python visualize_keyframes.py /path/to/output --video

  # Custom video settings
  python visualize_keyframes.py /path/to/output --video --fps 60 --duration 45

  # Skip interactive visualization, only generate video
  python visualize_keyframes.py /path/to/output --video --no-display
        """
    )
    
    parser.add_argument(
        'output_dir',
        type=str,
        help='Directory containing keyframes.txt and keyframes/ folder'
    )
    
    # Video generation options
    video_group = parser.add_argument_group('Video Generation')
    video_group.add_argument(
        '--video', '-v',
        action='store_true',
        help='Generate roaming video along trajectory'
    )
    video_group.add_argument(
        '--video-output', '-o',
        type=str,
        default=None,
        help='Output video filename (default: roaming.mp4 in output_dir)'
    )
    video_group.add_argument(
        '--fps',
        type=int,
        default=30,
        help='Video frame rate (default: 30)'
    )
    video_group.add_argument(
        '--duration',
        type=float,
        default=30.0,
        help='Total video duration in seconds (default: 30.0)'
    )
    video_group.add_argument(
        '--width',
        type=int,
        default=1920,
        help='Video width in pixels (default: 1920)'
    )
    video_group.add_argument(
        '--height',
        type=int,
        default=1080,
        help='Video height in pixels (default: 1080)'
    )
    video_group.add_argument(
        '--height-offset',
        type=float,
        default=1.0,
        help='Camera height above trajectory in meters (default: 1.0)'
    )
    video_group.add_argument(
        '--fov',
        type=float,
        default=60.0,
        help='Camera field of view in degrees (default: 60.0)'
    )
    video_group.add_argument(
        '--look-down',
        type=float,
        default=0.0,
        help='How much lower the look-at point is relative to camera in meters (default: 0.0). '
             'Positive values make camera look down, showing more ground points below. '
             'Recommended: 1.0-3.0 for typical outdoor scenes.'
    )
    video_group.add_argument(
        '--near-clip',
        type=float,
        default=0.1,
        help='Near clipping plane distance in meters (default: 0.1). '
             'Points closer than this distance to camera will not be rendered. '
             'Use smaller values (e.g., 0.01) if nearby points disappear.'
    )
    video_group.add_argument(
        '--far-clip',
        type=float,
        default=1000.0,
        help='Far clipping plane distance in meters (default: 1000.0). '
             'Points farther than this distance from camera will not be rendered.'
    )
    
    # Display options
    display_group = parser.add_argument_group('Display Options')
    display_group.add_argument(
        '--no-display',
        action='store_true',
        help='Skip interactive visualization (useful for headless video generation)'
    )
    display_group.add_argument(
        '--voxel-size',
        type=float,
        default=0.1,
        help='Voxel size for downsampling in meters (default: 0.1)'
    )
    
    return parser.parse_args()


def main():
    args = parse_args()
    
    output_dir = args.output_dir
    output_path = Path(output_dir)
    
    if not output_path.exists():
        print(f"Error: Output directory does not exist: {output_dir}")
        sys.exit(1)
    
    # File paths
    poses_file = output_path / "keyframes.txt"
    keyframes_dir = output_path / "keyframes"
    
    if not poses_file.exists():
        print(f"Error: keyframes.txt not found: {poses_file}")
        sys.exit(1)
    
    if not keyframes_dir.exists():
        print(f"Error: keyframes directory not found: {keyframes_dir}")
        sys.exit(1)
    
    # Check video dependencies if video generation requested
    if args.video and not HAS_VIDEO_DEPS:
        print("Error: Video generation requires scipy and imageio.")
        print("Install with: pip install scipy imageio imageio-ffmpeg")
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
    pcd_down = downsample_point_cloud(merged_pcd, voxel_size=args.voxel_size)
    
    # Color by z-coordinate
    print("\n[4/5] Coloring by z-coordinate...")
    pcd_colored = color_by_z_coordinate(pcd_down)
    
    # Create trajectory visualization
    print("\n[5/5] Creating trajectory visualization...")
    # Use bright colors: bright red line, bright yellow points
    traj_line, traj_points = create_trajectory_visualization(
        keyframes,
        line_color=[1.0, 0.0, 0.0],      # Bright red
        point_color=[1.0, 1.0, 0.0],     # Bright yellow
        line_radius=0.10,                # Thick line (0.15m radius)
        point_size=0.15                  # Large keyframe markers (0.25m radius)
    )
    
    # Generate roaming video if requested
    if args.video:
        video_output = args.video_output
        if video_output is None:
            video_output = output_path / "roaming.mp4"
        else:
            video_output = Path(video_output)
            if not video_output.is_absolute():
                video_output = output_path / video_output
        
        create_roaming_video(
            pcd_colored=pcd_colored,
            traj_line=traj_line,
            traj_points=traj_points,
            keyframes=keyframes,
            output_path=video_output,
            fps=args.fps,
            duration=args.duration,
            height_offset=args.height_offset,
            width=args.width,
            height=args.height,
            fov=args.fov,
            look_down_offset=args.look_down,
            near_clip=args.near_clip,
            far_clip=args.far_clip
        )
    
    # Interactive visualization (unless --no-display)
    if not args.no_display:
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
        print("\nStarting interactive visualization...")
        print("Controls:")
        print("  - Mouse: Rotate view")
        print("  - Shift + Mouse: Pan view")
        print("  - Mouse wheel: Zoom")
        print("  - Q or close window: Exit")
        print("\nVisualization includes:")
        print("  - Point cloud (colored by z-axis)")
        print("  - Trajectory line (bright red)")
        print("  - Keyframe positions (bright yellow spheres)")
        
        o3d.visualization.draw_geometries(geometries,
                                          window_name="FAST-LIVO2 Keyframes + Trajectory",
                                          width=1920,
                                          height=1080)
    
    # Save the merged point cloud
    save_path = output_path / "keyframes_merged_downsampled.pcd"
    print(f"\nSaving merged and downsampled point cloud to: {save_path}")
    o3d.io.write_point_cloud(str(save_path), pcd_colored)
    print("Done!")


if __name__ == "__main__":
    main()
