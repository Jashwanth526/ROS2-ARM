# Coordinate Transformation Issue

## Problem
- Blue cylinder actual position: (1.25, 0.85, 0.79)
- Detected in camera: (0.646, -0.002, 0.957) 
- Transformed to base_link: (1.030, 1.869, 1.050) ❌ WRONG!
- Y-coordinate especially wrong: 1.869 vs 0.85 (2.2x error)

## Root Cause
The TF2 transformation from `depth_camera` to `base_link` is producing incorrect results.
Camera has rotation rpy="0 0.75 1.57" which is complex and may not be properly handled.

## Solution Options
1. **Fix TF tree** - Ensure depth_camera frame matches Gazebo's rgbd_camera optical frame
2. **Use point cloud** - Subscribe to `/rgbd_camera/points` instead of depth image
3. **Direct geometry** - Calculate transformation manually using known camera mount
4. **Calibrate empirically** - Add correction factors based on observed vs actual positions

## Recommended: Use Point Cloud
The `/rgbd_camera/points` topic provides already-transformed 3D points in world coordinates.
This bypasses the depth→3D→TF chain entirely.
