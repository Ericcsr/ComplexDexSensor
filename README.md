# Repo for real object pointcloud collection

- Depth camera: Real sense D415
- System: Ubuntu 22.04, 20.04

## Install
```
pip install open3d
pip install pyrealsense2
```

## Step 1: Collect calibration point cloud
```
python collect_pcd.py --mode calibration
```
Select points with `shift`, resulting point cloud will be stored at `./calib.ply`

## Step 2: Compute extrinsic matrix
```
python compute_extrinsic.py
```
Resulting extrinsic parameter will be saved in `./tf.npz`

## Step 3: Collect object point cloud
```
python collect_pcd.py --mode capture
```
Resulting point cloud will be stored at `./obj.ply`

## Step 4: Transform and crop object point cloud
Adjust `offsets` in `preprocess.py`
```
python preprocess.py
```
Resulting point cloud will be stored at `./obj_cropped.ply`