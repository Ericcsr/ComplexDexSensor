import numpy as np
import open3d as o3d

offsets = np.array([0.0, 0.1, 0.0])

pcd = o3d.io.read_point_cloud("obj.ply")
extrinsic = np.load("tf.npz")
R = extrinsic["R"]
t = extrinsic["t"]
pcd.rotate(R, center=(0,0,0))
pcd.translate(t)
cropped = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.2, -0.2, 0.01), max_bound=(0.2, 0.2, 0.3)))
o3d.visualization.draw_geometries([cropped])
o3d.io.write_point_cloud("obj_cropped.py",cropped)