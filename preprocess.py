import numpy as np
import open3d as o3d

camera_list = ["455","435", "415"]

offsets = np.array([0.0, -0.06, -0.15])
Rx = np.array([[1,0,0],[0,0,1],[0,-1,0]])
pcds = []

for camera in camera_list:
    pcd = o3d.io.read_point_cloud(f"obj_{camera}.ply")
    extrinsic = np.load(f"tf_{camera}.npz")
    R = extrinsic["R"]
    t = extrinsic["t"]
    pcd.rotate(R, center=(0,0,0))
    pcd.translate(t+offsets)
    cropped = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.2, -0.2, -0.14), max_bound=(0.2, 0.2, 0.15)))
    cropped.rotate(Rx, center=[0.0, 0.0, 0.0])
    cropped.scale(5, center=[0.0, 0.0, 0.0])
    pcds.append(cropped)
o3d.visualization.draw_geometries(pcds)

full_point_cloud = o3d.geometry.PointCloud()
points = []
colors = []
for i in range(3):
    points.append(np.asarray(pcds[i].points))
    colors.append(np.asarray(pcds[i].colors))

points = np.concatenate(points, axis=0)
colors = np.concatenate(colors, axis=0)
full_point_cloud.points = o3d.utility.Vector3dVector(points)
full_point_cloud.colors = o3d.utility.Vector3dVector(colors)

o3d.io.write_point_cloud("obj_cropped.ply",full_point_cloud)
points = np.asarray(cropped.points)
np.random.shuffle(points)
np.save("obj_cropped.npy", points)
