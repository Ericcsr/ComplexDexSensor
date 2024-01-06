import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
camera_list = ["455","435","415"]


#Rx = np.array([[1,0,0],[0,0,1],[0,-1,0]])
pcds = []
bottom = 0.015
for camera in camera_list:
    pcd = o3d.io.read_point_cloud(f"obj_{camera}.ply")
    extrinsic = np.load(f"tf_{camera}.npz")
    R = extrinsic["R"]
    t = extrinsic["t"]
    pcd.rotate(R, center=(0,0,0))
    pcd.translate(t)
    cropped = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.2, -0.2, bottom), max_bound=(0.2, 0.2, 0.3)))
    cropped = cropped.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)[0]
    # Compute normals
    #cropped.rotate(Rx, center=[0.0, 0.0, 0.0])
    #cropped.scale(5, center=[0.0, 0.0, 0.0])
    pcds.append(cropped)
o3d.visualization.draw_geometries(pcds)
# # Running ICP
# reg_p2p_1 = o3d.pipelines.registration.registration_colored_icp(pcds[1], pcds[0], 0.01, np.eye(4))
# reg_p2p_2 = o3d.pipelines.registration.registration_colored_icp(pcds[2], pcds[0], 0.01, np.eye(4))
# pcds[1].transform(reg_p2p_1.transformation)
# pcds[2].transform(reg_p2p_2.transformation)

# pcds[0] = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.2, -0.2, 0.01), max_bound=(0.2, 0.2, 0.3)))
# pcds[1] = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.2, -0.2, 0.01), max_bound=(0.2, 0.2, 0.3)))
# pcds[2] = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.2, -0.2, 0.01), max_bound=(0.2, 0.2, 0.3)))

full_point_cloud = o3d.geometry.PointCloud()
points = []
colors = []
for i in range(3):
    points.append(np.asarray(pcds[i].points))
    colors.append(np.asarray(pcds[i].colors))

points = np.concatenate(points, axis=0)
colors = np.concatenate(colors, axis=0)

# Squash the points as floor
bottom_points = points.copy()
bottom_points[:,2] = bottom
bottom_colors = colors.copy()

full_point_cloud.points = o3d.utility.Vector3dVector(np.vstack([points, bottom_points]))
full_point_cloud.colors = o3d.utility.Vector3dVector(np.vstack([colors, bottom_colors]))
sampled = full_point_cloud.farthest_point_down_sample(1000)
# remove outliers again
sampled = sampled.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)[0]
cvx_hull, _ = sampled.compute_convex_hull()
cvx_hull.compute_vertex_normals()
cvx_hull.compute_triangle_normals()
# Only for table experiment
#sampled = cvx_hull.sample_points_poisson_disk(1000)
o3d.visualization.draw_geometries([sampled])
o3d.visualization.draw_geometries([cvx_hull])
o3d.io.write_point_cloud("obj_cropped.ply",sampled)
o3d.io.write_triangle_mesh("obj_cropped.obj",cvx_hull)
print(sampled.get_axis_aligned_bounding_box())
points = np.asarray(sampled.points)
np.random.shuffle(points)
np.save("obj_cropped.npy", points)
