import open3d as o3d
import numpy as np
from argparse import ArgumentParser
from scipy.spatial.transform import Rotation

parser = ArgumentParser()
parser.add_argument("--object", type=str, default="default")
parser.add_argument("--mode", type=str, default="completion") # "grasping"
args = parser.parse_args()

# TODO: Should tune scale according to shape completion result.
offsets_dict = {"default":np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
                "table":np.array([-0.0, -0.0, -0.06, 0.0, 0.0, 0.0, 1.0]),
                "plane":np.array([0.0, -0.3, 0.0, -90.0, 0.0, 0.0, 15.0]),
                "coffeebottle":np.array([0.0, -2.0, 0.0, -90.0, 0.0, 0.0, 10.0]),
                "car":np.array([0.0, -0.3, 0.0, -90.0, 0.0, 0.0, 15.0])}

R_offset = Rotation.from_euler("xyz",offsets_dict[args.object][3:6],degrees=True)
scale = offsets_dict[args.object][6]
translation = offsets_dict[args.object][:3]

axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])



if args.mode == "completion":
    pcd = o3d.io.read_point_cloud("obj_cropped.ply")
    pcd.scale(scale, center=[0.0, 0.0, 0.0])
    pcd.rotate(R_offset.as_matrix(), center=[0.0, 0.0, 0.0])
    pcd.translate(translation)
    o3d.visualization.draw_geometries([pcd, axis])
    o3d.io.write_point_cloud(f"{args.object}_canonical.ply", pcd)
elif args.mode == "grasping":
    pcd = o3d.io.read_point_cloud(f"{args.object}_completed.ply")
    pcd.translate(-translation)
    pcd.rotate(R_offset.as_matrix().T, center=[0.0, 0.0, 0.0])
    pcd.scale(1/scale, center=[0.0, 0.0, 0.0])
    o3d.visualization.draw_geometries([pcd, axis])
    print(pcd.get_axis_aligned_bounding_box())
    o3d.io.write_point_cloud(f"{args.object}_grasping.ply", pcd)

