import pyrealsense2 as rs
import cv2 as cv
import numpy as np
import open3d as o3d
import time
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--mode", type=str, default="calibration")
args = parser.parse_args()

pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("Camera error or not connected!")
    exit(0)

config.enable_stream(rs.stream.depth, rs.format.z16, 30)
config.enable_stream(rs.stream.color, rs.format.rgb8, 30)

pipeline.start(config)

# get active camera profile and camera intrinsic
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# processing blocks
pc = rs.pointcloud()
# TODO: May be not needed
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** 1)

colorizer = rs.colorizer()

def paint_pcd(textcoords, color):
    cw, ch = color.shape[:2][::-1]
    v, u = (textcoords * (cw, ch) + 0.5).astype(np.uint).T
    print(u)
    print(v)
    out = color[u,v]
    return out

align_to = rs.stream.depth

align = rs.align(align_to)
i = 0
while True:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    # Get aligned frames
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if i == 0:
        i+=1
        time.sleep(0.1)
        continue
    i+=1

    #depth_frame = decimate.process(depth_frame)

    depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()

    w,h = depth_intrinsics.width, depth_intrinsics.height

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    
    mapped_frame, color_source = color_frame, color_image

    points = pc.calculate(depth_frame)
    pc.map_to(mapped_frame)
    v,t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape(-1,3) # xyz
    textcoords = np.asanyarray(t).view(np.float32).reshape(-1,2)
    colors = np.asanyarray(color_frame.get_data(), np.float32).reshape(-1,3) / 255.0
    #color_map = paint_pcd(textcoords, colors)

    o3d_pcd = o3d.geometry.PointCloud()
    o3d_pcd.points = o3d.utility.Vector3dVector(verts)
    o3d_pcd.colors = o3d.utility.Vector3dVector(colors)
    if args.mode == "capture":
        o3d.visualization.draw_geometries([o3d_pcd])
        o3d.io.write_point_cloud("obj.ply", o3d_pcd)
    elif args.mode == "calibration":
        cropped = o3d_pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, 0.0), max_bound=(1, 1, 1.0)))
        o3d.visualization.draw_geometries_with_vertex_selection([cropped])
        o3d.io.write_point_cloud("calib.ply", cropped)

    # 3 space = 0.059 m



