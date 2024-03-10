import open3d as o3d
import numpy as np

cropped_pcd = o3d.io.read_point_cloud("/home/honglang/PSP/files/point_cloud.pcd")

axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=np.zeros(3))

scene = [cropped_pcd, axes]

o3d.visualization.draw_geometries(scene, window_name="Cropped Point Cloud with XYZ Axes", width=800, height=600)
