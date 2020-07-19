# point_cloud_sphere_filling

<b>Example of usage</b>
<code> cd point_cloud_sphere_filling/ && mkdir build && cd build </code>
<code> reset && cmake .. && make && ./point_cloud_as_a_set_of_spheres ../pcd_files/thumb_workspace.pcd false ../spheres/thumb_workspace_spheres_0.005_0.01_1.7_50 0.005 0.01 0.001 1.7 30 50 5 255 0 0 </code>

<b>Inputs</b>
1.  point cloud to be filled with spheres (the more points it has, the better but on the cost of longer execution time.)
2.  show the concave hull of the point cloud (true or false), no very useful, to be removed in the future.
3.  basic name (and path) for the generated output files
4.  smallest sphere radius
5.  largest sphere radius
6.  increment value from smallest to largest sphere radii.
7.  overlap distance between nearest sphere, the smaller the value, the closer the resulting sphere centers and viceversa.
8.  [for visualizing the filling spheres] the number of points to draw each of the resulting spheres, in the output spheres point cloud.
9.  desired number of spheres filling the input point cloud.
10. number of iterations, in each iteration the spheres constructed are slightly shifted from the previous iteration, in order to get better coverage of the input point cloud.
11. the rgb values for coloring the output point cloud (merely for visualizing the output spheres).

<b>Outputs</b>
1. a text file containing a list of the resulting spheres, each entry (sphere) holds the x, y, z coordinates of one sphere center and its radius.
2. a point cloud of the resulting spheres for visualization using pcl_viewer.
3. a PNG photo of the input point cloud and the filling spheres at the last view angle before the program is terminated.
