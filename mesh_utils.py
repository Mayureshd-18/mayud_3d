import numpy as np
import pymeshlab as pml


if PML_VER == '2022.2.post3':
    pml.PercentageValue = pml.Percentage
    pml.PureValue = pml.AbsoluteValue


def poisson_mesh_reconstruction(points, normals=None):
    # points/normals: [N, 3] np.ndarray

    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # outlier removal
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=10)

    # normals
    if normals is None:
        pcd.estimate_normals()
    else:
        pcd.normals = o3d.utility.Vector3dVector(normals[ind])

    # visualize
    o3d.visualization.draw_geometries([pcd], point_show_normal=False)

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9
    )
    vertices_to_remove = densities < np.quantile(densities, 0.1)
    mesh.remove_vertices_by_mask(vertices_to_remove)

    # visualize
    o3d.visualization.draw_geometries([mesh])

    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)

    print(
        f"[INFO] poisson mesh reconstruction: {points.shape} --> {vertices.shape} / {triangles.shape}"
    )

    return vertices, triangles


def decimate_mesh(
    verts, faces, target, backend="pymeshlab", remesh=False, optimalplacement=True
):
    # optimalplacement: default is True, but for flat mesh must turn False to prevent spike artifect.

    _ori_vert_shape = verts.shape
    _ori_face_shape = faces.shape
	@@ -62,19 +68,20 @@ def decimate_mesh(
        ms.add_mesh(m, "mesh")  # will copy!

        # filters
        # ms.meshing_decimation_clustering(threshold=pml.Percentage(1))
        ms.meshing_decimation_quadric_edge_collapse(
            targetfacenum=int(target), optimalplacement=optimalplacement
        )

        if remesh:
            # ms.apply_coord_taubin_smoothing()
            ms.meshing_isotropic_explicit_remeshing(
                iterations=3, targetlen=pml.Percentage(1)
            )

        # extract mesh
        m = ms.current_mesh()
        verts = m.vertex_matrix()
        faces = m.face_matrix()

	@@ -94,7 +101,24 @@ def clean_mesh(
    repair=True,
    remesh=True,
    remesh_size=0.01,
):
    # verts: [N, 3]
    # faces: [N, 3]

	@@ -110,15 +134,15 @@ def clean_mesh(

    if v_pct > 0:
        ms.meshing_merge_close_vertices(
            threshold=pml.Percentage(v_pct)
        )  # 1/10000 of bounding box diagonal

    ms.meshing_remove_duplicate_faces()  # faces defined by the same verts
    ms.meshing_remove_null_faces()  # faces with area == 0

    if min_d > 0:
        ms.meshing_remove_connected_component_by_diameter(
            mincomponentdiag=pml.Percentage(min_d)
        )

    if min_f > 0:
	@@ -132,11 +156,12 @@ def clean_mesh(
    if remesh:
        # ms.apply_coord_taubin_smoothing()
        ms.meshing_isotropic_explicit_remeshing(
            iterations=3, targetlen=pml.AbsoluteValue(remesh_size)
        )

    # extract mesh
    m = ms.current_mesh()
    verts = m.vertex_matrix()
    faces = m.face_matrix()

	@@ -145,3 +170,129 @@ def clean_mesh(
    )

    return verts, faces
