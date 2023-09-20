import plotly.graph_objs as go
import numpy as np
from os.path import join as pj
from scipy import stats


def read_xyz(file_path: str):
    with open(file_path, "r") as file:
        lines = file.readlines()
    x, y, z = [], [], []

    for line in lines:
        parts = line.strip().split()
        if len(parts) == 3:
            x.append(float(parts[0]))
            y.append(float(parts[1]))
            z.append(float(parts[2]))

    return x, y, z


def path_to_graph_lay(file_path: str, size: int, name: str):
    x_m, y_m, z_m = read_xyz(file_path)
    return go.Scatter3d(
        name=name,
        x=x_m,
        y=y_m,
        z=z_m,
        mode="markers",
        marker=dict(size=size),  # Adjust the size of the points as needed
    )


def draw_path_lines(file_path: str, line_name: str):
    x_p, y_p, z_p = read_xyz(file_path)

    return go.Scatter3d(
        x=x_p,
        y=y_p,
        z=z_p,
        mode="lines+markers",
        marker=dict(size=4),
        line=dict(width=3),
        name=line_name,
    )


def clean_data(data_points, zscore_threshold=3):
    # calculate Z-scores for each dimension
    # the z-score range chosen is from -3.0 to 3.0 because 99.7% of normally distributed data falls within this range
    zscores = np.abs(stats.zscore(data_points, axis=0))
    # identify outliers by comparing Z-scores to the threshold and filter them out
    is_outlier = np.any(zscores > zscore_threshold, axis=1)
    filtered_points = data_points[~is_outlier]
    if filtered_points.shape[0] == 0:
        raise ValueError("All data points are identified as outliers.")
    return filtered_points


def main():
    scan_dir = "/home/ido/rbd/rbd-slam/RBD-SLAM/scans/20.09.23/12:55:27"
    fig_data = []

    scan_1 = True
    clean_map_1 = False
    scan_2 = True
    clean_map_2 = True
    present_plane = False
    initial_path_1 = True
    if scan_1:
        # fig_data.append(path_to_graph_lay(pj(scan_dir, "map_for_path2.xyz"), 2, "map"))
        fig_data.append(path_to_graph_lay(pj(scan_dir, "1_start.xyz"), 4, "start 1"))
        fig_data.append(path_to_graph_lay(pj(scan_dir, "exit1.xyz"), 4, "exit 1"))
        fig_data.append(path_to_graph_lay(pj(scan_dir, "1_end.xyz"), 4, "end 1"))
        fig_data.append(draw_path_lines(pj(scan_dir, "path1.xyz"), "path 1"))
        fig_data.append(
            path_to_graph_lay(pj(scan_dir, "plane_points.xyz"), 4, "plane points")
        )
        if initial_path_1:
            fig_data.append(
                draw_path_lines(pj(scan_dir, "1_initial_path.xyz"), "initial path 1")
            )

    if scan_2:
        fig_data.append(path_to_graph_lay(pj(scan_dir, "2_start.xyz"), 4, "start 2"))
        fig_data.append(path_to_graph_lay(pj(scan_dir, "exit2.xyz"), 4, "exit 2"))
        fig_data.append(path_to_graph_lay(pj(scan_dir, "2_end.xyz"), 4, "end 2"))
        fig_data.append(draw_path_lines(pj(scan_dir, "path2.xyz"), "path 2"))

    # fig_data.append(
    #     path_to_graph_lay(pj(scan_dir, "1_tree.xyz"), 2, "sparse plane points")
    # )

    if clean_map_1:
        x_m, y_m, z_m = read_xyz(pj(scan_dir, "map_for_path1.xyz"))
        d_map = np.array([x_m[:], y_m[:], z_m[:]]).transpose()
        c_map = clean_data(d_map)

        x_c = c_map[:, 0]
        y_c = c_map[:, 1]
        z_c = c_map[:, 2]

        clean_map = go.Scatter3d(
            name="map",
            x=x_c,
            y=y_c,
            z=z_c,
            mode="markers",
            marker=dict(size=2),  # Adjust the size of the points as needed
        )
        fig_data.append(clean_map)

    if clean_map_2:
        x_m, y_m, z_m = read_xyz(pj(scan_dir, "map_for_path2.xyz"))
        d_map = np.array([x_m[:], y_m[:], z_m[:]]).transpose()
        c_map = clean_data(d_map)

        x_c = c_map[:, 0]
        y_c = c_map[:, 1]
        z_c = c_map[:, 2]

        clean_map = go.Scatter3d(
            name="map",
            x=x_c,
            y=y_c,
            z=z_c,
            mode="markers",
            marker=dict(size=2),  # Adjust the size of the points as needed
        )
        fig_data.append(clean_map)

    if present_plane:
        x, y, z = read_xyz(pj(scan_dir, "plane_points.xyz"))

        point1 = np.array([x[1 - 1], y[1 - 1], z[1 - 1]])
        point2 = np.array([x[2 - 1], y[2 - 1], z[2 - 1]])
        point3 = np.array([x[3 - 1], y[3 - 1], z[3 - 1]])

        normal_vector = np.cross(point2 - point1, point3 - point1)
        normal_vector /= np.linalg.norm(normal_vector)

        # Define a point on the plane (can be any point)
        point_on_plane = point1

        # Create a meshgrid of points for the plane
        x_range = np.linspace(point_on_plane[0] - 0.7, point_on_plane[0] + 0.7, 4)
        y_range = np.linspace(point_on_plane[1] - 0.7, point_on_plane[1] + 0.7, 4)
        xx, yy = np.meshgrid(x_range, y_range)
        # print(np.meshgrid(x_range, y_range))

        zz = (
            -normal_vector[0] * (xx - point_on_plane[0])
            - normal_vector[1] * (yy - point_on_plane[1])
        ) / normal_vector[2] + point_on_plane[2]

        plane_trace = go.Surface(name="plane", x=xx, y=yy, z=zz, opacity=0.7)
        fig_data.append(plane_trace)

    layout = go.Layout(
        scene=dict(
            xaxis=dict(title="X"),
            yaxis=dict(title="Y"),
            zaxis=dict(title="Z"),
        )
    )

    fig = go.Figure(
        data=fig_data,
        layout=layout,
    )

    # Show the plot
    fig.show()


if __name__ == "__main__":
    main()
