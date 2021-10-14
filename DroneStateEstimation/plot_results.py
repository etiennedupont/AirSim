import argparse
import os
import matplotlib.pyplot as plt
import pandas as pd

def plot_trajectory(input_path_gt, color="blue"):
    data = pd.read_csv(input_path_gt)
    plt.plot(data["x"], data["y"], color=color)
    width_arrow = 0.001
    for index, row in data.iterrows():
        if index % 10 != 0:
            continue
        # Below we project the orientation of the local Y axis on the XY plane using the rotation quaternion
        orientation_x = 2*(row["qx"]*row["qy"] - row["qw"]*row["qz"])
        orientation_y = 1 - 2*(row["qx"]*row["qx"] + row["qz"]*row["qz"])
        if orientation_x != 0 or orientation_y != 0:
            orientation_x_norm = 5e-1 * orientation_x / (orientation_x**2+orientation_y**2)
            orientation_y_norm = 5e-1 * orientation_y / (orientation_x**2+orientation_y**2)
            plt.arrow(row["x"],row["y"],orientation_x_norm,orientation_y_norm, width=width_arrow, head_width=width_arrow*100, edgecolor=color)
    plt.gca().set_aspect("equal")
    # fig.savefig(os.path.join(output_path, "fig.svg"))

def main():
    parser = argparse.ArgumentParser(description="Plot drone ground truth trajectory vs estimated trajectory")
    parser.add_argument(
        "--input-path-gt",
        dest="input_path_gt",
        help="path to the csv file containing recorded ground truth data",
        required=True,
    )
    parser.add_argument(
        "--input-path-pred",
        dest="input_path_pred",
        help="path to the csv file containing recorded estimated data",
        required=False,
    )
    parser.add_argument(
        "--output-path",
        dest="output_path",
        help="path to the output directory",
        required=True,
    )
    args = parser.parse_args()
    fig = plt.figure()
    plot_trajectory(args.input_path_gt, color="green")
    plot_trajectory(args.input_path_pred, color="blue")
    fig.savefig(os.path.join(args.output_path, "fig.svg"))

if __name__ == "__main__":
    main()