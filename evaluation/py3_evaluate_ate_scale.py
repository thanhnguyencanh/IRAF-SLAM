import sys
import numpy
import numpy as np
import argparse
import py3_associate as associate

import matplotlib

matplotlib.use("Agg")
# matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from matplotlib.patches import Ellipse


def align(model, data):
    """Align two trajectories using the method of Horn (closed-form).

    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)

    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    """

    np.set_printoptions(precision=3, suppress=True)
    model = np.asarray(model)
    data = np.asarray(data)

    # Căn chỉnh về gốc tọa độ
    model_mean = model.mean(axis=1)
    data_mean = data.mean(axis=1)

    # Dịch chuyển cả hai quỹ đạo về cùng một điểm gốc
    model_zerocentered = (
        model - model_mean[:, np.newaxis]
    )  # Sử dụng np.newaxis để thêm chiều
    data_zerocentered = (
        data - data_mean[:, np.newaxis]
    )  # Sử dụng np.newaxis để thêm chiều

    # Tính toán ma trận W
    W = np.zeros((3, 3))
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:, column], data_zerocentered[:, column])

    # Phân rã SVD
    U, d, Vh = np.linalg.svd(W.transpose())
    S = np.eye(3)

    # Điều chỉnh S nếu cần
    if np.linalg.det(U) * np.linalg.det(Vh) < 0:
        S[2, 2] = -1
    rot = U @ S @ Vh

    # Tính toán các tham số căn chỉnh
    rotmodel = rot @ model_zerocentered
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
        dots += np.dot(data_zerocentered[:, column].transpose(), rotmodel[:, column])
        normi = np.linalg.norm(model_zerocentered[:, column])
        norms += normi * normi

    s = float(dots / norms)

    # Tính toán vector dịch chuyển
    transGT = data_mean - s * rot @ model_mean
    trans = data_mean - rot @ model_mean

    # Dịch chuyển và căn chỉnh các mô hình
    model_alignedGT = s * rot @ model + transGT[:, np.newaxis]
    model_aligned = rot @ model + trans[:, np.newaxis]

    # Tính toán lỗi căn chỉnh
    alignment_errorGT = model_alignedGT - data
    alignment_error = model_aligned - data

    # Tính toán lỗi dịch chuyển
    trans_errorGT = np.sqrt(np.sum(np.square(alignment_errorGT), axis=0))
    trans_error = np.sqrt(np.sum(np.square(alignment_error), axis=0))

    return rot, transGT, trans_errorGT, trans, trans_error, s


def calculate_rms(gt_trajectory, pred_trajectory):
    # Ensure both trajectories have the same number of points
    min_length = min(gt_trajectory.shape[1], pred_trajectory.shape[1])
    gt_trajectory = gt_trajectory[:, :min_length]
    pred_trajectory = pred_trajectory[:, :min_length]

    # Compute the error
    error = np.linalg.norm(gt_trajectory - pred_trajectory, axis=1)

    # Compute RMS
    rms = np.sqrt(np.mean(error**2))

    return rms


def plot_traj(ax, stamps, traj, style, color, label):
    """
    Plot a trajectory using matplotlib.

    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend

    """
    stamps.sort()
    interval = numpy.median([s - t for s, t in zip(stamps[1:], stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i] - last < 2 * interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x) > 0:
            ax.plot(x, y, style, color=color, label=label)
            label = ""
            x = []
            y = []
        last = stamps[i]
    if len(x) > 0:
        ax.plot(x, y, style, color=color, label=label)


if __name__ == "__main__":
    # parse command line
    parser = argparse.ArgumentParser(
        description="""
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    """
    )
    parser.add_argument(
        "first_file",
        help="ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)",
    )
    parser.add_argument(
        "second_file",
        help="estimated trajectory (format: timestamp tx ty tz qx qy qz qw)",
    )
    parser.add_argument(
        "--offset",
        help="time offset added to the timestamps of the second file (default: 0.0)",
        default=0.0,
    )
    parser.add_argument(
        "--scale",
        help="scaling factor for the second trajectory (default: 1.0)",
        default=1.0,
    )
    parser.add_argument(
        "--max_difference",
        help="maximally allowed time difference for matching entries (default: 10000000 ns)",
        default=20000000,
    )
    parser.add_argument(
        "--save",
        help="save aligned second trajectory to disk (format: stamp2 x2 y2 z2)",
    )
    parser.add_argument(
        "--save_associations",
        help="save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)",
    )
    parser.add_argument(
        "--plot",
        help="plot the first and the aligned second trajectory to an image (format: png)",
    )
    parser.add_argument(
        "--verbose",
        help="print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)",
        action="store_true",
    )
    parser.add_argument(
        "--verbose2",
        help="print scale eror and RMSE absolute translational error in meters after alignment with and without scale correction",
        action="store_true",
    )
    args = parser.parse_args()

    first_list = associate.read_file_list(args.first_file, False)
    second_list = associate.read_file_list(args.second_file, False)

    matches = associate.associate(
        first_list, second_list, float(args.offset), float(args.max_difference)
    )
    if len(matches) < 2:
        sys.exit(
            "Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?"
        )
    first_xyz = numpy.matrix(
        [[float(value) for value in first_list[a][0:3]] for a, b in matches]
    ).transpose()
    second_xyz = numpy.matrix(
        [
            [float(value) * float(args.scale) for value in second_list[b][0:3]]
            for a, b in matches
        ]
    ).transpose()

    dictionary_items = second_list.items()
    sorted_second_list = sorted(dictionary_items)

    second_xyz_full = numpy.matrix(
        [
            [
                float(value) * float(args.scale)
                for value in sorted_second_list[i][1][0:3]
            ]
            for i in range(len(sorted_second_list))
        ]
    ).transpose()  # sorted_second_list.keys()]).transpose()
    second_xyz_full = numpy.matrix(
        [
            [
                float(value) * float(args.scale)
                for value in sorted_second_list[i][1][0:3]
            ]
            for i in range(len(sorted_second_list))
        ]
    ).transpose()
    rot, transGT, trans_errorGT, trans, trans_error, scale = align(
        second_xyz, first_xyz
    )
    trans = trans[:, np.newaxis]  # Đảm bảo trans là vector cột

    second_xyz_aligned = scale * (rot @ second_xyz) + trans
    # second_xyz_aligned = scale * rot * second_xyz + trans
    second_xyz_notscaled = rot * second_xyz + trans
    second_xyz_notscaled_full = rot * second_xyz_full + trans
    first_stamps = list(first_list.keys())
    first_stamps.sort()

    first_xyz_full = numpy.matrix(
        [[float(value) for value in first_list[b][0:3]] for b in first_stamps]
    ).transpose()

    second_stamps = list(second_list.keys())
    second_stamps.sort()
    second_xyz_full = numpy.matrix(
        [
            [float(value) * float(args.scale) for value in second_list[b][0:3]]
            for b in second_stamps
        ]
    ).transpose()
    second_xyz_full_aligned = scale * rot * second_xyz_full + trans

    print(
        numpy.sqrt(numpy.dot(trans_error, trans_error) / len(trans_error)),
        scale,
        numpy.sqrt(numpy.dot(trans_errorGT, trans_errorGT) / len(trans_errorGT)),
        calculate_rms(first_xyz_full, second_xyz_full_aligned),
    )

    if args.save_associations:
        file = open(args.save_associations, "w")
        file.write(
            "\n".join(
                [
                    "%f %f %f %f %f %f %f %f" % (a, x1, y1, z1, b, x2, y2, z2)
                    for (a, b), (x1, y1, z1), (x2, y2, z2) in zip(
                        matches,
                        first_xyz.transpose().A,
                        second_xyz_aligned.transpose().A,
                    )
                ]
            )
        )
        file.close()

    if args.save:
        file = open(args.save, "w")
        file.write(
            "\n".join(
                [
                    "%f " % stamp + " ".join(["%f" % d for d in line])
                    for stamp, line in zip(
                        second_stamps, second_xyz_notscaled_full.transpose().A
                    )
                ]
            )
        )
        file.close()

    if args.plot:

        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_traj(
            ax, first_stamps, first_xyz_full.transpose().A, "-", "black", "ground truth"
        )
        plot_traj(
            ax,
            second_stamps,
            second_xyz_full_aligned.transpose().A,
            "-",
            "blue",
            "estimated",
        )
        # label="difference"
        # for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,first_xyz.transpose().A,second_xyz_aligned.transpose().A):
        #     ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
        #     label=""

        ax.legend()

        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        plt.axis("equal")
        plt.savefig(args.plot, format="pdf")
        plt.savefig(f"{args.plot}.png", dpi=600)
