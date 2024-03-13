# Author: Julian Viereck
# Date: 2020-11-11
# Brief: Script for generating N static walk step config file.

import copy
import yaml
import numpy as np
import os
import os.path


def translate_pos(pos, translate):
    pos["lf"][:2] += translate
    pos["rf"][:2] += translate
    pos["lh"][:2] += translate
    pos["rh"][:2] += translate
    return pos


def get_angle_pos(angle):
    fwd = np.array([0.2, 0.0, 0.0])
    lft = np.array([0.0, 0.1, 0.0])

    c = np.cos(angle)
    s = np.sin(angle)

    lf_init = np.array([0.2, 0.142, 0.0])
    rf_init = np.array([0.2, -0.142, 0.0])
    lh_init = np.array([-0.2, 0.142, 0.0])
    rh_init = np.array([-0.2, -0.142, 0.0])

    return {
        "lf": lf_init + c * fwd + s * lft,
        "rf": rf_init + c * fwd + s * lft,
        "lh": lh_init + c * fwd + s * lft,
        "rh": rh_init + c * fwd + s * lft,
    }, (c * fwd + s * lft)[:2]


# Step forward.
start = {
    "lf": np.array([0.2, 0.142, 0.0]),
    "rf": np.array([0.2, -0.142, 0.0]),
    "lh": np.array([-0.2, 0.142, 0.0]),
    "rh": np.array([-0.2, -0.142, 0.0]),
}

cfg_file = "cfg_solo12_step_static_fwd.yaml"


def config_emitter(start, ends, output_name):
    f = lambda a: float(a)

    # Generate one plan and then dump it starting with left and right leg first.
    lf_init = start["lf"][:2]
    rf_init = start["rf"][:2]
    lh_init = start["lh"][:2]
    rh_init = start["rh"][:2]

    with open(cfg_file) as file:
        config = yaml.load(file)

    pv = config["planner_variables"]

    t_end = 1.40 * len(ends) + 0.1

    # Add motions entries. Used by the centroidal neural network.
    motions = []

    center = (lf_init + rf_init + lh_init + rh_init) / 4
    motions.append(
        {
            "last_time": float(1.39),
            "x": float(center[0]),
            "y": float(center[1]),
            "yaw": float(0.0),
        }
    )

    for i, end in enumerate(ends):
        lf = end["lf"][:2]
        rf = end["rf"][:2]
        lh = end["lh"][:2]
        rh = end["rh"][:2]

        center = (lf + rf + lh + rh) / 4
        pv["com_viapoints"]["via%d" % (i)] = [
            1.4 * (i + 1),
            float(center[0]),
            float(center[1]),
            0.2,
        ]

        motions.append(
            {
                "last_time": float(1.4 * (i + 2) - 0.01),
                "x": float(center[0]),
                "y": float(center[1]),
                "yaw": float(0.0),
            }
        )

    # Have the last via point at the very end.
    pv["com_viapoints"]["via%d" % (len(ends) - 1)][0] = t_end

    pv["num_com_viapoints"] = len(ends)

    floor_height = 0.015

    motions[-1]["last_time"] += 2.0
    config["motions"] = motions

    # Initial position.
    irc = config["initial_robot_configuration"]
    center = (lf_init + rf_init + lh_init + rh_init) / 4
    irc["com"] = [float(center[0]), float(center[1]), 0.2]
    irc["eef_pose"]["eef_rf"] = [
        1.0,
        f(rf_init[0]),
        f(rf_init[1]),
        f(floor_height),
        1.0,
        0.0,
        0.0,
        0.0,
    ]
    irc["eef_pose"]["eef_lf"] = [
        1.0,
        f(lf_init[0]),
        f(lf_init[1]),
        f(floor_height),
        1.0,
        0.0,
        0.0,
        0.0,
    ]
    irc["eef_pose"]["eef_rh"] = [
        1.0,
        f(rh_init[0]),
        f(rh_init[1]),
        f(floor_height),
        1.0,
        0.0,
        0.0,
        0.0,
    ]
    irc["eef_pose"]["eef_lh"] = [
        1.0,
        f(lh_init[0]),
        f(lh_init[1]),
        f(floor_height),
        1.0,
        0.0,
        0.0,
        0.0,
    ]

    ###
    # Handling of contacts.

    def dump_file(out_file):
        def float_representer(dumper, value):
            text = "{0:.3f}".format(value)
            return dumper.represent_scalar(u"tag:yaml.org,2002:float", text)

        yaml.add_representer(float, float_representer)

        with open(out_file, "w") as file:
            yaml.dump(config, file, default_flow_style=False)

    config["contact_plan"]["effcnt_lf"] = [
        [
            0.00,
            0.15,
            f(lf_init[0]),
            f(lf_init[1]),
            f(floor_height),
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            -1.0,
        ],
    ]
    config["contact_plan"]["effcnt_rh"] = [
        [
            0.00,
            0.50,
            f(rh_init[0]),
            f(rh_init[1]),
            f(floor_height),
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            -1.0,
        ],
    ]
    config["contact_plan"]["effcnt_rf"] = [
        [
            0.00,
            0.85,
            f(rf_init[0]),
            f(rf_init[1]),
            f(floor_height),
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            -1.0,
        ],
    ]
    config["contact_plan"]["effcnt_lh"] = [
        [
            0.00,
            1.20,
            f(lh_init[0]),
            f(lh_init[1]),
            f(floor_height),
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            -1.0,
        ],
    ]

    for i, end in enumerate(ends):
        lf = end["lf"][:2]
        rf = end["rf"][:2]
        lh = end["lh"][:2]
        rh = end["rh"][:2]

        config["contact_plan"]["effcnt_lf"].append(
            [
                0.25 + 1.40 * i,
                1.55 + 1.40 * i,
                float(lf[0]),
                float(lf[1]),
                floor_height,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                -1.0,
            ]
        )
        config["contact_plan"]["effcnt_rh"].append(
            [
                0.6 + 1.40 * i,
                1.90 + 1.40 * i,
                float(rh[0]),
                float(rh[1]),
                floor_height,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                -1.0,
            ]
        )
        config["contact_plan"]["effcnt_rf"].append(
            [
                0.95 + 1.40 * i,
                2.25 + 1.40 * i,
                float(rf[0]),
                float(rf[1]),
                floor_height,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                -1.0,
            ]
        )
        config["contact_plan"]["effcnt_lh"].append(
            [
                1.30 + 1.40 * i,
                2.60 + 1.40 * i,
                float(lh[0]),
                float(lh[1]),
                floor_height,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                -1.0,
            ]
        )

    # Make sure the last foot stays in touch with the ground.
    config["contact_plan"]["effcnt_lf"][-1][1] = t_end + 0.2
    config["contact_plan"]["effcnt_rh"][-1][1] = t_end + 0.2
    config["contact_plan"]["effcnt_rf"][-1][1] = t_end + 0.2
    config["contact_plan"]["effcnt_lh"][-1][1] = t_end + 0.2

    pv["time_horizon"] = t_end

    dump_file(output_name)
    print(output_name)


start = {
    "lf": np.array([0.2, 0.142, 0.0]),
    "rf": np.array([0.2, -0.142, 0.0]),
    "lh": np.array([-0.2, 0.142, 0.0]),
    "rh": np.array([-0.2, -0.142, 0.0]),
}

if __name__ == "__main__":
    N = 10  # Number of steps to generate.
    # Random seed to use. Uses a fixed random seed to have
    # reproduceable output.
    np.random.seed(55)

    init_pos = copy.deepcopy(start)

    ends = []
    offsets = []

    for i in range(N):
        # Get a new position in a random direction.
        end, offset = get_angle_pos(np.pi * 2 * np.random.rand())

        for off in offsets:
            end = translate_pos(end, off)

        ends.append(end)
        offsets.append(offset)

    config_emitter(init_pos, ends, "cfg_solo12_static_long_%d.yaml" % (N))
