# Author: Julian Viereck
# Date: 2020-11-11
# Brief: Script for generating N static jumps config file.

import copy
import yaml
import numpy as np
import os
import os.path

# Doing multiple jumps in a row - quickly from one to the other.

cfg_file = "cfg_solo12_jump.yaml"


def gen_multiple_jump_yaml(out_file_prefix, translations):
    f = lambda a: float(a)

    def dump_file(out_file):
        def float_representer(dumper, value):
            text = "{0:.3f}".format(value)
            return dumper.represent_scalar(u"tag:yaml.org,2002:float", text)

        yaml.add_representer(float, float_representer)

        with open(out_file, "w") as file:
            yaml.dump(config, file, default_flow_style=False)

    # Generate one plan and then dump it starting with left and right leg first.

    lf_init = np.array([0.2, 0.142])
    rf_init = np.array([0.2, -0.142])
    lh_init = np.array([-0.2, 0.142])
    rh_init = np.array([-0.2, -0.142])

    with open(cfg_file) as file:
        config = yaml.load(file)

    pv = config["planner_variables"]

    t_end = 0.5 + 0.7 * len(translations)
    pv["num_com_viapoints"] = len(translations) + 1
    pv["time_horizon"] = t_end

    ###
    # Handling of contacts.
    floor_heights = 0.015

    config["contact_plan"]["effcnt_lf"] = [
        [
            0.00,
            0.40,
            f(lf_init[0]),
            f(lf_init[1]),
            floor_heights,
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
            0.40,
            f(rh_init[0]),
            f(rh_init[1]),
            floor_heights,
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
            0.40,
            f(rf_init[0]),
            f(rf_init[1]),
            floor_heights,
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
            0.40,
            f(lh_init[0]),
            f(lh_init[1]),
            floor_heights,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            -1.0,
        ],
    ]

    motions = []
    trans = np.array([0.0, 0.0])
    for i, translation in enumerate(translations):
        motion = {
            "last_time": float(0.79 + 0.7 * i),
            "x": float(trans[0]),
            "y": float(trans[1]),
            "yaw": float(0.0),
        }

        trans += translation
        pv["com_viapoints"]["via" + str(i)] = [
            0.8 + 0.7 * i,
            float(trans[0]),
            float(trans[1]),
            0.2,
        ]

        motion["next_via_com"] = [float(trans[0]), float(trans[1]), 0.2]
        motions.append(motion)

        lf = lf_init + trans
        rf = rf_init + trans
        lh = lh_init + trans
        rh = rh_init + trans

        config["contact_plan"]["effcnt_lf"].append(
            [
                0.7 + 0.7 * i,
                1.1 + 0.7 * i,
                float(lf[0]),
                float(lf[1]),
                floor_heights,
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
                0.7 + 0.7 * i,
                1.1 + 0.7 * i,
                float(rf[0]),
                float(rf[1]),
                floor_heights,
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
                0.7 + 0.7 * i,
                1.1 + 0.7 * i,
                float(lh[0]),
                float(lh[1]),
                floor_heights,
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
                0.7 + 0.7 * i,
                1.1 + 0.7 * i,
                float(rh[0]),
                float(rh[1]),
                floor_heights,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                -1.0,
            ]
        )

    pv["com_viapoints"]["via" + str(len(translations))] = [
        t_end,
        float(trans[0]),
        float(trans[1]),
        0.2,
    ]

    motions[-1]["last_time"] += 0.5
    config["motions"] = motions

    config["contact_plan"]["effcnt_lf"][-1][1] += 0.2
    config["contact_plan"]["effcnt_rf"][-1][1] += 0.2
    config["contact_plan"]["effcnt_lh"][-1][1] += 0.2
    config["contact_plan"]["effcnt_rh"][-1][1] += 0.2

    dump_file(out_file_prefix + ".yaml")
    print(out_file_prefix + ".yaml")


def get_angle_pos(angle):
    fwd = np.array([0.2, 0.0])
    lft = np.array([0.0, 0.1])

    return fwd * np.cos(angle) + lft * np.sin(angle)


start = {
    "lf": np.array([0.2, 0.142, 0.0]),
    "rf": np.array([0.2, -0.142, 0.0]),
    "lh": np.array([-0.2, 0.142, 0.0]),
    "rh": np.array([-0.2, -0.142, 0.0]),
}

if __name__ == "__main__":
    N = 10  # Number of jumps to generate.
    # Random seed to use. Uses a fixed random seed to have
    # reproduceable output.
    np.random.seed(55)

    translations = []
    for j in range(N):
        trans = get_angle_pos(np.random.rand() * 2.0 * np.pi)
        translations.append(trans)

    out_file_prefix = "cfg_solo12_jump_%04d" % (N)
    gen_multiple_jump_yaml(out_file_prefix, translations)
