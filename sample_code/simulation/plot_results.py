"""
Create plots from the simulation example.

The figures are generated from fresh simulation output each time the script
is run.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from simulation_example import (
    build_state_space_model,
    run_forced_response,
    run_lqr_tracking,
    run_open_loop,
)


HERE = Path(__file__).resolve().parent


def main() -> None:
    output_dir = (
        HERE
        / "simulation_results"
    )

    output_dir.mkdir(
        exist_ok=True
    )

    model = build_state_space_model()

    time, states, _ = run_open_loop(
        model
    )

    plt.figure(
        figsize=(9.0, 5.8)
    )

    for index, name in enumerate(
        model.state_names
    ):
        plt.plot(
            time,
            states[index],
            label=name,
        )

    plt.xlabel(
        "Time [s]"
    )
    plt.ylabel(
        "State perturbation"
    )
    plt.title(
        "Illustrative open-loop response"
    )
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        output_dir / "open_loop_states.png",
        dpi=180,
    )

    plt.close()

    time, states, controls = run_forced_response(
        model
    )

    plt.figure(
        figsize=(9.0, 5.8)
    )

    for index, name in enumerate(
        model.state_names
    ):
        plt.plot(
            time,
            states[index],
            label=name,
        )

    plt.xlabel(
        "Time [s]"
    )
    plt.ylabel(
        "State perturbation"
    )
    plt.title(
        "Illustrative forced response"
    )
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        output_dir / "forced_response_states.png",
        dpi=180,
    )

    plt.close()

    (
        time,
        states,
        controls,
        _lqr,
        C_track,
        x_ss,
        _u_ss,
    ) = run_lqr_tracking(
        model
    )

    tracked = (
        C_track @ states
    )

    targets = (
        C_track @ x_ss
    )

    plt.figure(
        figsize=(9.0, 5.8)
    )

    plt.plot(
        time,
        tracked[0],
        label="w",
    )

    plt.plot(
        time,
        np.full_like(
            time,
            targets[0],
        ),
        linestyle="--",
        label="w target",
    )

    plt.plot(
        time,
        tracked[1],
        label="theta",
    )

    plt.plot(
        time,
        np.full_like(
            time,
            targets[1],
        ),
        linestyle="--",
        label="theta target",
    )

    plt.xlabel(
        "Time [s]"
    )
    plt.ylabel(
        "Tracked output"
    )
    plt.title(
        "Illustrative LQR set-point tracking"
    )
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        output_dir / "lqr_tracking_outputs.png",
        dpi=180,
    )

    plt.close()

    plt.figure(
        figsize=(9.0, 5.8)
    )

    for index, name in enumerate(
        model.control_names
    ):
        plt.plot(
            time,
            controls[index],
            label=name,
        )

    plt.xlabel(
        "Time [s]"
    )
    plt.ylabel(
        "Control perturbation"
    )
    plt.title(
        "Illustrative LQR control history"
    )
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        output_dir / "lqr_controls.png",
        dpi=180,
    )

    plt.close()

    print(
        f"Saved figures to: {output_dir}"
    )


if __name__ == "__main__":
    main()
