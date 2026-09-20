"""
Compare the public trim example with approximate digitizations of
Lee (2021), Figs. 2.12, 2.16, and 2.17.

The reference values are approximate graph readings, not original TRAC
numerical output. The script therefore reports trend diagnostics rather
than claiming formal validation.
"""

from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from trim_example import (
    PublicTrimModel,
    run_speed_sweep,
    KW_TO_HP,
)


HERE = Path(__file__).resolve().parent


def main() -> None:
    reference = pd.read_csv(
        HERE / "reference_data.csv"
    )

    model = PublicTrimModel()

    sweep = run_speed_sweep(
        model,
        reference["speed_kt"].to_numpy(),
    )

    rows = []

    for (
        (_, result),
        (_, ref),
    ) in zip(
        sweep,
        reference.iterrows(),
    ):
        rows.append(
            {
                "speed_kt": ref["speed_kt"],
                "collective_deg": result.collective_deg,
                "longitudinal_cyclic_deg":
                    result.longitudinal_cyclic_deg,
                "beta0_model_deg": result.beta0_deg,
                "beta0_reference_deg": ref["beta0_deg"],
                "beta1c_model_deg": result.beta1c_deg,
                "beta1c_reference_deg": ref["beta1c_deg"],
                "beta1s_model_deg": result.beta1s_deg,
                "beta1s_reference_deg": ref["beta1s_deg"],
                "lambda0_model": result.lambda0,
                "lambda0_reference": ref["lambda0"],
                "lambda1c_model": result.lambda1c,
                "lambda1c_reference": ref["lambda1c"],
                "lambda1s_model": result.lambda1s,
                "lambda1s_reference": ref["lambda1s"],
                "power_model_hp":
                    result.total_power_kw * KW_TO_HP,
                "power_reference_hp": ref["power_hp"],
                "max_scaled_residual":
                    result.max_scaled_residual,
                "converged":
                    result.converged,
            }
        )

    df = pd.DataFrame(
        rows
    )

    output_dir = HERE / "comparison_results"
    output_dir.mkdir(
        exist_ok=True
    )

    df.to_csv(
        output_dir / "comparison.csv",
        index=False,
    )

    plt.figure(
        figsize=(8.8, 5.6)
    )

    plt.plot(
        df["speed_kt"],
        df["lambda0_model"],
        marker="o",
        label="Public model lambda0",
    )

    plt.plot(
        df["speed_kt"],
        df["lambda0_reference"],
        marker="s",
        linestyle="--",
        label="Dissertation lambda0",
    )

    plt.xlabel(
        "Forward speed [kt]"
    )
    plt.ylabel(
        "Mean inflow ratio"
    )
    plt.title(
        "Mean inflow comparison"
    )
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        output_dir / "inflow_mean.png",
        dpi=180,
    )
    plt.close()

    plt.figure(
        figsize=(8.8, 5.6)
    )

    plt.plot(
        df["speed_kt"],
        df["lambda1s_model"],
        marker="o",
        label="Public model lambda1s",
    )

    plt.plot(
        df["speed_kt"],
        df["lambda1s_reference"],
        marker="s",
        linestyle="--",
        label="Dissertation lambda1s",
    )

    plt.xlabel(
        "Forward speed [kt]"
    )
    plt.ylabel(
        "First-harmonic inflow ratio"
    )
    plt.title(
        "Longitudinal inflow-harmonic comparison"
    )
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        output_dir / "inflow_harmonic.png",
        dpi=180,
    )
    plt.close()

    plt.figure(
        figsize=(8.8, 5.6)
    )

    plt.plot(
        df["speed_kt"],
        df["beta1c_model_deg"],
        marker="o",
        label="Public model beta1c",
    )

    plt.plot(
        df["speed_kt"],
        df["beta1c_reference_deg"],
        marker="s",
        linestyle="--",
        label="Dissertation beta1c",
    )

    plt.xlabel(
        "Forward speed [kt]"
    )
    plt.ylabel(
        "Flapping angle [deg]"
    )
    plt.title(
        "Longitudinal flapping comparison"
    )
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        output_dir / "flapping_beta1c.png",
        dpi=180,
    )
    plt.close()

    plt.figure(
        figsize=(8.8, 5.6)
    )

    plt.plot(
        df["speed_kt"],
        df["power_model_hp"],
        marker="o",
        label="Public model",
    )

    plt.plot(
        df["speed_kt"],
        df["power_reference_hp"],
        marker="s",
        linestyle="--",
        label="Dissertation TRAC",
    )

    plt.xlabel(
        "Forward speed [kt]"
    )
    plt.ylabel(
        "Approximate power [hp]"
    )
    plt.title(
        "Power comparison"
    )
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        output_dir / "power.png",
        dpi=180,
    )
    plt.close()

    lambda0_mape = float(
        np.mean(
            np.abs(
                (
                    df["lambda0_model"]
                    - df["lambda0_reference"]
                )
                / df["lambda0_reference"]
            )
        )
        * 100.0
    )

    power_mape = float(
        np.mean(
            np.abs(
                (
                    df["power_model_hp"]
                    - df["power_reference_hp"]
                )
                / df["power_reference_hp"]
            )
        )
        * 100.0
    )

    beta1c_mae = float(
        np.mean(
            np.abs(
                df["beta1c_model_deg"]
                - df["beta1c_reference_deg"]
            )
        )
    )

    lambda1s_mae = float(
        np.mean(
            np.abs(
                df["lambda1s_model"]
                - df["lambda1s_reference"]
            )
        )
    )

    print(
        f"lambda0 mean absolute percentage difference: "
        f"{lambda0_mape:.2f}%"
    )

    print(
        f"power mean absolute percentage difference: "
        f"{power_mape:.2f}%"
    )

    print(
        f"beta1c mean absolute difference: "
        f"{beta1c_mae:.3f} deg"
    )

    print(
        f"lambda1s mean absolute difference: "
        f"{lambda1s_mae:.5f}"
    )

    print(
        f"All cases converged: "
        f"{bool(df['converged'].all())}"
    )


if __name__ == "__main__":
    main()
