#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

def load_and_analyze_data(file):
    """Load simulation results and perform basic analysis"""
    df = pd.read_csv(f"logsdisinfect/{file}.csv")

    # Ensure correct dtypes
    if 'Can Replan' in df.columns:
        df['Can Replan'] = df['Can Replan'].astype(bool)
    if 'Successful Termination' in df.columns:
        df['Successful Termination'] = df['Successful Termination'].astype(int)

    # Create output directory
    plot_dir = Path('plots')
    plot_dir.mkdir(exist_ok=True)
    return df, plot_dir


def failures(df, plot_dir):
    """Plot total plans made grouped by BDI and Replan"""

    summary = df.groupby(["Problem Rate", "Have BDI", "Can Replan"], as_index=False).agg({
        "Batery Failures": "mean",
        "Dirty Failures": "mean",
        "ICU Failures": "mean",
    })

    summary_long = summary.melt(
        id_vars=["Problem Rate", "Have BDI", "Can Replan"],
        value_vars=["Batery Failures", "Dirty Failures", "ICU Failures"],
        var_name="Failures",
        value_name="# Failures"
    )

    print(summary_long)

    g = sns.relplot(
        data=summary_long,
        x="Problem Rate",
        y="# Failures",
        hue="Failures",      # this will separate the lines for Runtime and Battery Failures
        col="Have BDI",
        row="Can Replan"
    )

    for ax in g.axes.flat:
        ax.set_xticks([0, 25, 50, 75, 100])

    # Tweak the supporting aspects of the plot
    plt.savefig(plot_dir / "failures.png")

def plans(df, plot_dir):
    """Plot total plans made grouped by BDI and Replan"""

    summary = df.groupby(["Problem Rate", "Have BDI", "Can Replan"], as_index=False).agg({
        "Total Plans Made": "mean",
    })

    g = sns.relplot(
        data=summary,
        x="Problem Rate",
        y="Total Plans Made",
        col="Have BDI",
        row="Can Replan"
    )

    for ax in g.axes.flat:
        ax.set_xticks([0, 25, 50, 75, 100])

    # Tweak the supporting aspects of the plot
    plt.savefig(plot_dir / "plans.png")

def runtime(df, plot_dir):
    """Plot total plans made grouped by BDI and Replan"""

    summary = df.groupby(["Problem Rate", "Have BDI", "Can Replan"], as_index=False).agg({
        "Runtime (s)": "mean",
    })

    g = sns.relplot(
        data=summary,
        x="Problem Rate",
        y="Runtime (s)",
        col="Have BDI",
        row="Can Replan",
        kind="line"
    )

    for ax in g.axes.flat:
        ax.set_xticks([0, 25, 50, 75, 100])

    # Tweak the supporting aspects of the plot
    plt.savefig(plot_dir / "runtime.png")


# === MAIN ===

def main():
    sns.set_style("whitegrid")
    plt.rcParams['figure.figsize'] = [10, 6]

    missions = ["disinfect"]
    problem_rates = [0, 25, 50, 75, 100]
    replan_values = [True, False]
    bdi_values = [True, False]

    df = pd.DataFrame()
    for mission in missions:
        for problem_rate in problem_rates:
            for replan in replan_values:
                for bdi in bdi_values:
                    try:
                        df_file, plot_dir = load_and_analyze_data(
                            f"simulation_summary_{mission}_{problem_rate}_{replan}_{bdi}"
                        )
                        df = pd.concat([df, df_file], ignore_index=True)
                    except FileNotFoundError:
                        continue


    if df.empty:
        print("No data found.")
        return

    print("Generating analysis plots...")

    failures(df, plot_dir)
    plans(df, plot_dir)
    runtime(df, plot_dir)

    print(f"\nPlots saved to {plot_dir}")

if __name__ == "__main__":
    main()
