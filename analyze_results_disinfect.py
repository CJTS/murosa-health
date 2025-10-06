#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import numpy as np

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

def failures(summary, plot_dir):
    """Plot total plans made grouped by BDI and Replan"""

    summary_long = summary.melt(
        id_vars=["Problem Rate", "Have BDI", "Can Replan"],
        value_vars=["Batery Failures", "Dirty Failures", "ICU Failures", ],
        var_name="Failures",
        value_name="# Failures"
    )

    g = sns.relplot(
        data=summary_long,
        x="Problem Rate",
        y="# Failures",
        hue="Failures",
        col="Have BDI",
        row="Can Replan",
        kind="line"
    )

    for ax in g.axes.flat:
        ax.set_xticks([0, 25, 50, 75, 100])

    # Tweak the supporting aspects of the plot
    plt.savefig(plot_dir / "failures.png")

def completion(summary, plot_dir):
    """Plot total plans made grouped by BDI and Replan"""

    summary_long = summary.melt(
        id_vars=["Problem Rate", "Have BDI", "Can Replan"],
        value_vars=["Total Missions", "Completed Missions"],
        var_name="Missions",
        value_name="# Missions"
    )

    g = sns.relplot(
        data=summary_long,
        x="Problem Rate",
        y="# Missions",
        hue="Missions",
        col="Have BDI",
        row="Can Replan",
        kind="line"
    )

    for ax in g.axes.flat:
        ax.set_xticks([0, 25, 50, 75, 100])

    # Tweak the supporting aspects of the plot
    plt.savefig(plot_dir / "completion.png")

def plans(summary, plot_dir):
    """Plot total plans made grouped by BDI and Replan"""

    g = sns.relplot(
        data=summary,
        x="Problem Rate",
        y="Total Plans Made",
        col="Have BDI",
        row="Can Replan",
        kind="line"
    )

    for ax in g.axes.flat:
        ax.set_xticks([0, 25, 50, 75, 100])

    # Tweak the supporting aspects of the plot
    plt.savefig(plot_dir / "plans.png")

def plans_same_chart(summary, plot_dir):
    """Plot total plans made with 4 lines (BDI/Replan combinations) and different markers"""

    # Combine Have BDI + Can Replan into a single label
    summary["Config"] = summary.apply(
        lambda row: getLabel(row['Can Replan'], row['Have BDI']),
        axis=1
    )

    plt.figure(figsize=(10, 6))
    sns.lineplot(
        data=summary,
        x="Problem Rate",
        y="Total Plans Made",
        hue="Config",      # different colors
        style="Config",    # different markers
        markers=True,
        dashes=False       # solid lines instead of dashed
    )

    plt.xticks([0, 25, 50, 75, 100])
    plt.title("Total Plans Made by Problem Rate")
    plt.ylabel("Total Plans Made (mean)")
    plt.xlabel("Problem Rate (%)")
    plt.legend(title="Configuration")

    plt.tight_layout()
    plt.savefig(plot_dir / "plans_same_chart.png")
    plt.close()

def actions(summary, plot_dir):
    """Plot total actions made grouped by BDI and Replan"""

    g = sns.relplot(
        data=summary,
        x="Problem Rate",
        y="Total Actions",
        col="Have BDI",
        row="Can Replan",
        kind="line"
    )

    for ax in g.axes.flat:
        ax.set_xticks([0, 25, 50, 75, 100])

    # Tweak the supporting aspects of the plot
    plt.savefig(plot_dir / "actions.png")

def runtime(summary, plot_dir):
    """Plot total plans made grouped by BDI and Replan"""

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

def failures_vs_completion(summary, plot_dir):
    """Bar chart of failures with overlayed lines for each config"""

    # Add Completion % and Total Failures
    summary["Completion %"] = summary["Completed Missions"] / summary["Total Missions"] * 100
    summary["Total Failures"] = summary[["Batery Failures", "Dirty Failures", "ICU Failures"]].sum(axis=1)

    # Map Can Replan to descriptive labels
    summary["Replan Label"] = summary["Can Replan"].map({True: "Plan Recovery", False: "Baseline"})

    plt.figure(figsize=(10, 6))
    sns.scatterplot(
        data=summary,
        x="Total Failures",
        y="Completion %",
        hue="Replan Label",   # Human-readable labels
        style="Have BDI",     # optional: marker style for BDI
        s=100
    )

    # Add regression lines per Replan Label
    sns.lmplot(
        data=summary,
        x="Total Failures",
        y="Completion %",
        hue="Replan Label",
        markers=["o","s"],
        ci=None,
        scatter=False
    )

    plt.xlabel("Total Failures")
    plt.ylabel("Completion (%)")
    plt.title("Effect of Failures on Completion: Baseline vs Plan Recovery")
    plt.tight_layout()
    plt.savefig(plot_dir / "failures_vs_completion.png")
    plt.close()

def completion_vs_problem_rate(summary, plot_dir):
    """
    Show how plan completion changes with Problem Rate,
    comparing Baseline vs Plan Recovery.
    """

    # Calculate Completion %
    summary["Completion %"] = summary["Completed Missions"] / summary["Total Missions"] * 100

    # Human-readable label for Can Replan
    summary["Replan Label"] = summary["Can Replan"].map({True: "Plan Recovery", False: "Baseline"})

    plt.figure(figsize=(10, 6))

    # Lineplot with markers
    sns.lineplot(
        data=summary,
        x="Problem Rate",
        y="Completion %",
        hue="Replan Label",
        style="Have BDI",  # optional: different marker for BDI
        markers=True,
        dashes=False,
        palette="Set1",
        linewidth=2
    )

    plt.xticks([0, 25, 50, 75, 100])
    plt.xlabel("Problem Rate (%)")
    plt.ylabel("Completion (%)")
    plt.title("Plan Completion vs Problem Rate")
    plt.grid(True)
    plt.legend(title="Configuration")
    plt.tight_layout()
    plt.savefig(plot_dir / "completion_vs_problem_rate.png")
    plt.close()


def getLabel(can_replan, bdi): 
    if can_replan and bdi:
        return "BDI Plan Recovery"
    elif can_replan and not bdi:
        return "Plan Recovery"
    elif not can_replan and bdi:    
        return "BDI Baseline"
    else:
        return "Baseline"

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
    
    df_filtered = df[~np.isclose(df["Runtime (s)"], 40)]
    
    summary = df_filtered.groupby(["Problem Rate", "Have BDI", "Can Replan"], as_index=False).agg({
        "Runtime (s)": "mean",
        "Batery Failures": "mean",
        "Dirty Failures": "mean",
        "ICU Failures": "mean",
        "Total Actions": "mean",
        "Total Plans Made": "mean",
        "Total Missions": "mean",
        "Completed Missions": "mean",
        "Successful Termination": "mean",
    })

    print(summary)

    print("Generating analysis plots...")

    failures(summary, plot_dir)
    plans(summary, plot_dir)
    plans_same_chart(summary, plot_dir)
    runtime(summary, plot_dir)
    actions(summary, plot_dir)
    completion(summary, plot_dir)
    failures_vs_completion(summary, plot_dir)
    completion_vs_problem_rate(summary, plot_dir)

    print(f"\nPlots saved to {plot_dir}")

if __name__ == "__main__":
    main()
