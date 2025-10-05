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
    plot_dir = Path('logsdisinfect/analysis_plots')
    plot_dir.mkdir(exist_ok=True)
    return df, plot_dir


# === PLOTS ===

# def plot_runtime_by_problem_rate(df, plot_dir):
#     plt.figure(figsize=(10, 6))
#     sns.boxplot(x='Problem Rate', y='Runtime (s)', data=df, hue='Can Replan')
#     plt.xlabel("Problem Rate (%)")
#     plt.ylabel("Runtime (s)")
#     plt.savefig(plot_dir / 'runtime_by_problem_rate.png')
#     plt.close()

# def plot_success_rate(df, plot_dir):
#     success = df.groupby("Problem Rate")['Successful Termination'].mean().reset_index()
#     plt.figure(figsize=(10, 6))
#     sns.barplot(x='Problem Rate', y='Successful Termination', data=success)
#     plt.ylim(0, 1)
#     plt.ylabel("Success Rate")
#     plt.xlabel("Problem Rate (%)")
#     plt.savefig(plot_dir / 'success_rate.png')
#     plt.close()

# def plot_failures(df, plot_dir):
#     failures = df.groupby("Problem Rate")['Failures'].mean().reset_index()
#     plt.figure(figsize=(10, 6))
#     sns.barplot(x='Problem Rate', y='Failures', data=failures)
#     plt.ylabel("Avg Failures per Run")
#     plt.xlabel("Problem Rate (%)")
#     plt.savefig(plot_dir / 'failures_by_problem_rate.png')
#     plt.close()

# def plot_actions(df, plot_dir):
#     actions = df.groupby("Problem Rate", as_index=False)['Total Actions'].mean()
#     plt.figure(figsize=(10, 6))
#     sns.lineplot(data=actions, x="Problem Rate", y="Total Actions", marker="o")
#     plt.ylabel("Avg Actions")
#     plt.xlabel("Problem Rate (%)")
#     plt.savefig(plot_dir / 'actions_by_problem_rate.png')
#     plt.close()

# def plot_replans(df, plot_dir):
#     replans = df.groupby("Problem Rate", as_index=False)['Total Plans Made'].mean()
#     plt.figure(figsize=(10, 6))
#     sns.lineplot(data=replans, x="Problem Rate", y="Total Plans Made", marker="o", color="orange")
#     plt.ylabel("Avg Plans Made")
#     plt.xlabel("Problem Rate (%)")
#     plt.savefig(plot_dir / 'plans_by_problem_rate.png')
#     plt.close()

# def plot_correlations(df, plot_dir):
#     metrics = ['Runtime (s)', 'Failures', 'Total Actions', 'Total Plans Made', 'Successful Termination']
#     corr = df[metrics].corr()
#     plt.figure(figsize=(10, 10))
#     sns.heatmap(corr, annot=True, cmap="coolwarm", fmt=".2f")
#     plt.title("Correlation Matrix")
#     plt.savefig(plot_dir / 'correlation_heatmap.png')
#     plt.close()


def total_actions(df):
    print(str(df))
    print(str(df["Run Number"]))
    print(str(df["Total Actions"]))
    actions = df.groupby(["Problem Rate", "Can Replan", "Have BDI"], as_index=False)['Total Actions']
    print(str(actions))
    print(str(actions.mean()))

    pivot = df.pivot_table(
        index="Can Replan", 
        columns="Have BDI", 
        values="Total Actions",
        aggfunc="mean"   # or "sum" depending on what makes sense
    )

    pivot.plot(kind="bar", figsize=(8,6))

    plt.title("Total Actions by Can Replan and BDI")
    plt.xlabel("Can Replan")
    plt.ylabel("Total Actions")
    plt.grid(axis="y", linestyle="--", alpha=0.6)
    plt.legend(title="Have BDI")
    plt.tight_layout()
    plt.show()
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

    # plot_runtime_by_problem_rate(df, plot_dir)
    # plot_success_rate(df, plot_dir)
    # plot_failures(df, plot_dir)
    # plot_actions(df, plot_dir)
    # plot_replans(df, plot_dir)
    # plot_correlations(df, plot_dir)
    total_actions(df)

    print(f"\nPlots saved to {plot_dir}")

if __name__ == "__main__":
    main()
