#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
from pathlib import Path

def load_and_analyze_data(file):
    """Load simulation results and perform basic analysis"""
    # Read the CSV file
    df = pd.read_csv(f"logs/{file}.csv")
    
    # Convert boolean columns to proper boolean type
    # df['Replan'] = df['Replan'].astype(bool)
    
    # Create output directory for plots
    plot_dir = Path('logs/analysis_plots')
    plot_dir.mkdir(exist_ok=True)
    
    return df, plot_dir

def plot_runtime_by_problem_rate(df, plot_dir):
    """Plot runtime distribution by problem rate"""
    plt.figure(figsize=(10, 6))
    sns.boxplot(x='Successful Termination', y='Runtime (s)', data=df)
    plt.title('Runtime Distribution by Problem Rate')
    plt.savefig(plot_dir / 'runtime_by_problem_rate.png')
    plt.close()

def plot_success_rate_by_parameters(df, plot_dir):
    """Plot success rate by problem rate and replan status"""
    # Calculate success rate
    # rate = df.groupby(['Problem Rate', 'Replan']).mean().reset_index()
    # rate['Problem Rate'] = rate['Problem Rate'].astype(str)

    fig, ax1 = plt.subplots()
    # plt.figure(figsize=(10, 6))
    sns.lineplot(data=df)
    # ax1.set_ylabel('Runtime (s)')
    # ax2 = ax1.twinx()
    # sns.lineplot(x='Run Number', y='Had Failure', ax=ax2, data=df, linewidth=3)
    # ax2.set_ylabel('Had Failure')
    # ax1.legend_.remove()  # Removes the barplot's legend
    # ax2.legend_.remove()  # Removes the lineplot's legend
    # handles_bar, labels_bar = ax1.get_legend_handles_labels()
    # handles_line, labels_line = ax2.get_legend_handles_labels()
    # ax1.legend(handles_bar + handles_line, labels_bar + labels_line, title="Replan Status", loc="best")
    # plt.title('Success Rate by Problem Rate and Replan Status')
    # plt.ylim(0, 1)
    plt.savefig(plot_dir / 'success_rate_by_parameters.png')
    plt.close()

def plot_failure_rate_by_parameters(df, plot_dir):
    """Plot failure rate by problem rate and replan status"""
    # Calculate failure rate
    failure_rate = df.groupby(['Problem Rate', 'Replan'])['Had Failure'].mean().reset_index()
    
    plt.figure(figsize=(10, 6))
    sns.barplot(x='Problem Rate', y='Had Failure', hue='Replan', data=failure_rate)
    plt.title('Failure Rate by Problem Rate and Replan Status')
    plt.ylim(0, 1)
    plt.savefig(plot_dir / 'failure_rate_by_parameters.png')
    plt.close()

def plot_runtime_heatmap(df, plot_dir):
    """Create a heatmap of average runtime by problem rate and replan status"""
    # Calculate average runtime
    avg_runtime = df.pivot_table(
        values='Runtime (s)',
        index='Problem Rate',
        columns='Replan',
        aggfunc='mean'
    )
    
    plt.figure(figsize=(10, 6))
    sns.heatmap(avg_runtime, annot=True, fmt='.2f', cmap='YlOrRd')
    plt.title('Average Runtime by Problem Rate and Replan Status')
    plt.savefig(plot_dir / 'runtime_heatmap.png')
    plt.close()

def main():
    # Set style for better looking plots
    sns.set_style("whitegrid")
    plt.rcParams['figure.figsize'] = [10, 6]

    missions = ["patrol"]
    problem_rates = [0]
    replan_values = [False]

    df = pd.DataFrame()

    for mission in missions:
        for problem_rate in problem_rates:
            for replan in replan_values:
                # Load and analyze data
                df_file, plot_dir = load_and_analyze_data(f"simulation_summary_{mission}_{problem_rate}_{replan}")
                df = pd.concat([df, df_file], ignore_index=True)
    # Generate plots
    print("Generating analysis plots...")
    plot_runtime_by_problem_rate(df, plot_dir)
    plot_success_rate_by_parameters(df, plot_dir)
    # plot_failure_rate_by_parameters(df, plot_dir)
    # plot_runtime_heatmap(df, plot_dir)
    
    # Print summary statistics
    print(f"\nPlots have been saved to {plot_dir}")

if __name__ == "__main__":
    main() 