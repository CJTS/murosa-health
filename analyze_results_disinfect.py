#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
from pathlib import Path

def load_and_analyze_data(file):
    """Load simulation results and perform basic analysis"""
    # Read the CSV file
    df = pd.read_csv(f"logsdisinfect/{file}.csv")
    
    # Convert boolean columns to proper boolean type
    df['Replan'] = df['Replan'].astype(bool)
    
    # Create output directory for plots
    plot_dir = Path('logsdisinfect/analysis_plots')
    plot_dir.mkdir(exist_ok=True)
    
    return df, plot_dir

def plot_runtime_by_problem_rate(df, plot_dir):
    """Plot runtime distribution by problem rate"""
    
    # rate = df.groupby(['Problem Rate', 'Replan'])['Runtime (s)'].mean().reset_index()
    plt.figure(figsize=(10, 6))
    sns.boxplot(x='Problem Rate', y='Runtime (s)', data=df, hue='Replan', palette=['#1f77b4', '#ff7f0e'])
    # plt.title('Runtime Distribution by Problem Rate')
    plt.savefig(plot_dir / 'runtime_by_problem_rate.png')
    plt.close()

def plot_success_rate_by_parameters(df, plot_dir):
    """Plot success rate by problem rate and replan status"""
    # Calculate success rate
    rate = df.groupby(['Problem Rate', 'Replan']).mean().reset_index()
    rate['Problem Rate'] = rate['Problem Rate'].astype(str)


    rate_bar = rate.copy()
    rate_bar['Problem Rate'] = rate_bar['Problem Rate'].astype(str)
    fig, ax1 = plt.subplots()
    # plt.figure(figsize=(10, 6))
    sns.barplot(
        x='Problem Rate',
        y='Successful Termination',
        hue='Replan',
        ax=ax1,
        data=rate_bar,
        palette=['#1f77b4', '#8c564b']
    )
    ax1.set_xlabel('Obstacle Occurrence Rate (%)')
    ax1.set_ylabel('Successful Termination')
    #ax2 = ax1.twinx()
    # sns.lineplot(
    #     x=rate['Problem Rate'].astype(float),  # convertendo para float para evitar erro
    #     y='Had Failure',
    #     hue='Replan',
    #     ax=ax2,
    #     data=rate,
    #     palette=['#ff7f0e', '#e377c2'],
    #     marker='o',
    #     linewidth=3
    # )
    #ax2.set_ylabel('Had Failure')
    ax1.set_ylim(0, 1)
    #ax2.set_ylim(0, 1)
    ax1.legend_.remove()  # Removes the barplot's legend
    #ax2.legend_.remove()  # Removes the lineplot's legend
    handles_bar, labels_bar = ax1.get_legend_handles_labels()
    #handles_line, labels_line = ax2.get_legend_handles_labels()
    ax1.legend(handles_bar, labels_bar, title="Replan Status", loc="best")
    # plt.title('Success Rate by Problem Rate and Replan Status')
    plt.ylim(0, 1)
    plt.savefig(plot_dir / 'success_rate_by_parameters.png')
    plt.close()



def plot_failure_rate_by_parameters(df, plot_dir):
    """Plot failure rate by problem rate and replan status"""
    # Calculate failure rate
    failure_rate = df.groupby(['Problem Rate', 'Replan'])['Had Failure'].mean().reset_index()
    
    plt.figure(figsize=(10, 6))
    sns.barplot(x='Problem Rate', y='Had Failure', hue='Replan', data=failure_rate)
    # plt.title('Failure Rate by Problem Rate and Replan Status')
    plt.xlabel('Obstacle Occurrence Rate (%)')
    plt.ylabel('Proportion of Missions with Problems')
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
    sns.heatmap(avg_runtime, annot=True, fmt='.2f', cmap='YlOrRd', annot_kws={"size": 12})
    # plt.title('Average Runtime by Problem Rate and Replan Status')
    plt.savefig(plot_dir / 'runtime_heatmap.png')
    plt.close()

def main():
    # Set style for better looking plots
    sns.set_style("whitegrid")
    plt.rcParams['figure.figsize'] = [10, 6]

    missions = ["disinfect"]
    problem_rates = [10, 25, 50, 75]
    replan_values = [True, False]

    df = pd.DataFrame()

    for mission in missions:
        for problem_rate in problem_rates:
            for replan in replan_values:
                # Load and analyze data
                df_file, plot_dir = load_and_analyze_data(f"simulation_summary_{mission}_{problem_rate}_{replan}")
                df = pd.concat([df, df_file], ignore_index=True)
    # Generate plots
    print("Generating analysis plots...")
    #plot_runtime_by_problem_rate(df, plot_dir)
    plot_success_rate_by_parameters(df, plot_dir)
    plot_failure_rate_by_parameters(df, plot_dir)
    plot_runtime_heatmap(df, plot_dir)
    
    # Print summary statistics
    print(f"\nPlots have been saved to {plot_dir}")

if __name__ == "__main__":
    main() 