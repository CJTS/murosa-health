#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
from pathlib import Path

def load_and_analyze_data():
    """Load simulation results and perform basic analysis"""
    # Read the CSV file
    df = pd.read_csv('logs/simulation_summary.csv')
    
    # Convert boolean columns to proper boolean type
    df['Replan'] = df['Replan'].astype(bool)
    
    # Create output directory for plots
    plot_dir = Path('logs/analysis_plots')
    plot_dir.mkdir(exist_ok=True)
    
    return df, plot_dir

def plot_runtime_by_problem_rate(df, plot_dir):
    """Plot runtime distribution by problem rate"""
    plt.figure(figsize=(10, 6))
    sns.boxplot(x='Problem Rate', y='Runtime (s)', data=df)
    plt.title('Runtime Distribution by Problem Rate')
    plt.savefig(plot_dir / 'runtime_by_problem_rate.png')
    plt.close()

def plot_success_rate_by_parameters(df, plot_dir):
    """Plot success rate by problem rate and replan status"""
    # Calculate success rate
    success_rate = df.groupby(['Problem Rate', 'Replan'])['Successful Termination'].mean().reset_index()
    
    plt.figure(figsize=(10, 6))
    sns.barplot(x='Problem Rate', y='Successful Termination', hue='Replan', data=success_rate)
    plt.title('Success Rate by Problem Rate and Replan Status')
    plt.ylim(0, 1)
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
    
    # Load and analyze data
    df, plot_dir = load_and_analyze_data()
    
    # Generate plots
    print("Generating analysis plots...")
    plot_runtime_by_problem_rate(df, plot_dir)
    plot_success_rate_by_parameters(df, plot_dir)
    plot_failure_rate_by_parameters(df, plot_dir)
    plot_runtime_heatmap(df, plot_dir)
    
    # Print summary statistics
    print("\nSummary Statistics:")
    print("\nAverage Runtime by Problem Rate:")
    print(df.groupby('Problem Rate')['Runtime (s)'].mean())
    
    print("\nSuccess Rate by Problem Rate and Replan Status:")
    print(df.groupby(['Problem Rate', 'Replan'])['Successful Termination'].mean())
    
    print("\nFailure Rate by Problem Rate and Replan Status:")
    print(df.groupby(['Problem Rate', 'Replan'])['Had Failure'].mean())
    
    print(f"\nPlots have been saved to {plot_dir}")

if __name__ == "__main__":
    main() 