#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

def load_and_analyze_data(filepath):
    """Carrega e prepara os dados de um CSV de simulação"""
    df = pd.read_csv(filepath)

    # Extrai parâmetros do nome do arquivo
    filename = Path(filepath).stem
    parts = filename.split("_")

    # Exemplo esperado: simutaliton_summary_disinfect_25_true
    problem_rate = int(parts[-2])
    replan = parts[-1].lower() == 'true'

    # Adiciona colunas
    df['Problem Rate'] = problem_rate
    df['Replan'] = replan

    return df


def plot_success_rate_by_parameters(df, plot_dir):
    """Gráfico de taxa de sucesso por taxa de obstáculos e replanejamento"""
    rate = df.groupby(['Problem Rate', 'Replan']).mean(numeric_only=True).reset_index()
    rate['Problem Rate'] = rate['Problem Rate'].astype(str)

    fig, ax = plt.subplots(figsize=(10, 6))
    sns.barplot(
        x='Problem Rate',
        y='Successful Termination',
        hue='Replan',
        data=rate,
        palette=['#1f77b4', '#8c564b'],
        ax=ax
    )

    ax.set_xlabel('Problem Occurrence Rate (%)', fontsize=14)
    ax.set_ylabel('Successful Termination Rate', fontsize=14)
    ax.set_ylim(0, 1)

    ax.tick_params(axis='both', labelsize=12)

    ax.legend(title="Replan Status", fontsize=12, title_fontsize=13)

    plt.tight_layout()
    plt.savefig(plot_dir / 'success_rate_by_parameters.png')
    plt.close()


def plot_failure_rate_by_parameters(df, plot_dir):
    """Gráfico de taxa de falhas"""
    failure_rate = df.groupby(['Problem Rate', 'Replan'])['Had Failure'].mean().reset_index()
    
    plt.figure(figsize=(10, 6))
    ax = sns.barplot(x='Problem Rate', y='Had Failure', hue='Replan', data=failure_rate)
    
    plt.xlabel('Problem Occurrence Rate (%)', fontsize=14)
    plt.ylabel('Proportion of Missions with Problems', fontsize=14)
    
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    
    plt.legend(title='Replan', fontsize=14, title_fontsize=15)
    
    plt.ylim(0, 1)
    plt.tight_layout()
    plt.savefig(plot_dir / 'failure_rate_by_parameters.png')
    plt.close()


def plot_runtime_heatmap(df, plot_dir):
    """Heatmap de tempo médio de execução"""
    avg_runtime = df.pivot_table(
        values='Runtime (s)',
        index='Problem Rate',
        columns='Replan',
        aggfunc='mean'
    )
    
    plt.figure(figsize=(10, 6))
    sns.heatmap(avg_runtime, annot=True, fmt='.2f', cmap='YlOrRd', annot_kws={"size": 12})
    plt.xlabel('Replan')
    plt.ylabel('Problem Rate')
    plt.tight_layout()
    plt.savefig(plot_dir / 'runtime_heatmap.png')
    plt.close()


def main():
    sns.set_style("whitegrid")

    problem_rates = [10, 25, 50, 75]
    replan_values = [True, False]
    base_dir = Path("logsdisinfect/summarys")
    plot_dir = base_dir / "analysis_plots3"
    plot_dir.mkdir(exist_ok=True)

    all_data = pd.DataFrame()

    for rate in problem_rates:
        for replan in replan_values:
            filename = f"simulation_summary_disinfect_BDI_{rate}_{replan}.csv"
            filepath = base_dir / filename
            if not filepath.exists():
                print(f"Arquivo não encontrado: {filepath}")
                continue

            df_part = load_and_analyze_data(filepath)
            all_data = pd.concat([all_data, df_part], ignore_index=True)

    if all_data.empty:
        print("Nenhum dado carregado. Verifique os nomes dos arquivos.")
        return

    print("Gerando gráficos de análise...")
    plot_success_rate_by_parameters(all_data, plot_dir)
    plot_failure_rate_by_parameters(all_data, plot_dir)
    # plot_runtime_heatmap(all_data, plot_dir)

    print(f"\n Gráficos salvos em: {plot_dir}")


if __name__ == "__main__":
    main()
