import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

missions = ["disinfect"]
problem_rates = [10, 25, 50, 75]
replan_values = [True, False]

def load_csv(path):
    return pd.read_csv(path)

def success_rate(df):
    return df["Successful Termination"].mean()

def main():
    plot_dir = Path("logsdisinfect/analysis_plots2")
    plot_dir.mkdir(exist_ok=True)

    scenarios = {
        "Baseline":      {"bdi": False, "replan": False},
        "BDI Baseline":         {"bdi": True,  "replan": False},
        "BDI Plan Recovery":            {"bdi": True,  "replan": True},
    }

    results = {label: [] for label in scenarios}

    for rate in problem_rates:
        for label, cfg in scenarios.items():

            if cfg["bdi"]:
                filename = f"logsdisinfect/summarys/simulation_summary_disinfect_BDI_{rate}_{cfg['replan']}.csv"
            else:
                filename = f"logsdisinfect/summarys/simulation_summary_disinfect_{rate}_{cfg['replan']}.csv"

            df = load_csv(filename)
            results[label].append(success_rate(df))

    # ---- PLOT ----
    plt.figure(figsize=(10, 6))

    for label, values in results.items():
        plt.plot(problem_rates, values, marker="o", linewidth=2, label=label)

    plt.xlabel("Obstacle Occurrence Rate (%)")
    plt.ylabel("Success Rate")
    plt.title("Success Rate Comparison Across Scenarios")
    plt.ylim(0, 1.1)
    plt.grid(True)
    plt.legend()
    plt.xticks(problem_rates)
    plt.tight_layout()

    plt.savefig(plot_dir / "success_rate_lineplot.png")
    plt.close()

    print(f"Gráfico salvo em: {plot_dir/'success_rate_lineplot.png'}")

if __name__ == "__main__":
    main()