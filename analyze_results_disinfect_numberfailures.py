import matplotlib.pyplot as plt
from pathlib import Path
# Dados da tabela
problem_rates = [10, 25, 50, 75, 100]
failures = [4, 5, 18, 22, 30]
runtime_increase = [0, 1.15, 16.13, 18.89, 22.81]
base_dir = Path("logsdisinfect/summarys")
plot_dir  = base_dir / "analysis_plots2"
# Criar o gráfico
plt.figure(figsize=(8, 5))
plt.plot(failures, runtime_increase, marker='o')
plt.xlabel('Number of Failures')
plt.ylabel('Runtime Increase (%)')
plt.title('Runtime Increase vs. Failures')
plt.grid(True)
plt.savefig(plot_dir / 'number_failures_lineplot.png')

# Exibir
plt.show()