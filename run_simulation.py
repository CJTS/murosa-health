#!/usr/bin/env python3

import os
import subprocess
import signal
import sys
from pathlib import Path
import time

def setup_log_directory(run_number, problem_rate, replan):
    """Create logs directory for specific run if it doesn't exist"""
    log_dir = Path(f"logs/run_{run_number}_rate_{problem_rate}_replan_{replan}")
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir

def cleanup_containers():
    """Remove all existing containers and pods"""
    subprocess.run(["podman", "rm", "-f", "-a"], check=False)
    subprocess.run(["podman", "pod", "rm", "-f", "-a"], check=False)

def start_services(run_number, problem_rate, replan):
    """Start all required services and return their process objects"""
    processes = {}
    log_dir = setup_log_directory(run_number, problem_rate, replan)
    
    # Set environment variables
    os.environ['PROBLEM_RATE'] = str(problem_rate)
    os.environ['REPLAN'] = str(replan)
    
    # Start rosbridge
    print("Starting rosbridge...")
    rosbridge_log = open(log_dir / "rosbridge.log", "w")
    processes["rosbridge"] = subprocess.Popen(
        ["podman-compose", "-f", "experiment_trials.yaml", "up", "rosbridge"],
        stdout=rosbridge_log,
        stderr=subprocess.STDOUT
    )

    # Start gradle application
    print("Starting gradle application...")
    gradle_log = open(log_dir / "gradle.log", "w")
    processes["gradle"] = subprocess.Popen(
        ["./gradlew", "run"],
        cwd="jason_health",
        stdout=gradle_log,
        stderr=subprocess.STDOUT
    )

    time.sleep(10)
    
    # Start health service
    print("Starting health service...")
    health_log = open(log_dir / "health.log", "w")
    processes["health"] = subprocess.Popen(
        ["podman-compose", "-f", "experiment_trials.yaml", "up", "health"],
        stdout=health_log,
        stderr=subprocess.STDOUT
    )
    
    return processes

def cleanup_processes(processes):
    """Terminate all running processes"""
    for name, process in processes.items():
        if process.poll() is None:  # Process is still running
            print(f"Stopping {name} service...")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()

def analyze_health_log(run_number, runtime, problem_rate, replan):
    """Analyze the health log file for specific patterns and return results"""
    log_path = f"logs/run_{run_number}_rate_{problem_rate}_replan_{replan}/health.log"
    results = {
        "run_number": run_number,
        "runtime": runtime,
        "problem_rate": problem_rate,
        "replan": replan,
        "had_failure": False,
        "successful_termination": False
    }
    
    try:
        with open(log_path, 'r') as f:
            log_content = f.read()
            results["had_failure"] = "failure" in log_content.lower()
            results["successful_termination"] = "Success" in log_content
    except FileNotFoundError:
        print(f"Warning: Could not find health log for run {run_number}")
    
    return results

def write_summary(results_list):
    """Write a summary of all simulation runs to a CSV file"""
    summary_path = "logs/simulation_summary.csv"
    with open(summary_path, 'w') as f:
        # Write header
        f.write("Run Number,Problem Rate,Replan,Runtime (s),Had Failure,Successful Termination\n")
        
        # Write results for each run
        for result in results_list:
            f.write(f"{result['run_number']},{result['problem_rate']},{result['replan']},{result['runtime']:.2f},{result['had_failure']},{result['successful_termination']}\n")

def run_simulation_with_timeout(run_number, processes, problem_rate, replan):
    """Run the simulation with a 60-second timeout"""
    start_time = time.time()
    timeout = 60  # seconds
    
    try:
        # Wait for health service to complete with timeout
        processes["health"].wait(timeout=timeout)
        runtime = time.time() - start_time
        return True, runtime
    except subprocess.TimeoutExpired:
        print(f"\nSimulation run {run_number} exceeded {timeout} seconds. Terminating...")
        runtime = time.time() - start_time
        return False, runtime
    finally:
        # Log the actual runtime
        with open(f"logs/run_{run_number}_rate_{problem_rate}_replan_{replan}/runtime.log", "w") as f:
            f.write(f"Runtime: {runtime:.2f} seconds\n")
            if runtime > timeout:
                f.write("Simulation was terminated due to timeout\n")

def main():
    try:
        # Create main logs directory
        Path("logs").mkdir(exist_ok=True)
        
        all_results = []
        problem_rates = [0, 25, 50, 75, 100]
        replan_values = [False, True]
        run_number = 1
        
        for problem_rate in problem_rates:
            for replan in replan_values:
                for _ in range(30):  # 30 runs for each combination
                    print(f"\nStarting simulation run {run_number}/150")
                    print(f"Problem Rate: {problem_rate}, Replan: {replan}")
                    cleanup_containers()
                    
                    print("All services started. Press Ctrl+C to stop all services.")
                    print(f"Logs are being written to the 'logs/run_{run_number}_rate_{problem_rate}_replan_{replan}' directory.")
                    
                    processes = start_services(run_number, problem_rate, replan)
                    
                    # Run simulation with timeout
                    completed, runtime = run_simulation_with_timeout(run_number, processes, problem_rate, replan)
                    
                    print(f"Simulation run {run_number} {'completed' if completed else 'terminated due to timeout'}. Stopping other services...")
                    cleanup_processes(processes)
                    
                    # Analyze health log and store results
                    results = analyze_health_log(run_number, runtime, problem_rate, replan)
                    all_results.append(results)
                    
                    # Print current run results
                    print(f"Run {run_number} Results:")
                    print(f"  Problem Rate: {problem_rate}")
                    print(f"  Replan: {replan}")
                    print(f"  Runtime: {runtime:.2f} seconds")
                    print(f"  Had Failure: {results['had_failure']}")
                    print(f"  Successful Termination: {results['successful_termination']}")
                    
                    run_number += 1
                    
                    # Add a small delay between runs to ensure proper cleanup
                    time.sleep(2)
        
        # Write final summary
        write_summary(all_results)
        print("\nAll simulation runs completed successfully!")
        print(f"Summary written to logs/simulation_summary.csv")
        
        # Run analysis script
        print("\nRunning analysis script...")
        subprocess.run(["python3", "analyze_results.py"], check=True)
        
    except KeyboardInterrupt:
        print("\nReceived interrupt signal. Stopping all services...")
        cleanup_processes(processes)
        sys.exit(0)
    except Exception as e:
        print(f"An error occurred: {e}")
        cleanup_processes(processes)
        sys.exit(1)

if __name__ == "__main__":
    main() 