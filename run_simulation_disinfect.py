import os
import subprocess
import signal
import sys
from pathlib import Path
import time

def setup_log_directory(mission, run_number, problem_rate, replan, bdi):
    """Create logs directory for specific run if it doesn't exist"""
    log_dir = Path(f"logsdisinfect/run_{mission}_{run_number}_rate_{problem_rate}_replan_{replan}_bdi_{bdi}")
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir

def start_services(mission, run_number, problem_rate, replan, bdi):
    """Start all required services and return their process objects"""
    processes = {}
    log_dir = setup_log_directory(mission, run_number, problem_rate, replan, bdi)
    
    # Set environment variables
    os.environ['PROBLEM_RATE'] = str(problem_rate)
    os.environ['REPLAN'] = str(replan)
    os.environ['BDI'] = str(bdi)
    
    # Start rosbridge
    # print("Starting rosbridge...")
    # rosbridge_log = open(log_dir / "rosbridge.log", "w")
    # processes["rosbridge"] = subprocess.Popen(
    #     ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"],
    #     stdout=rosbridge_log,
    #     stderr=subprocess.STDOUT
    # )

    if bdi:
        # Start gradle application
        print("Starting gradle application...")
        gradle_log = open(log_dir / "gradle.log", "w")
        processes["gradle"] = subprocess.Popen(
            ["./gradlew", "run"],
            cwd="jason",
            stdout=gradle_log,
            stderr=subprocess.STDOUT
        )
        time.sleep(2)
    
    # Start health service
    print("Starting " + mission + " coordinator...")
    coordinator_log = open(log_dir / "coordinator.log", "w")
    processes["coordinator"] = subprocess.Popen(
        ["ros2", "launch", "coordinator", mission + ".launch.py"],
        stdout=coordinator_log,
        stderr=subprocess.STDOUT
    )

    print("Starting " + mission + " service...")
    mission_log = open(log_dir / (mission + ".log"), "w")
    processes[mission] = subprocess.Popen(
        ["ros2", "launch", "murosa_plan", mission + ".launch.py"],
        stdout=mission_log,
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

def analyze_health_log(mission, run_number, runtime, problem_rate, replan, bdi):
    """Analyze the health log file for specific patterns and return results"""
    coordinator_log_path = f"logsdisinfect/run_{mission}_{run_number}_rate_{problem_rate}_replan_{replan}_bdi_{bdi}/coordinator.log"
    disinfect_log_path = f"logsdisinfect/run_{mission}_{run_number}_rate_{problem_rate}_replan_{replan}_bdi_{bdi}/disinfect.log"
    results = {
        "run_number": run_number,
        "runtime": runtime,
        "problem_rate": problem_rate,
        "replan": replan,
        "failures": 0,
        "total_actions": 0,
        "total_replans": 0,
        "successful_termination": False
    }
    
    try:
        with open(coordinator_log_path, 'r') as f:
            coordinator_log_content = f.read()
            with open(disinfect_log_path, 'r') as f:
                disinfect_log_content = f.read()
                results["had_failure"] = "error found" in coordinator_log_content.lower()
                results["successful_termination"] = "Mission Completed" in coordinator_log_content
                results["total_actions"] = disinfect_log_content.count("Action finished")
                results["total_replans"] = disinfect_log_content.count("Creating plan for:")
                results["failures"] = coordinator_log_content.count("Error found")
    except FileNotFoundError:
        print(f"Warning: Could not find health log for run {run_number}")
    
    return results

def write_summary(results_list, mission, problem_rate, replan, bdi):
    """Write a summary of all simulation runs to a CSV file"""
    summary_path = f"logsdisinfect/simulation_summary_{mission}_{problem_rate}_{replan}_{bdi}.csv"
    with open(summary_path, 'w') as f:
        # Write header
        f.write("Run Number,Problem Rate,Can Replan,Runtime (s),Failures,Total Actions,Total Plans Made,Successful Termination\n")
        
        # Write results for each run
        for result in results_list:
            f.write(f"{result['run_number']},{result['problem_rate']},{result['replan']},{result['runtime']:.2f},{result['failures']},{result['total_actions']},{result['total_replans']},{result['successful_termination']}\n")

def run_simulation_with_timeout(mission, run_number, processes, problem_rate, replan, bdi):
    """Run the simulation with a 60-second timeout"""
    start_time = time.time()
    timeout = 60  # seconds
    
    try:
        # Wait for health service to complete with timeout
        processes["coordinator"].wait(timeout=timeout)
        runtime = time.time() - start_time
        return True, runtime
    except subprocess.TimeoutExpired:
        print(f"\nSimulation run {run_number} exceeded {timeout} seconds. Terminating...")
        runtime = time.time() - start_time
        return False, runtime
    finally:
        # Log the actual runtime
        with open(f"logsdisinfect/run_{mission}_{run_number}_rate_{problem_rate}_replan_{replan}_bdi_{bdi}/runtime.log", "w") as f:
            f.write(f"Runtime: {runtime:.2f} seconds\n")
            if runtime > timeout:
                f.write("Simulation was terminated due to timeout\n")

def main():
    processes = {}
    try:
        # Create main logs directory
        Path("logsdisinfect").mkdir(exist_ok=True)
        
        all_results = []
        # missions = ["health", "patrol"]
        # problem_rates = [0, 25, 50, 75, 100]
        # replan_values = [False, True]
        
        missions = ["disinfect"]
        problem_rates = [50]
        replan_values = [True, False]
        BDI_values = [True, False]

        run_number = 1

        executions = 1
        
        for mission in missions:
            for problem_rate in problem_rates:
                for replan in replan_values:
                    for bdi in BDI_values:
                        for _ in range(executions):  # 30 runs for each combination
                            print(f"\nStarting simulation run {run_number}/{len(missions) * len(problem_rates) * len(replan_values) * len(BDI_values) * executions}...")
                            print(f"Problem Rate: {problem_rate}, Replan: {replan}, BDI: {bdi}")
                                
                            print("All services started. Press Ctrl+C to stop all services.")
                            print(f"Logs are being written to the 'logs/run_{mission}_{run_number}_rate_{problem_rate}_replan_{replan}_bdi_{bdi}' directory.")
                            
                            processes = start_services(mission, run_number, problem_rate, replan, bdi)
                            
                            # Run simulation with timeout
                            completed, runtime = run_simulation_with_timeout(mission,run_number, processes, problem_rate, replan, bdi)
                            
                            print(f"Simulation run {run_number} {'completed' if completed else 'terminated due to timeout'}. Stopping other services...")
                            cleanup_processes(processes)
                            
                            # Analyze health log and store results
                            results = analyze_health_log(mission, run_number, runtime, problem_rate, replan, bdi)
                            all_results.append(results)
                            
                            # Print current run results
                            print(f"Run {run_number} Results:")
                            print(f"  Problem Rate: {problem_rate}")
                            print(f"  Replan: {replan}")
                            print(f"  BDI: {bdi}")
                            print(f"  Runtime: {runtime:.2f} seconds")
                            print(f"  Failure: {results['failures']}")
                            print(f"  Actions: {results['total_actions']}")
                            print(f"  Replans: {results['total_replans']}")
                            print(f"  Successful Termination: {results['successful_termination']}")
                            
                            run_number += 1
                            
                            # Add a small delay between runs to ensure proper cleanup
                            time.sleep(2)
        
                        write_summary(all_results, mission, problem_rate, replan, bdi)
                        all_results = []
        print("\nAll simulation runs completed successfully!")
        print(f"Summary written to logs/simulation_summary.csv")
        
        # Run analysis script
        print("\nRunning analysis script...")
        subprocess.run(["python3", "analyze_results_disinfect.py"], check=True)

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