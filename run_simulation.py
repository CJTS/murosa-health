#!/usr/bin/env python3

import os
import subprocess
import signal
import sys
from pathlib import Path
import time

def setup_log_directory(run_number):
    """Create logs directory for specific run if it doesn't exist"""
    log_dir = Path(f"logs/run_{run_number}")
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir

def cleanup_containers():
    """Remove all existing containers and pods"""
    subprocess.run(["podman", "rm", "-f", "-a"], check=False)
    subprocess.run(["podman", "pod", "rm", "-f", "-a"], check=False)

def start_services(run_number):
    """Start all required services and return their process objects"""
    processes = {}
    log_dir = setup_log_directory(run_number)
    
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

def main():
    try:
        # Create main logs directory
        Path("logs").mkdir(exist_ok=True)
        
        for run_number in range(1, 31):
            print(f"\nStarting simulation run {run_number}/30")
            cleanup_containers()
            
            print("All services started. Press Ctrl+C to stop all services.")
            print(f"Logs are being written to the 'logs/run_{run_number}' directory.")
            
            processes = start_services(run_number)
            
            # Wait for health service to complete
            processes["health"].wait()
            
            print(f"Health service terminated for run {run_number}. Stopping other services...")
            cleanup_processes(processes)
            
            # Add a small delay between runs to ensure proper cleanup
            time.sleep(2)
            
        print("\nAll 30 simulation runs completed successfully!")
        
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