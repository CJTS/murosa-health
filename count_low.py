import os
from pathlib import Path

def count_low_battery_occurrences(base_path, start, end):
    """Count occurrences of 'low_battery' in coordinator.log files."""
    results = {}
    for i in range(start, end + 1):
        folder_name = f"run_patrol_{i}_rate_0_replan_False"
        log_file_path = Path(base_path) / folder_name / "coordinator.log"
        
        if log_file_path.exists():
            with open(log_file_path, 'r') as log_file:
                content = log_file.read()
                count = content.count("low_battery")
                results[folder_name] = count
        else:
            results[folder_name] = "Log file not found"
    
    return results

def main():
    base_path = "/home/user/workspace/murosa-health/logs"
    start = 301
    end = 330
    
    results = count_low_battery_occurrences(base_path, start, end)
    
    for folder, count in results.items():
        print(f"{folder}: {count}")

if __name__ == "__main__":
    main()