#!/usr/bin/env python3
import os
import subprocess
import signal
import sys
from pathlib import Path
import time
import psutil
import gc

def setup_log_directory(mission, run_number, problem_rate, replan, bdi):
    log_dir = Path(f"logsdisinfect/run_{mission}_{run_number}_rate_{problem_rate}_replan_{replan}_bdi_{bdi}")
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir

def _env_for_run(run_number):
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(run_number % 233)  # keep ROS_DOMAIN_ID in 0-232 range
    # keep other envs (PROBLEM_RATE/REPLAN/BDI) set per-process if desired by caller
    return env

def start_services(mission, run_number, problem_rate, replan, bdi):
    processes = {}
    logs = {}
    log_dir = setup_log_directory(mission, run_number, problem_rate, replan, bdi)

    # create a per-run env and also set PROBLEM_RATE/REPLAN/BDI in that env
    common_env = _env_for_run(run_number)
    common_env['PROBLEM_RATE'] = str(problem_rate)
    common_env['REPLAN'] = str(replan)
    common_env['BDI'] = str(bdi)

    # Helper to open log files
    def _open_log(name):
        f = open(log_dir / name, "w", buffering=1)  # line-buffered
        logs[name] = f
        return f

    # On UNIX, start_new_session=True causes the child to be its own session leader (like setsid),
    # so we can kill the whole group via os.killpg(proc.pid, ...)
    popen_kwargs = dict(start_new_session=True, env=common_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    # Start rosbridge
    print("Starting rosbridge...")
    logs["rosbridge.log"] = _open_log("rosbridge.log")
    processes["rosbridge"] = subprocess.Popen(
        ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"],
        **popen_kwargs
    )

    time.sleep(5)

    if bdi:
        print("Starting gradle application...")
        env_gradle = common_env.copy()
        logs["gradle.log"] = _open_log("gradle.log")
        processes["gradle"] = subprocess.Popen(
            ["./gradlew", "run"],
            cwd="jason",
            env=env_gradle,
            start_new_session=True,
            stdout=logs["gradle.log"],
            stderr=subprocess.STDOUT,
            text=True
        )
        time.sleep(5)

    print(f"Starting {mission} coordinator...")
    logs["coordinator.log"] = _open_log("coordinator.log")
    processes["coordinator"] = subprocess.Popen(
        ["ros2", "launch", "coordinator", mission + ".launch.py"],
        **popen_kwargs
    )

    print(f"Starting {mission} service...")
    logs[f"{mission}.log"] = _open_log(f"{mission}.log")
    processes[mission] = subprocess.Popen(
        ["ros2", "launch", "murosa_plan", mission + ".launch.py"],
        **popen_kwargs
    )

    # start threads to stream stdout from those Popen pipes to files (non-blocking)
    import threading
    def _stream_output(proc, logfile):
        try:
            with proc.stdout:
                for line in proc.stdout:
                    logfile.write(line)
        except Exception:
            pass

    for name, proc in processes.items():
        lf = logs.get(f"{name}.log") or logs.get(f"{name}.log") or logs.get("coordinator.log")
        # some entries like 'rosbridge' have "rosbridge.log"
        lf = lf or logs.get(f"{name}.log") or logs.get("rosbridge.log")
        if lf and proc.stdout:
            t = threading.Thread(target=_stream_output, args=(proc, lf), daemon=True)
            t.start()

    return processes, logs

def terminate_process_group(pid, timeout=5):
    """
    Kill a process group safely (tries SIGINT, then SIGTERM, then SIGKILL).
    Requires that the child was started with start_new_session=True or setsid.
    """
    try:
        pgid = os.getpgid(pid)
    except Exception:
        # fallback: maybe process already dead or not permitted
        try:
            p = psutil.Process(pid)
            p.terminate()
            p.wait(timeout=timeout)
        except Exception:
            pass
        return

    # try SIGINT first (gives nodes chance to shutdown cleanly)
    try:
        os.killpg(pgid, signal.SIGINT)
    except PermissionError:
        pass
    except ProcessLookupError:
        return

    # wait a bit
    end = time.time() + timeout
    while time.time() < end:
        alive = False
        for p in psutil.process_iter(["pid"]):
            try:
                if os.getpgid(p.info["pid"]) == pgid:
                    alive = True
                    break
            except Exception:
                continue
        if not alive:
            return
        time.sleep(0.1)

    # try SIGTERM
    try:
        os.killpg(pgid, signal.SIGTERM)
    except Exception:
        pass

    end = time.time() + timeout
    while time.time() < end:
        alive = False
        for p in psutil.process_iter(["pid"]):
            try:
                if os.getpgid(p.info["pid"]) == pgid:
                    alive = True
                    break
            except Exception:
                continue
        if not alive:
            return
        time.sleep(0.1)

    # force kill
    try:
        os.killpg(pgid, signal.SIGKILL)
    except Exception:
        pass

def kill_process_tree(pid):
    """Legacy helper kept for compatibility: attempts psutil tree kill then pg kill"""
    try:
        parent = psutil.Process(pid)
    except psutil.NoSuchProcess:
        return
    # attempt graceful terminate on children
    children = parent.children(recursive=True)
    for child in children:
        try:
            child.terminate()
        except Exception:
            pass
    gone, alive = psutil.wait_procs(children, timeout=3)
    for p in alive:
        try:
            p.kill()
        except Exception:
            pass

    # finally kill the process group if possible
    try:
        terminate_process_group(pid, timeout=3)
    except Exception:
        try:
            parent.terminate()
        except Exception:
            pass

def cleanup_ros2_nodes(exclude_pid=None, ros_domain_id=None):
    """
    Kill lingering ROS2 processes that belong to ros_domain_id.
    If ros_domain_id is None, we still try to kill processes whose cmdline looks like ros2/launch
    but we avoid killing this script (exclude_pid).
    """
    if exclude_pid is None:
        exclude_pid = os.getpid()

    for proc in psutil.process_iter(["pid", "name", "cmdline"]):
        try:
            pid = proc.info["pid"]
            if pid == exclude_pid:
                continue

            # attempt to read the environment for ROS_DOMAIN_ID
            try:
                penv = proc.environ()
                proc_ros_domain = penv.get("ROS_DOMAIN_ID")
            except (psutil.AccessDenied, NotImplementedError):
                proc_ros_domain = None

            # If ros_domain_id provided, only target matching processes
            if ros_domain_id is not None:
                if proc_ros_domain is None:
                    continue
                try:
                    if int(proc_ros_domain) != int(ros_domain_id):
                        continue
                except Exception:
                    continue

            # decide if this looks like a ROS2 process to kill
            cmdline = proc.info.get("cmdline") or []
            cmdstr = " ".join(cmdline).lower()
            looks_like_ros = any(x in cmdstr for x in ("ros2", "roslaunch", "launch", "murosa_plan", "coordinator", "rosbridge"))
            # also consider Python processes that load rclpy (best-effort)
            if not looks_like_ros:
                # check open files / cmdline heuristics
                continue

            print(f"[cleanup_ros2_nodes] Killing group of PID {pid} (cmdline: {cmdline})")
            try:
                terminate_process_group(pid, timeout=3)
            except Exception:
                kill_process_tree(pid)

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
        except Exception as e:
            print("Warning cleanup_ros2_nodes exception:", e)
            continue

def cleanup_processes(processes, exclude_pid=None, ros_domain_id=None):
    if exclude_pid is None:
        exclude_pid = os.getpid()

    # terminate children we started directly (preferred)
    for name, proc in processes.items():
        try:
            if proc and proc.poll() is None:
                print(f"Stopping {name} (pid {proc.pid}) service...")
                try:
                    terminate_process_group(proc.pid, timeout=4)
                except Exception:
                    kill_process_tree(proc.pid)
        except Exception:
            pass

    # As a safety net, scan system for leftover ros2 nodes in the same domain
    cleanup_ros2_nodes(exclude_pid=exclude_pid, ros_domain_id=ros_domain_id)

def cleanup_logs(logs):
    for f in logs.values():
        try:
            f.flush()
            f.close()
        except Exception:
            pass

def run_simulation_with_timeout(mission, run_number, processes, problem_rate, replan, bdi, timeout=40):
    start_time = time.time()
    try:
        # wait for coordinator to exit, else timeout
        coord = processes.get("coordinator")
        if coord is not None:
            coord.wait(timeout=timeout)
            runtime = time.time() - start_time
            return True, runtime
        else:
            # nothing to wait on: short-circuit
            runtime = time.time() - start_time
            return True, runtime
    except subprocess.TimeoutExpired:
        print(f"Simulation run {run_number} exceeded {timeout}s. Terminating...")
        runtime = time.time() - start_time
        return False, runtime
    finally:
        log_file = f"logsdisinfect/run_{mission}_{run_number}_rate_{problem_rate}_replan_{replan}_bdi_{bdi}/runtime.log"
        try:
            with open(log_file, "w") as f:
                f.write(f"Runtime: {runtime:.2f}s\n")
                if runtime > timeout:
                    f.write("Simulation terminated due to timeout\n")
        except Exception:
            pass

def analyze_health_log(mission, run_number, runtime, problem_rate, replan, bdi):
    coordinator_log_path = f"logsdisinfect/run_{mission}_{run_number}_rate_{problem_rate}_replan_{replan}_bdi_{bdi}/coordinator.log"
    disinfect_log_path = f"logsdisinfect/run_{mission}_{run_number}_rate_{problem_rate}_replan_{replan}_bdi_{bdi}/disinfect.log"
    results = {
        "run_number": run_number,
        "runtime": runtime,
        "problem_rate": problem_rate,
        "replan": replan,
        "bdi": bdi,
        "battery_failures": 0,
        "dirty_failures": 0,
        "icu_failures": 0,
        "total_actions": 0,
        "total_replans": 0,
        "total_missions": 0,
        "completed_missions": 0,
        "successful_termination": False
    }

    try:
        with open(coordinator_log_path, 'r') as f:
            coordinator_log_content = f.read()
            try:
                with open(disinfect_log_path, 'r') as f2:
                    disinfect_log_content = f2.read()
            except FileNotFoundError:
                disinfect_log_content = ""

            results["had_failure"] = "error found" in coordinator_log_content.lower()
            results["successful_termination"] = "Mission Completed" in coordinator_log_content
            results["total_actions"] = disinfect_log_content.count("Action finished")
            results["total_replans"] = disinfect_log_content.count("Creating plan for:")
            results["battery_failures"] = disinfect_log_content.count("low_battery")
            results["dirty_failures"] = coordinator_log_content.count("dirty_room")
            results["icu_failures"] = coordinator_log_content.count("icu_room")
            results["total_missions"] = coordinator_log_content.count("Creating mission")
            results["completed_missions"] = coordinator_log_content.count("Mission Completed")
    except FileNotFoundError:
        print(f"Warning: Could not find health log for run {run_number}")

    return results

def write_summary(results_list, mission, problem_rate, replan, bdi):
    summary_path = f"logsdisinfect/simulation_summary_{mission}_{problem_rate}_{replan}_{bdi}.csv"
    with open(summary_path, 'w') as f:
        f.write("Run Number,Problem Rate,Can Replan,Have BDI,Runtime (s),Batery Failures,Dirty Failures,ICU Failures,Total Actions,Total Plans Made,Total Missions,Completed Missions,Successful Termination\n")
        for result in results_list:
            f.write(f"{result['run_number']},{result['problem_rate']},{result['replan']},{result['bdi']},{result['runtime']},{result['battery_failures']},{result['dirty_failures']},{result['icu_failures']},{result['total_actions']},{result['total_replans']},{result['total_missions']},{result['completed_missions']},{result['successful_termination']}\n")

def main():
    Path("logsdisinfect").mkdir(exist_ok=True)
    run_number = 1
    executions = 30
    missions = ["disinfect"]
    problem_rates = [0, 25, 50, 75, 100]
    replan_values = [True, False]
    BDI_values = [True, False]

    all_results = []

    try:
        for mission in missions:
            for problem_rate in problem_rates:
                for replan in replan_values:
                    for bdi in BDI_values:
                        for _ in range(executions):
                            # Cleanup leftover nodes belonging to previous runs for safety
                            cleanup_ros2_nodes(ros_domain_id=run_number)

                            print(f"\nStarting simulation run {run_number}...")
                            processes, logs = start_services(mission, run_number, problem_rate, replan, bdi)
                            completed, runtime = run_simulation_with_timeout(mission, run_number, processes, problem_rate, replan, bdi, timeout=40)

                            print(f"Simulation run {run_number} {'completed' if completed else 'terminated due to timeout'}. Stopping other services...")
                            cleanup_processes(processes, ros_domain_id=run_number)
                            cleanup_logs(logs)

                            time.sleep(1)
                            gc.collect()

                            results = analyze_health_log(mission, run_number, runtime, problem_rate, replan, bdi)
                            all_results.append(results)

                            print(f"Run {run_number} Results:")
                            print(f"  Problem Rate: {problem_rate}")
                            print(f"  Replan: {replan}")
                            print(f"  BDI: {bdi}")
                            print(f"  Runtime: {runtime:.2f} seconds")
                            print(f"  Battery Failure: {results['battery_failures']}")
                            print(f"  Dirty Failure: {results['dirty_failures']}")
                            print(f"  ICU Failure: {results['icu_failures']}")
                            print(f"  Actions: {results['total_actions']}")
                            print(f"  Replans: {results['total_replans']}")
                            print(f"  Total missions: {results['total_missions']}")
                            print(f"  Completed missions: {results['completed_missions']}")
                            print(f"  Successful Termination: {results['successful_termination']}")

                            run_number += 1

                        write_summary(all_results, mission, problem_rate, replan, bdi)
                        all_results = []

        print("All simulations completed successfully!")

    except KeyboardInterrupt:
        print("KeyboardInterrupt: cleaning up...")
        cleanup_processes(locals().get('processes', {}))
        cleanup_logs(locals().get('logs', {}))
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        cleanup_processes(locals().get('processes', {}))
        cleanup_logs(locals().get('logs', {}))
        sys.exit(1)

if __name__ == "__main__":
    main()
