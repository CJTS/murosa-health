#!/bin/bash

podman rm -f -a && podman pod rm -f -a
podman build -t planner_nodes .
podman-compose -f experiment_trials.yaml up run
podman rm -f -a && podman pod rm -f -a