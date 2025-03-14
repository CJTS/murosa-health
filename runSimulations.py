import yaml
import subprocess
import time

def runMuRoSAPlanPatrol(index):
    compose_name = 'experiment_trials_' + index + '.yaml'
    with open(f'./trials/{compose_name}', 'w') as file:
        yaml.dump(get_compose_file(10, False), file)

    file = open(f'./logs/%s.log' % (index), 'w')
    print('Running simulation %s' % (index))

    subprocess.run('podman machine stop', shell=True)
    subprocess.run('podman machine start', shell=True)
    up_docker_controller_str = f'podman-compose -f ./trials/{compose_name} up run'
    subprocess.Popen([up_docker_controller_str], stdout=file, stderr=file, shell=True)
    start = time.time()
    runtime = time.time()
    simulation_timeout_s = 60
    while (runtime - start) <= simulation_timeout_s:
        runtime = time.time()
    print('Stopping simulation %s' % (index))

    stop_docker_controller_str = f'podman-compose -f ./trials/{compose_name} down run'
    subprocess.run(stop_docker_controller_str, shell=True)
    subprocess.run('podman rm -f -a', shell=True)
    subprocess.run('podman pod rm -f -a', shell=True)

def get_compose_file(opened_door, replaning):
    return {
        'version': "2.3",
        'services': {
            'run':  {
                'image': 'planner_nodes',
                'environment': [
                    'REPLAN=' + str(replaning),
                    'PROBLEM_RATE=' + str(opened_door)
                ]
            },
        }
    }

print('Simulating')
for i in range(30):
    runMuRoSAPlanPatrol('plansys2_%s' % (i + 1))
