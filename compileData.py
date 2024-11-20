import json

trials = []

def readFiles(index):
    trial_result = {
        'name': index,
        'task_time': check_task_timePlansys2(index),
        'status': check_simulation_status(index),
        'replan': check_simulation_replan_need(index)
    }
    print(trial_result)
    trials.append(trial_result)

def check_task_timePlansys2(index):
    with open(f'./logs/%s.log' % (index), 'r') as file:
        lines = file.readlines()
        alllines = ''
        for line in lines:
            alllines = alllines+line
            if "The time of execution of above program is" in line:
                return float(line.split(':')[1])
    return False

def check_simulation_status(index):
    with open(f'./logs/%s.log' % (index), 'r') as file:
        lines = file.readlines()
        alllines = ''
        for line in lines:
            alllines = alllines+line
            if "Movement completed, no more patrol." in line:
                return 'Success'
    return 'Failed'

def check_simulation_replan_need(index):
    with open(f'./logs/%s.log' % (index), 'r') as file:
        lines = file.readlines()
        alllines = ''
        for line in lines:
            alllines = alllines+line
            if "Plan Failed" in line or "Found a problem" in line:
                return 'Replan'
    return 'NoReplan'

print('Compiling')
for i in range(1):
    readFiles('plansys2_%s' % (i + 1))

with open(f'./result.json', 'w') as file:
    json.dump(trials, file)