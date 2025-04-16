#!/bin/bash

# Saia imediatamente se ocorrer algum erro
set -e

source /opt/ros/overlay_ws/install/setup.bash

# Função para verificar se o serviço está rodando em uma porta específica
wait_for_service() {
    local host=$1
    local port=$2
    while ! nc -z "$host" "$port"; do
        echo "Esperando pelo serviço em $host:$port..."
        sleep 10
    done
}

# Inicia o rosbridge_server em segundo plano
echo "Iniciando o rosbridge_server..."
cd src/murosa_plan_health/launch
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
rosbridge_pid=$!

# Aguarda o rosbridge_server estar disponível na porta padrão (9090)
wait_for_service localhost 9090

# Inicia o Jason em segundo plano
echo "Iniciando o Jason..."
cd ../../../jason_health
gradle clean
gradle run &
jason_pid=$!

# Aguarda alguns segundos para garantir que o Jason esteja rodando
sleep 20

# Inicia o murosa_plan_health em segundo plano
echo "Iniciando o murosa_plan_health..."
cd ../src/murosa_plan_health/launch
ros2 launch murosa_plan_health planning.launch.py &
murosa_pid=$!

# Aguarda todos os processos finalizarem
wait $murosa_pid

echo "Todos os serviços foram finalizados."
