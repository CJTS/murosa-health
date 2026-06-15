gnome-terminal -- bash -c "./scripts/rosbridge.sh; exec bash"
sleep 1
gnome-terminal -- bash -c "./scripts/jason.sh; exec bash"
sleep 2
gnome-terminal -- bash -c "./scripts/coordinator.sh; exec bash"
sleep 1
gnome-terminal -- bash -c "./scripts/agents.sh; exec bash"
sleep 1
gnome-terminal -- bash -c "./scripts/front.sh; exec bash"
sleep 1