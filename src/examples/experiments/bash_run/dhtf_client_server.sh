#!/bin/sh
xterm -title "client" -e "cd ~/Documents/ARGoS/argos3-kilobot ; argos3 -c src/examples/experiments/kilobot_ALF_dhtf_client.argos ; read"  &
P1=$!
echo P1 $P1
xterm -title "server" -e "cd ~/Documents/ARGoS/argos3-kilobot ; argos3 -c src/examples/experiments/kilobot_ALF_dhtf_server.argos ; read"&
P2=$! 
echo P2 $P2 
wait $P1 $P2