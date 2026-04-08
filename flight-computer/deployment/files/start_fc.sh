#!/bin/bash  
 
# Colors  
GREY='\033[38;5;244m'  
YELLOW='\033[0;33m'  
PURPLE='\033[0;35m'  
CYAN='\033[0;36m'  
NC='\033[0m' # No Color  
 
# Path to flight computer binary  
FC_BINARY="$HOME/hermes/droptest2/fc"  
  
# Path to state machine config file  
CONFIG_FILE="$HOME/hermes/droptest2/configs/drop_test_campaign/drop_test_2.toml" 
  
# Flight computer startup procedure  
echo -e "${GREY}"  
df -h  
echo -e "\n${YELLOW}Check if the available disk space is larger than 15GB ${NC}(press any key to continue)"  
  
read -s -n 1  
  
echo -e "\n${NC}Binary path: ${PURPLE}$FC_BINARY${GREY}" 
file "$FC_BINARY"  
echo "${NC}Config file: ${PURPLE}$CONFIG_FILE${GREY}"  
head "$CONFIG_FILE"  
echo -e "\n${YELLOW}Check path to flight-computer executable and state machine config file ${NC}(press any key to continue)"  
  
read -s -n 1  
  
echo -e "${GREY}\nStarting CAN BUS..."  
sudo ip link set can0 down  
# `restart-ms 50` is a hotfix because CAN often dies (SPI problem??). `ip -details -statistics link show can0` shows the number of restarts.  
sudo ip link set can0 up type can bitrate 1000000 dbitrate 1000000 fd on restart-ms 50  
  
echo "CAN BUS started!"  
  
echo "Resetting the boards..."  
sleep 2  
# Send the reset signal to all boards,. This should later be sent from the fc.  
cansend can0 00A##0  
sleep 1  
  
echo -e "\n${CYAN}Starting Flight Computer...\n"   
    
# Run flight computer with state machine config file as first argument   