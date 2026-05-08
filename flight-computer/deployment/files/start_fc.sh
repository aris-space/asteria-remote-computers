#!/bin/bash

# Colors
GREY='\033[38;5;244m'
YELLOW='\033[0;33m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Path to state machine config file (overridable via first arg)
CONFIG_FILE="${1:-$HOME/asteria/current-config.toml}"

# Path to flight computer binary (overridable via second arg)
FC_BINARY="${2:-$HOME/asteria/flight-computer}"

# Flight computer startup procedure
echo -e "${GREY}"
df -h /
echo -e "\n${YELLOW}Check if the available disk space is larger than 10GB ${NC}(press any key to continue)"

read -s -n 1

echo -e "\n${NC}Binary path: ${PURPLE}$FC_BINARY${GREY}"
file "$FC_BINARY"
echo -e "${NC}Config file: ${PURPLE}$CONFIG_FILE${GREY}"
head -n 5 "$CONFIG_FILE"
echo "..."
echo -e "\n${YELLOW}Check path to flight-computer executable and state machine config file ${NC}(press any key to continue)"

read -s -n 1

echo "Starting CAN BUS..."
sudo ip link set can0 down
# `restart-ms 50` is a hotfix because CAN often dies (SPI problem??). `ip -details -statistics link show can0` shows the number of restarts.
sudo ip link set can0 up type can bitrate 1000000 dbitrate 1000000 fd on restart-ms 50

sleep 2
echo "Resetting the boards..."
# Send the reset signal to all boards.
cansend can0 010##0
sleep 1

echo -e "\n${CYAN}Starting Flight Computer...${NC}\n"

# Run flight computer with state machine config file as first argument
tmux new-session -A -s "flight-computer" -c ~/asteria -e RUST_LOG="info" -- $FC_BINARY $CONFIG_FILE
