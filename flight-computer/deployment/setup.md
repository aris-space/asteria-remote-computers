# Raspberry Pi OS Setup for Flight computer Compute Module

Follow these steps to flash Raspberry Pi OS and set up a Compute Module 4 for use with the **ASTERIA** system.

## Flash Raspberry Pi OS

Follow this guide:  
👉 [How to Flash Raspberry Pi OS on CM4 eMMC using USB Boot (Jeff Geerling)](https://www.jeffgeerling.com/blog/2020/how-flash-raspberry-pi-os-compute-module-4-emmc-usbboot)

## System Configuration

- **Hostname**: `asteria-fc-<nr>` (e.g. `asteria-fc-1`)
- **User**: `asteria`
- **Password**: `asteria`
- **WiFi SSID**: `asd`
- **WiFi Password**: `123456u8`

## Initial Setup

After installation:

1. **Power on** the Compute Module.
2. **Turn on a hotspot** with password `123456u8`.
3. SSH into the board:
   ```bash
   ssh asteria@asteria-fc-<nr>.local
   ```

## Clean Default Files

Clear the home folder with `rm -r ./*` and
prevent default folders from respawning:

- Edit `~/.config/user-dirs.dirs`
- Uncomment all lines in the file.

## Install Dependencies

```bash
sudo apt-get update
sudo apt-get install vim git tmux can-utils gpiod
```

## Build the mavlink heartbeat service.

Setup `cargo-zigbuild` and then run

```bash
cd flight-computer/deployment/mavlink-heartbeat
cargo zigbuild --target aarch64-unknown-linux-gnu --release
```

and copy the binary into `$REMOTE:/usr/local/bin/mavlink-led`

## Upload Files

Transfer the necessary files (from your dev machine in the `files` directory):

```bash
export REMOTE = asteria@asteria-fc-<nr>.local
scp start_fc.sh $REMOTE:
rsync -r --rsync-path="sudo rsync" mavlink-config.conf $REMOTE:/etc/mavlink-router/main.conf
rsync -r --rsync-path="sudo rsync" cm-config.txt $REMOTE:/boot/firmware/config.txt
rsync -r --rsync-path="sudo rsync" systemd/ $REMOTE:/etc/systemd/system/
rsync -r --rsync-path="sudo rsync" wifi-led.sh $REMOTE:/usr/local/bin/wifi-led.sh
```

## Setup MAVLink Router

Follow the official instructions:  
👉 [MAVLink Router GitHub](https://github.com/mavlink-router/mavlink-router)

## Enable the systemd services

On the FC run:

```bash
systemctl enable --now mavlink-router
systemctl enable --now mavlink-led
systemctl enable --now wifi-led
```

## Build the Flight Computer

Follow your project’s `build_fc` procedure and use the PAT previously generated as a password.
