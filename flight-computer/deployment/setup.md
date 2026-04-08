# Raspberry Pi OS Setup for Hermes Compute Module

Follow these steps to flash Raspberry Pi OS and set up a Compute Module 4 for use with the **Hermes** system.

## Flash Raspberry Pi OS

Follow this guide:  
👉 [How to Flash Raspberry Pi OS on CM4 eMMC using USB Boot (Jeff Geerling)](https://www.jeffgeerling.com/blog/2020/how-flash-raspberry-pi-os-compute-module-4-emmc-usbboot)

## System Configuration

- **Hostname**: `hermes<nr>` (e.g. `hermes1`)
- **User**: `hermes`
- **Password**: `hermes`
- **WiFi SSID**: `asd`
- **WiFi Password**: `123456u8`

## Initial Setup

After installation:

1. **Power on** the Compute Module.
2. **Turn on a hotspot** with password `123456u8`.
3. SSH into the board:
   ```bash
   ssh hermes@hermes<nr>.local
   ```

## Clean Default Files

Clear the home folder:

```bash
rm -r ./*
```

Prevent default folders from respawning:

- Edit `~/.config/user-dirs.dirs`
- Uncomment all lines in the file.

## Install Dependencies

```bash
sudo apt-get update
sudo apt-get install vim git tmux can-utils
```

## Install Rust

See https://rustup.rs, something like `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`.

## Git Access

- Create a [Personal Access Token (PAT)](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens) with read access to the ARIS repositories.
- The 5 repositories `hermes-flight-computer`, `hermes-recovery-gnc`, `hermes-lib-remote-computers`, `hermes-can` and `hermes-mavlink` are required to have Read-Only "Contents" permission.
- Make sure the access token expire after EuRoc.
- Run `git config --global credential.helper store` to remember these credentials.

## Prepare Working Directory

```bash
mkdir hermes
```

## Upload Files

Transfer the necessary files (from your dev machine):

```bash
scp start_fc.sh hermes@hermes<nr>.local:
scp mavlink-config.conf hermes@hermes<nr>.local:
scp cm-config.txt hermes@hermes<nr>.local:
```

## Move Files to Correct Locations

```bash
sudo mkdir /etc/mavlink-router
sudo mv mavlink-config.conf /etc/mavlink-router/main.conf
sudo mv cm-config.txt /boot/firmware/config.txt
```

## Final Steps

Make the startup script executable:

```bash
chmod u+x start_fc.sh
```

## Setup MAVLink Router

Follow the official instructions:  
👉 [MAVLink Router GitHub](https://github.com/mavlink-router/mavlink-router)

## Build the Flight Computer

Follow your project’s `build_fc` procedure and use the PAT previously generated as a password.
