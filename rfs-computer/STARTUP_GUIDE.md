# RFS Computer Startup Guide

## Pre-Assembly 

1. **Connect your laptop to the hotspot network**  
   Turn on the hotspot (with a working internet connection) and connect your laptop to it.
   - SSID: `asd`
   - Password: `123456u8`

2. **Connect to the RFS via SSH**
   ```bash
   ssh pi@rfsc.local
   ```
   - Password: `123456u8`

3. **Clone the repository with submodules**
   ```bash
   git clone --recursive https://github.com/aris-space/asteria-remote-computers
   ```
   The `--recursive` flag ensures all necessary submodules are downloaded.

4. **Navigate to the project directory**
   ```bash
   cd asteria-remote-computers/rfs-computer
   ```

5. **Build the RFS computer binary**
   ```bash
   BUILDER_INITIALS="[XX]" cargo build --release --locked --package rfs-computer
   ```
   Replace `[XX]` with your initials (2 letters).

## Assembly

1. **Connect your laptop to the hotspot network**  
   Turn on the hotspot (no internet connection is required) and connect your laptop to it.
   - SSID: `asd`
   - Password: `123456u8`

2. **Connect to the RFS via SSH**
   ```bash
   ssh pi@rfsc.local
   ```
   - Password: `123456u8`

3. **Run the RFS computer binary**
   The binary must already be built on the RFS.
   ```bash
   cd asteria-remote-computers/rfs-computer
   target/release/rfs-computer configs/[CONFIG_FILE].toml
   ```
   Replace `[CONFIG_FILE]` with the appropriate configuration file name.
