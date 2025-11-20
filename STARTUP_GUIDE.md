# RFS Computer Startup Guide

## Development Environment Setup

### Network Setup

1. **Connect laptop to access point**
   - SSID: `asd5`
   - Password: `123456u8`

2. **Connect to RC via SSH**
   ```bash
   ssh pi@rfsc.local
   ```
   - Password: `123456u8`

3. **Connect RC to WiFi network**
   ```bash
   sudo nmcli device wifi connect "SSID" password "PASSWORD"
   ```
   Replace `SSID` and `PASSWORD` with your actual WiFi credentials.

4. **Test internet connection on RC**
   ```bash
   ping -c 4 8.8.8.8
   ```

5. **Connect your laptop to the same WiFi network**
   - Disconnect from `asd5` access point
   - Connect to the same WiFi network as the RC

### Remote Development Setup

6. **Connect via VS Code Remote**
   - Open VS Code
   - Use the Remote - SSH extension to connect to `pi@rfsc.local`

### Repository Setup

7. **Clone the repository with submodules**
   ```bash
   git clone --recursive -j8 https://github.com/aris-space/asteria-remote-computers
   ```
   The `--recursive` flag ensures all necessary submodules are downloaded.

8. **Navigate to repository directory**
   ```bash
   cd rfs-computer
   ```

### Build and Run

9. **Build and run the RFS computer software**
   ```bash
   cargo run --release --package rfs-computer -- configs/[YOUR_TEST_CONFIG].toml
   ```
   Replace `[YOUR_TEST_CONFIG]` with your actual configuration file name.

## Troubleshooting

### WiFi Connection Issues
- Verify the SSID and check signal strength: `nmcli device wifi list`

### SSH Connection Issues
- Ensure RC and laptop are on the same network
- Verify hostname resolution: `ping rfsc.local`

## Notes

