# Summary of Setting Up VNC Viewer with XFCE on Raspberry Pi 5 (Ubuntu 24.04)

This document summarizes the steps taken to configure TightVNC with XFCE on a Raspberry Pi 5 running Ubuntu 24.04, enabling remote desktop access via RealVNC Viewer over X11.

## Initial Context
- **System**: Raspberry Pi 5, Ubuntu 24.04 (ARM64), accessed via SSH (`ros@192.168.217.18`).
- **Goal**: Switch from Wayland to X11 and use VNC Viewer for remote desktop access with a functional desktop environment.
- **Starting State**: GDM3 with Wayland, autologin enabled for `ros`.

## Key Steps

### 1. Switching from Wayland to X11
- **Objective**: Enable X11 for better VNC compatibility.
- **Actions**:
  - Edited `/etc/gdm3/custom.conf`:
    ```
    [daemon]
    WaylandEnable=false
    AutomaticLoginEnable=true
    AutomaticLogin=ros
    ```
  - Restarted GDM3:
    ```
    sudo systemctl daemon-reload
    sudo systemctl restart gdm3
    ```
  - Verified X11:
    ```
    sudo loginctl show-session 5 -p Type
    ```
    - Output: `Type=x11`.

### 2. Installing and Configuring TightVNC
- **Objective**: Set up a VNC server for remote access.
- **Actions**:
  - Installed TightVNC:
    ```
    sudo apt install tightvncserver -y
    ```
  - Started manually:
    ```
    sudo -u ros -i tightvncserver :1
    ```
    - Set a VNC password (e.g., `vnc12345`).
  - Confirmed running:
    ```
    ps aux | grep vnc
    ```
    - Output: `Xtightvnc :1 ... -rfbport 5901`.
  - Opened port:
    ```
    sudo ufw allow 5901
    ```

### 3. Resolving Grey Screen with XFCE
- **Issue**: Initial VNC connection showed a grey screen (no desktop environment).
- **Actions**:
  - Installed XFCE:
    ```
    sudo apt install xfce4 xfce4-goodies -y
    ```
  - Updated `/home/ros/.vnc/xstartup`:
    ```
    #!/bin/sh
    xsetroot -solid grey
    startxfce4 &
    ```
    - Made executable:
      ```
      chmod +x /home/ros/.vnc/xstartup
      ```
  - Restarted VNC:
    ```
    tightvncserver -kill :1
    sudo -u ros -i tightvncserver :1
    ```
  - Connected via RealVNC Viewer (`192.168.217.18:1`):
    - Accepted “Unencrypted connection” warning.
    - Result: XFCE desktop displayed successfully.

### 4. Making VNC Persistent
- **Objective**: Ensure TightVNC starts on boot.
- **Actions**:
  - Created systemd service:
    ```
    sudo nano /etc/systemd/system/vncserver@.service
    ```
    - Content:
      ```
      [Unit]
      Description=TightVNC Server on Display %i
      After=network.target gdm3.service

      [Service]
      Type=forking
      User=ros
      ExecStart=/usr/bin/tightvncserver :%i
      ExecStop=/usr/bin/tightvncserver -kill :%i

      [Install]
      WantedBy=multi-user.target
      ```
  - Enabled and started:
    ```
    sudo systemctl daemon-reload
    sudo systemctl enable vncserver@1
    ```
  - Resolved Conflict:
    - Stopped manual instance:
      ```
      tightvncserver -kill :1
      ```
    - Started service:
      ```
      sudo systemctl start vncserver@1
      ```
    - Verified:
      ```
      sudo systemctl status vncserver@1
      ```
      - Output: `Active: active (running)`.

### 5. Addressing Browser Issues
- **Issue**: Firefox (Snap) wouldn’t launch due to confinement errors.
- **Actions**:
  - Attempted Snap Firefox:
    ```
    firefox &
    ```
    - Errors: `update.go:85: cannot change mount namespace ... permission denied`.
  - Considered Alternatives:
    - Suggested non-Snap Firefox or Chromium but pivoted to user request for Opera GX and VS Code.

### 6. Final Adjustments
- **User Requests**:
  - **Opera GX**: Explored via Wine (not pursued due to ARM64 limitations); suggested regular Opera as alternative.
  - **VS Code**: Provided installation steps (not yet confirmed completed).
- **Current State**: VNC with XFCE works; browser issue unresolved pending user preference.

## Final Configuration
- **VNC Server**: TightVNC on `192.168.217.18:1`, persistent via systemd.
- **Desktop**: XFCE (`startxfce4` in `xstartup`).
- **Access**: RealVNC Viewer from `192.168.217.209`, unencrypted (SSH tunneling optional).

## Outcome
- Successfully switched to X11 and set up TightVNC with XFCE for remote access.
- Persistent VNC achieved after resolving manual instance conflict.
- Browser functionality (e.g., Firefox) hit Snap-related snags; user shifted focus to Opera GX and VS Code.

