# Jetson Reinstall and Project Bring-Up (Ordered Runbook)

This guide follows your required order exactly and is written as a one-pass checklist.

Official download pages for Orin Nano (JetPack 6):
- JetPack archive: https://developer.nvidia.com/embedded/jetpack-archive
- JetPack 6.2.1 page: https://developer.nvidia.com/embedded/jetpack-sdk-621
- JetPack 6.2 page: https://developer.nvidia.com/embedded/jetpack-sdk-62
- Orin Nano getting started: https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit

## 1. Flash Ubuntu 22 and set device identity

Target identity:
- Hostname: `f1tenth`
- Username: `f1tenth`
- Password: `admin321`

For Jetson Orin Nano Dev Kit, the easiest and most reliable method is SD-card flashing.

### Option A (recommended): Flash SD card image (simplest)

1. Download the JetPack 6.x SD-card image for Jetson Orin Nano Dev Kit.
2. On your Ubuntu PC, write it to microSD (64 GB+ recommended):

```bash
# Example using balenaEtcher (GUI) or command line:
unzip -p jetson-orin-nano-jp6-sd-card-image.zip | sudo dd of=/dev/sdX bs=16M oflag=direct status=progress
sync
```

3. Insert microSD into Jetson, connect display/keyboard, and boot.
4. In first-boot wizard, set:
  - Username: `f1tenth`
  - Password: `admin321`
  - Hostname: `f1tenth`

Optional (headless/server-style, no GUI login after setup):

```bash
sudo systemctl set-default multi-user.target
sudo systemctl disable --now gdm3
sudo reboot
```

If you later want desktop back:

```bash
sudo systemctl set-default graphical.target
sudo systemctl enable --now gdm3
```

### Option B: SDK Manager over USB (if you prefer)

If SDK Manager shows "Could not detect a board", it is usually recovery-mode or cable/port related.

Checklist:

1. Use a USB-C **data** cable (not charge-only), directly to PC (no hub).
2. Put Jetson in force-recovery mode:
  - Power off Jetson.
  - Hold recovery button (or short FC REC to GND on button header).
  - Apply power while holding recovery, then release.
3. Verify detection on host:

```bash
lsusb | grep -i nvidia
```

You should see an NVIDIA USB device (Vendor ID `0955`).

4. Re-open SDK Manager and select JetPack 6.x for Orin Nano.

If SDK compatibility still blocks install on host Ubuntu 24.04, use Option A (SD image), then install remaining packages directly on-device.

### If ping works but SSH is refused (common on fresh image)

Symptom:

```bash
ping 192.168.55.1   # works
ssh nvidia@192.168.55.1   # connection refused
```

Meaning: Jetson has booted, but first-boot setup (`oem-config`) is not completed yet, so SSH is not available.

To complete first boot, you need one of these:
- A display connection (DisplayPort or DP-to-HDMI adapter) + keyboard
- A USB-to-TTL serial cable (advanced headless method)

After first-boot setup is completed, SSH over `192.168.55.1` becomes available.

### No display/adapter available: offline SD-card preseed workaround

If you have no monitor and cannot complete `oem-config`, you can pre-create the user from your Ubuntu PC.

1. Insert SD card into PC.
2. Mount rootfs partition (APP):

```bash
sudo mkdir -p /mnt/jetson-root
sudo mount /dev/sdb1 /mnt/jetson-root
export JROOT=/mnt/jetson-root
```

3. Create user and set password hash:

```bash
sudo useradd --root "$JROOT" -m -s /bin/bash -G sudo f1tenth || true
HASH=$(openssl passwd -6 'admin321')
sudo usermod --root "$JROOT" -p "$HASH" f1tenth
sudo grep '^f1tenth:' "$JROOT/etc/shadow"
```

4. Set hostname and force headless target:

```bash
echo f1tenth | sudo tee "$JROOT/etc/hostname"
sudo sed -i 's/^127.0.1.1.*/127.0.1.1\tf1tenth/' "$JROOT/etc/hosts"
sudo rm -f "$JROOT/etc/systemd/system/default.target"
sudo ln -s /lib/systemd/system/multi-user.target "$JROOT/etc/systemd/system/default.target"
```

5. Enable USB serial getty and SSH (if installed in image):

```bash
sudo mkdir -p "$JROOT/etc/systemd/system/getty.target.wants"
sudo ln -sf /lib/systemd/system/serial-getty@.service "$JROOT/etc/systemd/system/getty.target.wants/serial-getty@ttyGS0.service"

if [ -f "$JROOT/lib/systemd/system/ssh.service" ]; then
  sudo mkdir -p "$JROOT/etc/systemd/system/multi-user.target.wants"
  sudo ln -sf /lib/systemd/system/ssh.service "$JROOT/etc/systemd/system/multi-user.target.wants/ssh.service"
fi
```

6. Unmount and boot Jetson:

```bash
sync
sudo umount "$JROOT"
```

Then test:

```bash
ping -c 3 192.168.55.1
ssh f1tenth@192.168.55.1
```

Quick verification on Jetson after first boot:

```bash
hostnamectl
whoami
```

If needed, force hostname:

```bash
sudo hostnamectl set-hostname f1tenth
```

## 2. Set internet region and Wi-Fi priority

Requirement:
- Country/regulatory domain: DK
- Priority 1 hotspot: `net1tenth` / `admin321`
- Priority 2 hidden Wi-Fi: `ProjektNet` / `RobotRocks`

### 2.1 Set DK regulatory domain

Apply now:

```bash
sudo iw reg set DK
iw reg get
```

Persist across boots (systemd oneshot):

```bash
cat <<'EOF' | sudo tee /etc/systemd/system/set-regdom-dk.service
[Unit]
Description=Set WiFi regulatory domain to DK
After=network-pre.target
Before=network.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/iw reg set DK
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now set-regdom-dk.service
```

### 2.2 Configure Wi-Fi connections with priority

Create priority 1 hotspot profile:

```bash
sudo nmcli connection add type wifi ifname wlan0 con-name net1tenth ssid net1tenth
sudo nmcli connection modify net1tenth \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk admin321 \
  connection.autoconnect yes \
  connection.autoconnect-priority 100 \
  ipv4.method auto ipv6.method ignore
```

Create priority 2 hidden-network profile:

```bash
sudo nmcli connection add type wifi ifname wlan0 con-name ProjektNet ssid ProjektNet
sudo nmcli connection modify ProjektNet \
  802-11-wireless.hidden yes \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk RobotRocks \
  connection.autoconnect yes \
  connection.autoconnect-priority 50 \
  ipv4.method auto ipv6.method ignore
```

Bring up preferred network immediately:

```bash
sudo nmcli connection up net1tenth || sudo nmcli connection up ProjektNet
```

Verify priorities:

```bash
nmcli -f NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show
```

## 3. Install NVIDIA stack for CUDA workloads

On Jetson, CUDA drivers/toolchain come from JetPack components.

```bash
sudo apt update
sudo apt install -y nvidia-jetpack
```

Optional environment setup:

```bash
grep -q '/usr/local/cuda/bin' ~/.bashrc || echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
grep -q '/usr/local/cuda/lib64' ~/.bashrc || echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

Verification:

```bash
dpkg -l | grep -E 'nvidia-l4t-core|nvidia-jetpack|cuda-toolkit' || true
/usr/local/cuda/bin/nvcc --version
```

Note: `nvidia-smi` is typically not available on Jetson; use `nvcc --version`, `tegrastats`, and package checks instead.

## 4. Configure GitHub account + SSH key

You provided this known fingerprint:
- `SHA256:PC1maxRG5rqK4Ss/nUJCEZBOQNQLPif8yleA2GkKdD8`

Important: a fingerprint alone cannot recreate a private key.

### 4.1 Set Git identity

```bash
git config --global user.name "Pascal Bjorstorp"
git config --global user.email "<your_github_email>"
```

### 4.2 Check whether the old key is already present

```bash
ls -la ~/.ssh
ssh-keygen -lf ~/.ssh/id_ed25519.pub
```

If the shown fingerprint matches `SHA256:PC1maxRG5rqK4Ss/nUJCEZBOQNQLPif8yleA2GkKdD8`, reuse that key.

Enable agent and load key:

```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
ssh-add -l
```

If key is missing or fingerprint does not match, generate a new key:

```bash
ssh-keygen -t ed25519 -C "<your_github_email>"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub
```

Add the public key to GitHub, then test:

```bash
ssh -T git@github.com
```

## 5. Install ROS2 Humble, clone repo, and build until green

### 5.1 Install ROS2 Humble + tools

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg2 lsb-release
sudo add-apt-repository -y universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  git build-essential cmake

grep -q '/opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

Initialize rosdep:

```bash
sudo rosdep init || true
rosdep update
```

### 5.2 Fetch repository

```bash
mkdir -p ~/BachelorProject
cd ~/BachelorProject
git clone git@github.com:PascalBjorstorp/BachelorProject.git .
```

### 5.3 Install dependencies and build

Install available dependencies first:

```bash
cd ~/BachelorProject
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

Build:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If build fails due to missing ROS package `xyz_pkg`, install and retry:

```bash
sudo apt update
sudo apt install -y ros-humble-xyz-pkg
```

Then rerun:

```bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Repeat until build succeeds.

Smoke check after success:

```bash
source install/setup.bash
ros2 pkg list | grep -E '^f1tenth_|^mpc_|^mpcc_'
```

---

## Fast execution checklist

- [ ] Step 1 complete (flash + user/host set)
- [ ] Step 2 complete (DK + Wi-Fi priorities)
- [ ] Step 3 complete (CUDA/NVIDIA stack verified)
- [ ] Step 4 complete (GitHub SSH access verified)
- [ ] Step 5 complete (ROS2 + clone + successful colcon build)
