# SIM7670G + Raspberry Pi 5 Recovery & Connection Guide

## Updated Notes

* Preferred final setup: `wlan0` primary, `ppp0` LTE backup, `usb0` ignored.
* `usb0` RNDIS mode was tested and detected, but no working routed internet on this firmware/config.
* Remove any manually-added `usb0` default route after testing.

## Purpose

Commands that were successful/helpful after reset to restore:

* WiFi primary internet
* SIM7670G LTE backup internet via PPP (`ppp0`)
* Modem detection/debugging

---

# 1. Basic Detection / Debugging

```bash
ls /dev/ttyACM*
lsusb
mmcli -L
nmcli connection show
nmcli device status
ip a
ip route
```

Useful kernel logs:

```bash
dmesg | grep -i -E "ttyACM|cdc|modem|usb"
```

ModemManager logs:

```bash
sudo journalctl -u ModemManager -n 100 --no-pager
```

---

# 2. Restart Services (helpful after reset)

```bash
sudo systemctl restart ModemManager
sudo systemctl restart NetworkManager
```

Check status:

```bash
systemctl status ModemManager --no-pager
```

---

# 3. Confirm AT Port (worked on ttyACM2)

```bash
sudo minicom -D /dev/ttyACM2
```

Inside minicom:

```text
AT
ATI
AT+CPIN?
AT+CSQ
```

Exit minicom:

```text
Ctrl+A then X
```

---

# 4. Restore SIM7670 LTE Connection (SUCCESSFUL)

Delete broken profile:

```bash
sudo nmcli connection delete sim7670
```

Create proper GSM profile:

```bash
sudo nmcli connection add type gsm ifname "*" con-name sim7670 apn airtelgprs.com
```

Set options:

```bash
sudo nmcli connection modify sim7670 gsm.number "*99#"
sudo nmcli connection modify sim7670 connection.autoconnect yes
sudo nmcli connection modify sim7670 ipv4.route-metric 200
```

Bring connection up:

```bash
sudo nmcli connection up sim7670
```

---

# 5. Set WiFi as Primary Internet (SUCCESSFUL)

```bash
sudo nmcli connection modify "netplan-wlan0-MIBEE" ipv4.route-metric 100
```

If your WiFi profile name changes, check with:

```bash
nmcli connection show
```

---

# 6. Keep CUAV Ethernet From Becoming Default Route

```bash
sudo nmcli connection modify cuav-eth ipv4.never-default yes
sudo nmcli connection modify cuav-eth ipv4.route-metric 900
```

---

# 7. Block usb0 Default Route Permanently

Remove current bad route:

```bash
sudo ip route del default via 192.168.0.1 dev usb0
```

Ignore usb0 in NetworkManager:

```bash
sudo mkdir -p /etc/NetworkManager/conf.d
sudo nano /etc/NetworkManager/conf.d/99-ignore-usb0.conf
```

Paste:

```ini
[keyfile]
unmanaged-devices=interface-name:usb0
```

Apply:

```bash
sudo systemctl restart NetworkManager
```

---

# 8. Verify Final Working State

```bash
mmcli -L
nmcli device status
ip a
ip route
```

Expected:

```text
default via wlan0 metric 100
default dev ppp0 metric 200
```

---

# 9. Test LTE Specifically

```bash
ping -I ppp0 -c 4 8.8.8.8
ping -I ppp0 -c 4 google.com
```

---

# 10. Quick Recovery Sequence (after future reboot)

```bash
sudo systemctl restart ModemManager
sudo systemctl restart NetworkManager
sudo nmcli connection up sim7670
ip route
```

---

# 11. Arducam 64MP (OwlEye) Camera Configuration for RPi 5

### Hardware Connection
1. Connect the camera to one of the dual MIPI CSI ports on the Raspberry Pi 5 using the 15-pin to 22-pin FPC cable.
2. Ensure the black tab on the cable faces the HDMI ports, and metal contacts are properly seated.

### Software Configuration (Bookworm or Trixie OS)
1. Open the Raspberry Pi boot configuration file:
   ```bash
   sudo nano /boot/firmware/config.txt
   ```
2. Disable auto-detection by changing:
   ```text
   camera_auto_detect=0
   ```
3. Locate the `[all]` section and add the appropriate overlay line beneath it to enable the camera:
   * **For CAM1 port (default)**:
     ```text
     dtoverlay=ov64a40,link-frequency=360000000
     ```
   * **For CAM0 port**:
     ```text
     dtoverlay=ov64a40,cam0,link-frequency=360000000
     ```
4. Save the file and reboot:
   ```bash
   sudo reboot
   ```

### Camera Test Commands
* **List available cameras**:
  ```bash
  rpicam-still --list-cameras
  ```
* **Start a live preview**:
  ```bash
  rpicam-still -t 0
  ```
* **Preview with Continuous Autofocus**:
  ```bash
  rpicam-still -t 0 --autofocus-mode continuous
  ```

---

# 12. Notes

* `ttyACM0` = NetworkManager modem control path
* `usb0` = modem RNDIS interface (ignored in final setup)
* `ttyACM2` = AT command port confirmed working
* `ppp0` = LTE internet interface
* `wlan0` = primary WiFi internet
* `eth0` = CUAV local ethernet link

---

# Quectel EC200U Internet Setup on Raspberry Pi 5

## Overview

This guide describes how to:

* Connect a Quectel EC200U modem to a Raspberry Pi 5 via USB
* Verify modem detection
* Confirm internet connectivity
* Configure Wi-Fi as the primary connection
* Configure EC200U cellular data as an automatic failover connection

---

# Hardware

## Required Components

* Raspberry Pi 5
* Quectel EC200U Modem
* Active SIM card with mobile data
* USB-C cable
* Wi-Fi connection (optional)

---

# 1. Connect the Modem

Connect the EC200U USB-C port directly to the Raspberry Pi.

Verify detection:

```bash
lsusb
```

Expected output:

```text
Bus XXX Device XXX: ID 2c7c:0901 Quectel Wireless Solutions Co., Ltd.
```

---

# 2. Verify Serial Ports

Check that Linux created modem ports:

```bash
dmesg | grep ttyUSB
```

Expected:

```text
ttyUSB0
ttyUSB1
ttyUSB2
ttyUSB3
ttyUSB4
ttyUSB5
ttyUSB6
```

Alternatively:

```bash
ls /dev/ttyUSB*
```

---

# 3. Verify USB Network Interface

Check available network interfaces:

```bash
ip link
```

Expected:

```text
usb0
```

Example:

```text
4: usb0: <BROADCAST,MULTICAST,UP,LOWER_UP>
```

---

# 4. Check IP Address

Display interface details:

```bash
ip addr show usb0
```

Expected:

```text
inet 100.x.x.x/24
```

Example:

```text
inet 100.93.200.194/24
```

This indicates the modem has assigned an IP address to the Raspberry Pi.

---

# 5. Verify Routing

Check routing table:

```bash
ip route
```

Example:

```text
default via 100.93.200.1 dev usb0 metric 100
default via 10.19.244.195 dev wlan0 metric 600
```

Lower metric values have higher priority.

In the example above:

* Cellular (usb0) is primary
* Wi-Fi (wlan0) is secondary

---

# 6. Verify Internet Connectivity

Ping a public DNS server:

```bash
ping 8.8.8.8
```

Expected:

```text
64 bytes from 8.8.8.8
```

Check public IP:

```bash
curl ifconfig.me
```

or

```bash
curl ipinfo.io/ip
```

---

# 7. Verify Modem Using AT Commands

Install Minicom:

```bash
sudo apt update
sudo apt install minicom -y
```

Open modem AT port:

```bash
sudo minicom -D /dev/ttyUSB2
```

Test communication:

```text
AT
```

Expected:

```text
OK
```

---

# Useful AT Commands

## SIM Status

```text
AT+CPIN?
```

Expected:

```text
+CPIN: READY
```

---

## Signal Quality

```text
AT+CSQ
```

Example:

```text
+CSQ: 20,99
```

Signal Strength Guide:

| CSQ   | Quality |
| ----- | ------- |
| 0-9   | Poor    |
| 10-19 | Fair    |
| 20-31 | Good    |

---

## Network Registration

```text
AT+CEREG?
```

Expected:

```text
+CEREG: 0,1
```

or

```text
+CEREG: 0,5
```

---

## Check APN

```text
AT+CGDCONT?
```

---

# 8. Configure Wi-Fi as Primary Connection

Current connections:

```bash
nmcli connection show
```

Example:

```text
NAME                TYPE
MIBEE               wifi
Wired connection 1  ethernet
```

Where:

* MIBEE = Wi-Fi
* Wired connection 1 = EC200U (usb0)

---

## Set Wi-Fi Priority

```bash
sudo nmcli connection modify "MIBEE" ipv4.route-metric 100
sudo nmcli connection modify "MIBEE" ipv6.route-metric 100
```

---

## Set Cellular Backup Priority

```bash
sudo nmcli connection modify "Wired connection 1" ipv4.route-metric 600
sudo nmcli connection modify "Wired connection 1" ipv6.route-metric 600
```

---

## Reconnect Interfaces

```bash
sudo nmcli connection down "MIBEE"
sudo nmcli connection up "MIBEE"

sudo nmcli connection down "Wired connection 1"
sudo nmcli connection up "Wired connection 1"
```

---

## Verify Routing

```bash
ip route
```

Expected:

```text
default via 10.19.244.195 dev wlan0 metric 100
default via 100.93.200.1 dev usb0 metric 600
```

Meaning:

* Wi-Fi is primary
* Cellular is fallback

---

# 9. Test Automatic Failover

Verify active route:

```bash
ip route get 8.8.8.8
```

Expected:

```text
dev wlan0
```

Disable Wi-Fi:

```bash
sudo ip link set wlan0 down
```

Check route again:

```bash
ip route get 8.8.8.8
```

Expected:

```text
dev usb0
```

Internet should continue through EC200U.

Re-enable Wi-Fi:

```bash
sudo ip link set wlan0 up
```

Wi-Fi should automatically become the preferred route again.

---

# 10. Understanding Public IP vs Modem IP

Example:

```text
usb0 IP      : 100.93.200.194
Public IP    : 152.xx.xx.xx
```

This indicates the modem is behind Carrier Grade NAT (CGNAT).

Because of CGNAT:

* Outgoing internet works normally
* Direct SSH from the Internet usually does not work
* Port forwarding is generally unavailable

Recommended solutions:

* Tailscale
* WireGuard VPN
* Cloudflare Tunnel

---

# Troubleshooting

## No usb0 Interface

```bash
ip link
```

If usb0 is missing:

```bash
sudo reboot
```

Check USB cable and modem power.

---

## No IP Address on usb0

```bash
ip addr show usb0
```

Verify:

```text
AT+CPIN?
AT+CEREG?
AT+CGATT?
```

---

## No Internet

Check routing:

```bash
ip route
```

Ping:

```bash
ping 8.8.8.8
```

Verify APN settings.

---

# Useful Commands Summary

```bash
lsusb
dmesg | grep ttyUSB
ip link
ip addr show usb0
ip route
nmcli connection show
curl ifconfig.me
ping 8.8.8.8
sudo minicom -D /dev/ttyUSB2
```

---

# Result

Final configuration:

* Wi-Fi is the preferred internet connection.
* EC200U automatically provides internet if Wi-Fi disconnects.
* No manual switching required.
* Settings persist across reboots.
