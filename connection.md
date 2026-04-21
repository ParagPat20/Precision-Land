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

# 11. Notes

* `ttyACM0` = NetworkManager modem control path
* `usb0` = modem RNDIS interface (ignored in final setup)
  n- `ttyACM2` = AT command port confirmed working
* `ppp0` = LTE internet interface
* `wlan0` = primary WiFi internet
* `eth0` = CUAV local ethernet link
