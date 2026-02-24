#!/bin/bash
# Network Latency Diagnostic Script for Firebase Connection

echo "=========================================="
echo "Network Latency Diagnostic"
echo "=========================================="
echo ""

# Test 1: Basic connectivity
echo "[1/6] Testing basic internet connectivity..."
if ping -c 3 8.8.8.8 > /dev/null 2>&1; then
    echo "✓ Internet connection: OK"
else
    echo "✗ Internet connection: FAILED"
    exit 1
fi
echo ""

# Test 2: DNS resolution
echo "[2/6] Testing DNS resolution..."
if nslookup jech-flyt-default-rtdb.asia-southeast1.firebasedatabase.app > /dev/null 2>&1; then
    echo "✓ DNS resolution: OK"
    FIREBASE_IP=$(nslookup jech-flyt-default-rtdb.asia-southeast1.firebasedatabase.app | grep -A1 "Name:" | grep "Address:" | awk '{print $2}' | head -1)
    echo "  Firebase IP: $FIREBASE_IP"
else
    echo "✗ DNS resolution: FAILED"
    echo "  Run: sudo bash -c 'echo \"nameserver 8.8.8.8\" > /etc/resolv.conf'"
fi
echo ""

# Test 3: Latency to Google DNS
echo "[3/6] Testing latency to Google DNS (8.8.8.8)..."
ping -c 10 -q 8.8.8.8 | tail -n 2
echo ""

# Test 4: Latency to Firebase
echo "[4/6] Testing latency to Firebase database..."
ping -c 10 -q jech-flyt-default-rtdb.asia-southeast1.firebasedatabase.app 2>/dev/null | tail -n 2
echo ""

# Test 5: HTTP connection timing
echo "[5/6] Testing HTTP connection to Firebase..."
curl -w "\n  DNS Lookup:     %{time_namelookup}s\n  TCP Connect:    %{time_connect}s\n  TLS Handshake:  %{time_appconnect}s\n  Total Time:     %{time_total}s\n" \
  -o /dev/null -s https://jech-flyt-default-rtdb.asia-southeast1.firebasedatabase.app/.json 2>/dev/null

if [ $? -eq 0 ]; then
    echo "✓ HTTP connection: OK"
else
    echo "✗ HTTP connection: FAILED"
fi
echo ""

# Test 6: Packet loss test
echo "[6/6] Testing packet loss (100 pings)..."
PACKET_LOSS=$(ping -c 100 -q 8.8.8.8 2>/dev/null | grep "packet loss" | awk '{print $6}')
echo "  Packet loss: $PACKET_LOSS"
echo ""

echo "=========================================="
echo "Diagnostic Summary"
echo "=========================================="
echo ""
echo "Expected values for India → Singapore:"
echo "  Latency (avg):    50-100ms (Good), 100-200ms (OK), >200ms (Slow)"
echo "  Packet loss:      0-2% (Good), 2-5% (OK), >5% (Poor)"
echo "  TLS Handshake:    <1s (Good), 1-3s (OK), >3s (Slow)"
echo ""
echo "If your values are much worse than 'OK', this explains"
echo "the 4-minute Firebase connection delay."
echo ""
echo "Possible fixes:"
echo "  1. Use wired connection instead of WiFi"
echo "  2. Check DNS: cat /etc/resolv.conf (should have 8.8.8.8)"
echo "  3. Try different internet connection/ISP"
echo "  4. Check router/firewall settings"
