#!/usr/bin/env python3
"""
Pre-warm Firebase connection before starting main script.
Run this 5 minutes before your mission to establish Firebase connection.
"""
import firebase_admin
from firebase_admin import credentials, db
import time
import os

DRONE_ID = "Victoris"
CREDENTIALS_PATH = os.path.join(os.path.dirname(__file__), "..", "serviceAccountKey.json")
DATABASE_URL = "https://jech-flyt-default-rtdb.asia-southeast1.firebasedatabase.app"

print("[PREWARM] Starting Firebase pre-connection...")
start = time.time()

try:
    cred = credentials.Certificate(CREDENTIALS_PATH)
    firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
    print(f"[PREWARM] Firebase app initialized in {time.time() - start:.1f}s")
    
    # Test connection
    ref = db.reference(f'missions/{DRONE_ID}/status')
    ref.update({'pre_warmed': int(time.time() * 1000)})
    print(f"[PREWARM] Connection established in {time.time() - start:.1f}s")
    print("[PREWARM] Firebase is ready! You can now start main.py")
    
    # Keep connection alive
    print("[PREWARM] Keeping connection alive for 30 seconds...")
    time.sleep(30)
    
except Exception as e:
    print(f"[PREWARM] Error: {e}")
