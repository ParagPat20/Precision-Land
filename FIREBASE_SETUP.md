# Firebase Realtime Database Setup Guide

## Firebase RTDB Rules Configuration

To enable GPS/telemetry data and mission commands in Firebase Realtime Database, you need to configure the database rules.

### Step 1: Access Firebase Console

1. Go to [Firebase Console](https://console.firebase.google.com/)
2. Select your project: **jech-flyt**
3. Navigate to **Realtime Database** in the left sidebar
4. Click on the **Rules** tab

### Step 2: Apply Database Rules

Copy and paste the following rules into the Firebase Console Rules editor:

```json
{
  "rules": {
    "missions": {
      "$droneId": {
        "active_command": {
          ".read": "auth != null",
          ".write": "auth != null",
          "status": {
            ".validate": "newData.isString() && (newData.val() == 'PENDING' || newData.val() == 'IN_PROGRESS' || newData.val() == 'COMPLETED' || newData.val() == 'FAILED' || newData.val() == 'ABORTED' || newData.val() == 'ABORT_REQUESTED' || newData.val() == 'EXPIRED_STALE' || newData.val() == 'CANCELLED')"
          },
          "telemetry": {
            ".read": "auth != null",
            ".write": "auth != null",
            "lat": {
              ".validate": "newData.isNumber()"
            },
            "lng": {
              ".validate": "newData.isNumber()"
            },
            "alt": {
              ".validate": "newData.isNumber()"
            },
            "heading": {
              ".validate": "newData.isNumber()"
            },
            "mode": {
              ".validate": "newData.isString()"
            },
            "updated_at": {
              ".validate": "newData.isNumber()"
            }
          },
          "payload": {
            ".read": "auth != null",
            ".write": "auth != null"
          },
          "id": {
            ".validate": "newData.isString()"
          },
          "type": {
            ".validate": "newData.isString()"
          },
          "timestamp": {
            ".validate": "newData.isNumber()"
          },
          "sender_uid": {
            ".validate": "newData.isString()"
          },
          "sender_email": {
            ".validate": "newData.isString()"
          },
          "started_at": {
            ".validate": "newData.isNumber()"
          },
          "completed_at": {
            ".validate": "newData.isNumber()"
          },
          "failed_at": {
            ".validate": "newData.isNumber()"
          },
          "aborted_at": {
            ".validate": "newData.isNumber()"
          }
        }
      }
    }
  }
}
```

### Step 3: Publish Rules

1. Click **Publish** button at the top
2. Confirm the changes

### Step 4: Verify Authentication

Make sure Firebase Authentication is enabled:
1. Go to **Authentication** in Firebase Console
2. Ensure **Sign-in method** includes at least one provider (e.g., Email/Password, Google)
3. Users must be authenticated to read/write mission data

## What These Rules Do

- **Authentication Required**: Only authenticated users can read/write mission data
- **Telemetry Validation**: Ensures GPS coordinates (lat, lng), altitude, heading, and mode are valid numbers/strings
- **Status Validation**: Only allows valid mission status values
- **Structure Validation**: Ensures data structure matches expected format

## Testing

After applying rules, test by:
1. Sending a mission command from the Flutter app
2. Checking Firebase Console → Realtime Database → `missions/Victoris/active_command`
3. Verifying telemetry data appears under `telemetry` when mission is IN_PROGRESS

## Troubleshooting

If GPS data is not appearing:
1. Check Firebase Console → Realtime Database → Rules tab to ensure rules are published
2. Verify authentication is working in your Flutter app
3. Check Python script logs for "Telemetry Error" messages
4. Ensure mission status is set to "IN_PROGRESS" before telemetry is sent
