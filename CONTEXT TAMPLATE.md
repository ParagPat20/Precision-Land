# [PROJECT NAME] — Agent Context & Handoff

> Single source of truth for agents and humans.
> Read before touching anything. Update before leaving.
> Compatible with: Claude, Cursor, Windsurf, Copilot, Gemini, Codex, any agent or IDE.

---

# NAVIGATION

| Goal                              | Go to                        |
|-----------------------------------|------------------------------|
| Resume quickly / what's happening | HANDOFF STATE                |
| Which file owns what              | FILE MAP                     |
| Directory layout + versions       | WORKING DIRECTORY            |
| Firebase / DB / API paths         | DATA & PATHS                 |
| Build and ship                    | BUILD & DEPLOY               |
| Rules that must never break       | HARD RULES                   |
| Code patterns used here           | CODE PATTERNS                |
| Why we made a choice              | DECISION LOG                 |
| What has been tested              | VALIDATION LOG               |
| Open tasks                        | BACKLOG                      |
| Hardware pins (embedded projects) | HARDWARE                     |
| Security notes                    | SECURITY                     |

---

# AGENT PROTOCOL

## Session START — do in order
1. Read HANDOFF STATE. Understand status, blockers, and the single next step.
2. Run: ls -la [relevant dirs] and git log -5 --oneline to verify actual current state.
3. Check last-modified times: git log --diff-filter=M --name-only -10
4. Read FILE MAP for areas you will touch.
5. If touching firmware/hardware — verify HARDWARE section AND actual config file before editing.
6. Never trust stale notes. Always view the actual file first.

## Session END — mandatory before closing
1. Rewrite HANDOFF STATE: date+time, summary, blockers, next step.
2. Update WORKING DIRECTORY if files were added, renamed, or status changed.
3. If contracts/paths/pins/APIs changed — update DATA & PATHS or HARDWARE.
4. Add a row to DECISION LOG for every non-obvious choice made.
5. Append a row to VALIDATION LOG if build ran or meaningful tests done.
6. Update CONTEXT FILE LOG at the bottom with what you changed in this file.

## What agents must never do
- Never commit secrets, API keys, service accounts, or private credentials.
- Never write AI/agent/IDE/tool name in commit messages or code comments.
- Never mark build as passing without actually running the build command.
- Never leave HANDOFF STATE blank or with stale date.
- Never silently skip a section update — note "no change" if nothing changed.

---

# HANDOFF STATE

> This section is the most important. Update it every single session.

| Field           | Value                                                   |
|-----------------|---------------------------------------------------------|
| Last updated    | YYYY-MM-DD HH:MM (timezone)                             |
| Last updated by | [agent name / human name / IDE used]                    |
| Build status    | passing / failing / untested                            |
| Active branch   | [branch] — confirm with: git branch                     |
| Last commit     | [run: git log -1 --oneline — paste result here]         |
| Blockers        | none / [describe clearly]                               |
| Next step       | [ONE concrete action — enough to resume cold]           |

## What was done this session
[3-7 bullets. File changed → what behavior changed. Be specific.]
- [e.g. RaceControl.tsx: added REDEEM COUPON card with border + return-icon lookup]
- [e.g. firestore.rules: added write access for redemption_qr_tokens for admin role]

## What reliably works
[Update as features stabilize. Note the date each was last confirmed.]
- [Feature A — confirmed working YYYY-MM-DD]
- [Feature B — confirmed working YYYY-MM-DD]

## Known issues / caveats
[Partial implementations, known bugs, things that look done but aren't.]
- [e.g. Serial SAVE/LOAD commands don't map to valid menu entries — does nothing]
- [e.g. IMU and MAVLink both write ekf_yaw — no selection rule, last-write wins]

---

# PROJECT SNAPSHOT

Purpose: [one sentence — what this project does and for whom]

Stack:
- Frontend / App: [e.g. React 19, Vite 7, TypeScript, HashRouter]
- Backend / DB: [e.g. Firebase Auth + Firestore + Realtime Database]
- Firmware: [e.g. ESP32, FreeRTOS, Arduino IDE 2.x]
- Hosting / Deploy: [e.g. GitHub Pages via npm run deploy to gh-pages branch]
- UI lib: [e.g. LVGL 9.5.0, TFT_eSPI 2.5.43]
- Agents / IDEs used on this project: [e.g. Claude Code, Cursor, Windsurf]

Component version compatibility:
| Component      | Version    | Works with         | Status      | Notes / issues                   |
|----------------|------------|--------------------|-------------|----------------------------------|
| Web app        | v[x]       | Firmware V[x]      | active      | Uses hardware/ RTDB paths        |
| Firmware V1    | ESP_R4T_V1 | Web v1 only        | deprecated  | LED matrix bug, do not flash     |
| Firmware V2    | ESP_R4T_V2 | Web v2 (current)   | active      | Current integration target       |
| [Library X]    | v[x]       | [platform]         | active      | [e.g. lv_conf.h must be outside] |

---

# WORKING DIRECTORY

> Agent: on session start, run the commands below to get actual last-modified times.
> Do not rely on this table alone — verify with the filesystem.
>
> git log --diff-filter=M --name-only -10
> ls -lt src/ (or relevant dir)

```
[project-root]/
├── src/ (or web/)           # Web / app source — actively edited
├── firmware_v2/             # ACTIVE firmware — use this for new work
├── firmware_v1/             # Legacy firmware — keep for reference, do not flash
├── backup/                  # Old snapshots — do not integrate without review
├── docs/ (or .agent/)       # Agent docs, workflows, this file
├── dist/                    # Build output — gitignored, never edit manually
├── public/                  # Static assets
├── config.h / constants.ts  # Central config — all pins and constants live here
├── firestore.rules          # Security rules — deploy separately, not via npm deploy
└── context.md               # This file
```

File/folder status table — update when anything changes:
| Path                   | Status      | Last edited  | Who/agent    | Notes                                        |
|------------------------|-------------|--------------|--------------|----------------------------------------------|
| src/pages/Admin/       | active      | YYYY-MM-DD   | [name]       | RaceControl + Coupons tabs most recent       |
| firmware_v2/           | active      | YYYY-MM-DD   | [name]       | Current firmware — V1 is deprecated          |
| firmware_v1/           | deprecated  | YYYY-MM-DD   | [name]       | Reference only — has known LED matrix bug    |
| backup/                | archive     | YYYY-MM-DD   | [name]       | Pre-MAVLink-removal snapshot                 |
| firestore.rules        | active      | YYYY-MM-DD   | [name]       | Needs separate firebase deploy after changes |
| context.md             | active      | YYYY-MM-DD   | [name]       | This file                                    |
| [add row per area]     |             |              |              |                                              |

---

# FILE MAP

| Feature / Area            | Primary file(s)                                       | Notes                               |
|---------------------------|-------------------------------------------------------|-------------------------------------|
| App entry / routing       | src/main.tsx, src/App.tsx                             | HashRouter, SPA redirect logic      |
| Firebase init             | src/firebase.ts                                       | RTDB + Firestore clients            |
| Auth + role mirror        | src/context/AuthContext.tsx                           | Mirrors role to RTDB users/{uid}    |
| Live race UI              | src/pages/Admin/tabs/RaceControl.tsx                  | currentRace + hardware writes       |
| TV leaderboard            | src/pages/Leaderboard/TVBoardLive.tsx                 | Boot sequencing + listeners         |
| Coupons / desk redeem     | CouponManagement.tsx, useAdminDeskRedeem.ts           | Shared award/decline path           |
| Progression config        | src/context/ProgressionConfigContext.tsx              | Reads system/progression            |
| Progression math          | src/utils/progression.ts, raceResults.ts              |                                     |
| Dashboard rewards         | Dashboard/index.tsx, MilestoneRewardCard.tsx          | Congrats modal + redemption listener|
| Shared types              | src/types/index.ts                                    | UserProfile, RaceSession, tokens    |
| Constants / gates         | src/constants.ts                                      | SUPERADMIN_EMAIL lives here         |
| Hardware config (firmware)| config.h                                              | Pins, PID gains, task frequencies   |
| Shared state (firmware)   | shared_data.h / shared_data.cpp                       | All global structs + mutexes        |
| Navigation (firmware)     | task_navigation.cpp                                   | Wall-follow controller              |
| Mission manager (firmware)| task_mission_manager.cpp                              | Currently empty — see BACKLOG       |
| Mapping (firmware)        | task_ekf_debug.cpp                                    | Cell discretization + wall inference|
| Build config              | vite.config.ts / platformio.ini                       | manualChunks for firebase/vendor    |
| Security rules            | firestore.rules, database.rules.json                  | Deploy separately from web          |
| [add row per feature]     |                                                       |                                     |

---

# HARDWARE
> Delete this entire section if project has no firmware or embedded component.

Verify all pins in actual source (config.h or equivalent) before editing anything.
This table is baseline only — the .ino / .h file is always authoritative.

| Component          | Pin(s)          | Type          | Notes                               |
|--------------------|-----------------|---------------|-------------------------------------|
| Motor Left         | GPIO 26, 13, 15 | PWM/Digital   | TB6612FNG H-Bridge                  |
| Motor Right        | GPIO 25, 14, 12 | PWM/Digital   | TB6612FNG H-Bridge                  |
| Encoder Left       | GPIO 18, 19     | Interrupt     | Quadrature — TaskEncoder @ 200Hz    |
| Encoder Right      | GPIO 23, 27     | Interrupt     | Quadrature                          |
| IR Front           | GPIO 34         | ADC           | TaskDistanceSensors @ 50Hz          |
| IR Right           | GPIO 36         | ADC           |                                     |
| IR Left            | GPIO 39         | ADC           |                                     |
| IMU MPU6050        | SDA:21, SCL:22  | I2C           | Shares bus — needs g_mutex_i2c      |
| OLED SSD1306       | SDA:21, SCL:22  | I2C           | Shared bus with IMU                 |
| NeoPixel Matrix    | GPIO 5          | Digital       | 32x8                                |
| [add yours]        | GPIO xx         | [type]        | [note]                              |

Active firmware path: [path/to/firmware_v2/] — flash with [Arduino IDE / PlatformIO]
Legacy firmware path: [path/to/firmware_v1/] — deprecated, [reason]

RTDB paths used by firmware:
| Path                  | Direction       | Purpose                       |
|-----------------------|-----------------|-------------------------------|
| hardware/command      | web → firmware  | Start / reset commands        |
| hardware/status       | firmware → web  | Heartbeat                     |
| hardware/lapQueue     | firmware → web  | Detected laps                 |
| hardware/gateResults  | firmware → web  | Matrix scroll data            |
| hardware/config       | web → firmware  | debounce_ms, theme, dispMode  |
| gate/status           | optional        | May not exist — read independently, not in Promise.all |

---

# DATA & PATHS

Realtime Database (RTDB):
| Path                  | Purpose                                    |
|-----------------------|--------------------------------------------|
| currentRace           | Live race session — shape in raceResults.ts|
| users/{uid}           | Role + permissions mirror for RTDB rules   |
| system/config         | System-level config                        |
| hardware/*            | Firmware integration (see HARDWARE)        |
| [add paths]           |                                            |

Firestore:
| Collection / Path         | Purpose                                        |
|---------------------------|------------------------------------------------|
| users/{uid}               | App profile — source of truth                  |
| racer_stats/{uid}         | Public stats                                   |
| system/progression        | Admin-editable — read via ProgressionConfigProvider |
| redemption_qr_tokens/{id} | Coupon / QR token lifecycle                    |
| [add collections]         |                                                |

API endpoints (if applicable):
| Endpoint          | Method | Auth required | Purpose              |
|-------------------|--------|---------------|----------------------|
| /api/[route]      | POST   | yes / no      | [what it does]       |

EEPROM / NVS map (embedded only):
| Address / Key | Content                | Default            |
|---------------|------------------------|--------------------|
| 0 / magic     | Magic byte (0xAB)      | First-boot detect  |
| 1 / speed     | Speed index 0-2        | 1 (MED)            |
| 2 / jog       | Jog index 0-4          | 1 (500 steps)      |
| [add yours]   |                        |                    |

---

# BUILD & DEPLOY

| Command                               | What it does                               |
|---------------------------------------|--------------------------------------------|
| npm run dev                           | Dev server only — no full tsc check        |
| npm run build                         | tsc -b && vite build → dist/               |
| npm run preview                       | Serve dist/ locally                        |
| npm run deploy                        | build + push to gh-pages branch            |
| firebase deploy --only firestore:rules| Deploy Firestore rules — separate from web |
| git log -1 --oneline                  | Confirm active commit                      |
| [pio run -t upload / Arduino flash]   | Flash firmware — separate from web deploy  |

Deploy model:
| Artifact          | How it ships                                | Target              |
|-------------------|---------------------------------------------|---------------------|
| Web app           | npm run deploy → GitHub Pages               | gh-pages branch     |
| Firestore rules   | firebase deploy --only firestore:rules      | Firebase Console    |
| RTDB rules        | Firebase Console or CLI                     | Firebase project    |
| Firmware          | Arduino IDE / PlatformIO flash              | Physical device     |

Pre-ship checklist:
- [ ] npm run build passes (tsc + bundler — not dev server alone)
- [ ] If race / live logic changed: smoke-test Race Control + TV mode
- [ ] If Firestore rules changed: deploy rules and log date in VALIDATION LOG
- [ ] If firmware changed: verify pins match HARDWARE table, flash to test device first
- [ ] No secrets in commit

---

# HARD RULES

Rules that must never be broken by agents or humans.

## Never do
- No delay() inside FreeRTOS tasks — use vTaskDelayUntil or non-blocking state machines.
- No blocking mutex wait — always use timeout: xSemaphoreTake(mutex, pdMS_TO_TICKS(5)).
- No hardcoded constants outside config.h / constants.ts.
- No long-lived DB secrets or service accounts in firmware or git.
- No inline theme colors outside the designated theme file.
- No runtime heap allocation for LVGL objects — allocate once at setup only.
- No new pins assigned without checking HARDWARE table for conflicts.
- No Promise.all for independent RTDB reads that may fail separately — one denial blocks all.
- No AI/agent/IDE name in commit messages or code comments.
- No marking build passing without running the actual build command.
- [Add your project-specific rules]

## Always do
- Run npm run build (or platform equivalent) before marking a session done.
- Use tsc / type checking — dev server alone is not sufficient for type safety.
- Protect every shared struct read/write with mutex + timeout.
- Deploy Firestore rules separately after changing firestore.rules.
- Mirror role/permissions to RTDB users/{uid} when auth state changes.
- Use normalizeUserAge when merging UserProfile reads (age is int 5-100).
- Keep laps[] aligned with currentLap on any manual decrement.
- [Add your project-specific rules]

## Patterns to avoid
- Do not use Promise.all for hardware/status + gate/status — read independently.
- Do not rely on *.tmp or *_errors.txt scratch files for truth.
- Do not assume gh-pages is current after pushing feature branch — run deploy.
- Do not infer from MAVLink and IMU simultaneously without a yaw source policy.
- Do not use VSPI for TFT display if touch is already on VSPI.
- [Add your project-specific anti-patterns]

---

# CODE PATTERNS

Named patterns used in this codebase. Reference these instead of re-explaining each session.

## FreeRTOS Task Loop
```cpp
// Standard task loop — always yield, never block
void TaskExample(void *pvParams) {
  TickType_t xLastWake = xTaskGetTickCount();
  for (;;) {
    // work
    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(TASK_PERIOD_MS));
  }
}
```

## Mutex Data Access
```cpp
// Always lock with timeout — never block infinitely
if (xSemaphoreTake(g_mutex_data, pdMS_TO_TICKS(5)) == pdTRUE) {
  // read or write shared struct
  xSemaphoreGive(g_mutex_data);
}
```

## Admin-only Firestore Write
```ts
// Users never self-edit redemptionHistory — admin batch only
// See: src/utils/adminRedemptionResolve.ts + useAdminDeskRedeem.ts
```

## Touch Validation (LVGL / TFT projects)
```cpp
if (p.z < TOUCH_PRESSURE_MIN || p.z > TOUCH_PRESSURE_MAX) { yield(); return; }
x = constrain(map(p.x, 200, 3700, 1, SCREEN_WIDTH), 1, SCREEN_WIDTH);
y = constrain(map(p.y, 240, 3800, 1, SCREEN_HEIGHT), 1, SCREEN_HEIGHT);
```

## SPI Init Order (TFT + touch on separate buses)
```cpp
// Touch on VSPI first, then TFT on HSPI — order matters
touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
touchscreen.begin(touchscreenSPI);
tft.init();
```

[Add more patterns as they stabilize — short pointer to source file beats pasting full code]

---

# MODES & STATES
> Delete this section if project has no explicit state machine or run modes.

| Mode / State       | Trigger               | Behavior                                        |
|--------------------|-----------------------|-------------------------------------------------|
| IDLE               | default / stop cmd    | No motion, waiting                              |
| MODE_RECORD        | UI explore start      | Wall-follow + live map building                 |
| MODE_FAST          | UI load + fast start  | Replay saved path at higher speed               |
| NAV_MOVE_FORWARD   | mission manager       | Active wall-follow / cell traversal             |
| NAV_TURN_LEFT/RIGHT| mission manager       | Yaw-based turn — wall-follow paused during turn |
| NAV_STOP           | any stop command      | Zero motor commands, wait                       |
| [add yours]        |                       |                                                 |

Serial / launch commands:
| Command             | Effect                                          |
|---------------------|-------------------------------------------------|
| start / explore     | Begin exploration                               |
| stop                | Halt all motion                                 |
| rwf / rightwall     | Right-wall-follow mode                          |
| lwf / leftwall      | Left-wall-follow mode (mirror of RWF)           |
| yaw / resetyaw      | Re-zero yaw via yaw_offset_rad                  |
| freerun             | Reactive center-follow in cm                    |
| cell                | Single-cell walk                                |
| [add yours]         |                                                 |

UI state machine rules (if applicable):
- ui_active_process: 0=IDLE, 1=HOME, 2=DOCK, 3=CALB
- +/- jog buttons blocked when ui_active_process != 0
- Pressing +/- during IDLE switches HOME button to STOP (red)
- Clicking STOP calls stop function and reverts to HOME (green)

---

# SECURITY

- Web client Firebase keys are normal to expose — security is enforced by rules, not key secrecy.
- Firestore rules source: firestore.rules — must be deployed separately. Web deploy does not update rules.
- RTDB rules source: database.rules.json — deploy via Firebase Console or CLI.
- SUPERADMIN_EMAIL is a privileged bootstrap constant — guard the account, coordinate any changes.
- Firmware: avoid long-lived DB secrets. Use strict RTDB rules + Auth. Rotate anything exposed.
- Service account JSON: never commit, never paste in chat, never log to serial.
- hardware/* and gate/* reads require signed-in users for Race Control hardware verification.
- [Add project-specific security notes]

---

# DECISION LOG

Append only. Never delete rows.

| Date       | Decision                                       | Why                                                      |
|------------|------------------------------------------------|----------------------------------------------------------|
| YYYY-MM-DD | HashRouter over BrowserRouter                  | GitHub Pages has no history-API fallback                 |
| YYYY-MM-DD | Firestore + RTDB split                         | Profiles need queries; race needs sub-second sync        |
| YYYY-MM-DD | Decouple gate/status from hardware/status      | V2 only writes hardware/ — Promise.all fails when gate/* absent |
| YYYY-MM-DD | laps[] aligned with currentLap on decrement    | Prevents next lap index jumping after manual reset       |
| YYYY-MM-DD | Touch on dedicated VSPI bus                    | Prevents SPI conflict with TFT on HSPI                   |
| YYYY-MM-DD | EEPROM magic byte 0xAB                         | Detect first boot vs saved settings                      |
| YYYY-MM-DD | Single context.md handoff file                 | One place for agents and humans — no scattered docs      |
| YYYY-MM-DD | [your decision]                                | [reason]                                                 |

---

# VALIDATION LOG

| Date       | Who / Agent | Check                       | Result  | Notes                              |
|------------|-------------|-----------------------------|---------|------------------------------------|
| YYYY-MM-DD | [name]      | npm run build               | pass    |                                    |
| YYYY-MM-DD | [name]      | Race Control smoke test     | pass    | start/stop/lap tested in dev       |
| YYYY-MM-DD | [name]      | Firmware flash V2           | pass    | Beam sensor + matrix display ok    |
| YYYY-MM-DD | [name]      | EEPROM persistence          | pass    | Settings survive reboot            |
| YYYY-MM-DD | [name]      | [test]                      | fail    | [what broke and where]             |

---

# BACKLOG

| # | Area         | Task                                              | Status   | Priority |
|---|--------------|---------------------------------------------------|----------|----------|
| 1 | Firmware     | Single yaw source policy — IMU vs MAVLink         | open     | high     |
| 2 | Firmware     | Implement TaskMissionManager with flood-fill       | deferred | high     |
| 3 | Firmware     | Unify IR thresholds between control and mapping   | open     | medium   |
| 4 | Web          | Serial telemetry parsing — incoming → UI labels   | open     | medium   |
| 5 | Web          | GitHub Actions CI for npm run build on PR         | deferred | low      |
| 6 | Embedded HMI | SPI DMA optimization for display performance      | deferred | low      |
| 7 | [add yours]  |                                                   | open     |          |

---

# GLOSSARY
> Delete if project is small or terms are obvious.

| Term                 | Meaning                                                  |
|----------------------|----------------------------------------------------------|
| RTDB                 | Firebase Realtime Database                               |
| currentRace          | RTDB node for the live race session                      |
| gh-pages             | Git branch holding only built static assets for hosting  |
| redemptionHistory    | Append-only list on users/{uid} — admin writes only      |
| g_mutex_i2c          | FreeRTOS mutex arbitrating I2C bus between IMU and OLED  |
| ekf_yaw              | Heading — may be IMU-integrated or MAVLink-fed, see RULES|
| ui_active_process    | HMI state: 0=IDLE 1=HOME 2=DOCK 3=CALB                  |
| [add yours]          |                                                          |

---

# CONTEXT FILE LOG
> Track changes to this context.md itself. Append only.

| Date       | Changed by    | What changed in this file                           |
|------------|---------------|-----------------------------------------------------|
| YYYY-MM-DD | [name/agent]  | Created initial template                            |
| YYYY-MM-DD | [name/agent]  | Updated HANDOFF STATE after session                 |
| YYYY-MM-DD | [name/agent]  | Added V2 firmware rows to WORKING DIRECTORY         |

---

[End of context.md — HANDOFF STATE is the only section that must be read and updated every session without exception.]