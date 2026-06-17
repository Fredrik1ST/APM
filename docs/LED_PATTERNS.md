This document describe the planned blinking patterns of the two rear red & green LEDs for various modes.

# Convention:

⚫ = Off

🟢 / 🔴 = On

🟢✨ / 🔴✨ = Blink

🟢✨✨ / 🔴✨✨ = Rapid blink

# Universal

| Green LED | Red LED | Meaning |
|---|---|---|
| ⚫ | ⚫ | System not powered |
| ⚫ | 🔴 | System off / Arduino disconnected |
| 🟢 | 🔴 | Connecting / Idle |
| 🟢 | ⚫ | Mode running normally |

# Normal mode

| Green LED | Red LED | Meaning |
|---|---|---|
| 🟢 | ⚫ | Running cruise controller (i.e. following pacing profile) |
| 🟢 | 🔴✨ | Runner falling behind (switches to distance control if runner does not catch up) |
| 🟢✨ | ⚫ | Running distance controller (until runner catches up to pacing profile, on which cruise control resumes)

# Pace only mode

| Green LED | Red LED | Meaning |
|---|---|---|
| 🟢 | ⚫ | Following pacing profile, runner detected |
| 🟢 | 🔴✨ | Runner lost or at standstill (run stops if this persists past `runner_timeout`) |

# Rear camera test mode
| Green LED | Red LED | Meaning |
|---|---|---|
| 🟢 | ⚫ | Runner detected |
| ⚫ | 🔴 | No runner detected |

# Front camera test mode
| Green LED | Red LED | Meaning |
|---|---|---|
| 🟢 | ⚫ | Lane lines detected |
| ⚫ | 🔴 | No lane lines detected |

# GNSS test mode
| Green LED | Red LED | Meaning |
|---|---|---|
| 🟢 | ⚫ | GNSS fix received recently |
| 🟢✨✨ | ⚫ | Short blink off/on for every fix received |
| ⚫ | 🔴 | No GNSS fix received |