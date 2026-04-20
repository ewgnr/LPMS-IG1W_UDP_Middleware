# LPMS IG1W UDP Middleware

A lightweight C++ middleware that connects to an LPMS IG1W IMU sensor over TCP, parses real-time motion data, and forwards processed IMU output via UDP for downstream applications (robotics, visualization, tracking, etc.).

---

## Features

- TCP connection to LPMS IG1W sensor
- Custom binary protocol parser (state-machine based)
- Automatic initialization sequence (info → config → streaming)
- Real-time IMU data extraction:
  - Quaternion orientation
  - Euler angles
  - Angular velocity
  - Linear acceleration
- UDP output stream (127.0.0.1:9001)
- Debug mode toggle (`p` key)
- Command queue with retry mechanism

---

## 📡 Data Flow


LPMS IG1W Sensor (TCP)
↓
C++ Middleware
↓
Parsed IMU data
↓
UDP output (127.0.0.1:9001)

---

## UDP Output Format

```cpp
struct ImuPacket
{
    float qw, qx, qy, qz;   // quaternion orientation

    float wx, wy, wz;       // angular velocity
    float ax, ay, az;       // linear acceleration

    uint32_t timestamp;
}; 
```

---

## Controls

| Key | Action |
|-----|--------|
| `p` | Toggle debug printing ON/OFF |

---

## Requirements

- Windows OS  
- :contentReference[oaicite:0]{index=0} (C++ workload installed)  
- Winsock2 (built-in Windows networking API)

### Linker dependency:
```cpp
ws2_32.lib
```
---

## How It Works

1. Connects to the IMU sensor at `10.10.0.1:65006` via TCP  
2. Sends initialization commands:
   - Request sensor info  
   - Request configuration  
   - Set streaming mode  
3. Receives continuous binary data stream  
4. Parses incoming bytes using a state-machine parser  
5. Extracts IMU data:
   - Quaternion orientation  
   - Angular velocity  
   - Linear acceleration  
   - Calibration/raw sensor values  
6. Sends simplified IMU packet over UDP to `127.0.0.1:9001`

## Recommended Project Structure

```cpp
LPMS-IG1W_UDP_Middleware/
│── main.cpp
│── LPMS-IG1W_UDP_Middleware.sln
│── .gitignore
│── README.md
```

---

## Notes

- Designed for **real-time streaming**, not offline analysis  
- UDP output is intended for local consumers (e.g., visualization tools, robotics pipelines)  
- Uses a **custom binary protocol** specific to LPMS IG1W  
- Debug output can be toggled with the `p` key  
- Build artifacts (x64/, Debug/, Release/, etc.) should NOT be committed to Git  

---

## License

Free to use for research and development. No warranty provided.