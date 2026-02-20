# 🖱️ Mouseofono: Invisible Ears at Your Fingertips

**Mouseofono** converts a standard computer mouse sensor into a high-precision vibration sensor and microphone. Based on recent acoustic eavesdropping research ([arXiv:2509.13581v2](https://arxiv.org/html/2509.13581v2)), it allows users to record audio and analyze vibrations by capturing micro-movements (sensor jitter) that are otherwise discarded by the operating system.

![C++17](https://img.shields.io/badge/C++-17-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20WSL2%20%7C%20Docker-brightgreen.svg)
![License](https://img.shields.io/badge/License-MIT-orange.svg)

---

## 🔥 Key Features

- **Hybrid Architecture**: Native Windows capture via **Raw Input API** + Linux/Docker support via **evdev**.
- **Advanced DSP Engine**: 
  - **Sinc Reconstruction**: Band-limited signal reconstruction for high-fidelity audio.
  - **Wiener Filter**: Active spectral subtraction to remove optical sensor noise (sensor jitter).
  - **Phase-Correct Timing**: QPC-synchronized timestamps ensuring a perfect 1:1 timeline.
- **Sensitivity Optimization**:
  - **Adaptive Bandwidth**: Automatically scales based on measured polling rate.
  - **--raw mode**: Direct delta accumulation for low-energy vibration detection.
  - **Noise Calibration**: 10-second profiling phase to characterize unique sensor noise.

## 👋 Quick Start (Windows)

### 1. Requirements
- A high-DPI mouse (e.g., Logitech G203, Razer Viper 8K).
- Visual Studio 2022 (Build Tools).
- **Pro Tip**: Set your mouse to **MAX DPI** in G HUB/Synapse for best results.

### 2. Build
```powershell
mkdir build; cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

### 3. Usage Workflow
To get the best audio quality, follow the **Calibration Workflow**:

1. **Calibrate**: Place the mouse on the target surface and keep it absolutely still.
   ```powershell
   .\Release\mouseofono.exe --calibrate
   ```
2. **Record**: Start recording with the noise filter active.
   ```powershell
   .\Release\mouseofono.exe 15.0 --wiener --raw
   ```
3. **Analyze**: Open `output.wav` in Audacity.

## 🎛️ CLI Arguments

| Flag | Description | Default |
|---|---|---|
| `[gain]` | Output multiplier (float). | 1.0 |
| `--raw` | Bypass sinc processing (best for vibration). | ON (v1.2) |
| `--wiener` | Apply spectral subtraction filter (requires calibration). | OFF |
| `--calibrate`| Capture 10s of jitter and save noise profile. | - |
| `--fs <hz>` | Target sample rate. | 16000 |
| `--no-agc` | Disable Automatic Gain Control. | OFF |

---

## 📖 Research Context
This project implements techniques described in the paper *"Invisible Ears at Your Fingertips: Acoustic Eavesdropping via Mouse Sensors"*. It addresses the challenge of non-uniform sampling and high noise floor inherent in optical sensors, transforming a human-interface device into an acoustic sensor capable of picking up vibrations and speech.

## 🤝 Contributing
Feel free to open issues or submit PRs. The move towards Phase 2 (Docker and multi-channel support) is currently underway.

---
*Developed by Antigravity AI.*
