# ArduRocket — Development Setup

How to get a fresh machine from nothing to a **built, runnable ArduRocket** in
**VS Code on Ubuntu under WSL2** — the environment this vehicle is developed on.

This is the *onboarding* doc. Once you're set up, the day-to-day workflow lives elsewhere:
- **[`ARDUROCKET_README.md`](ARDUROCKET_README.md)** — how to run the sim and fly it (§5),
  ground-station setup (§3), configuration (§6).
- **[`ARDUROCKET_PLAN.md`](ARDUROCKET_PLAN.md)** — why it's built this way.
- **[`ARDUROCKET_STATUS.md`](ARDUROCKET_STATUS.md)** — what's done and what's next.

---

## 1. WSL + Ubuntu (Windows PowerShell)

```powershell
wsl --install -d Ubuntu       # skip if Ubuntu is already installed
wsl --update
```

Open **Ubuntu** and do everything below inside that Linux shell — the build, the
toolchains, and the simulator all run in Linux, not Windows.

## 2. Get the source

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

Already cloned? Just make sure submodules are current:

```bash
git submodule update --init --recursive
```

## 3. Build prerequisites

```bash
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile        # or restart the shell — puts waf + toolchains on PATH
```

This installs gcc, the ARM toolchain, the Python build deps, and MAVProxy.

## 4. VS Code on WSL

VS Code runs on **Windows** and reaches into Linux over the WSL remote — you do not
install it inside Ubuntu.

1. Install **VS Code** on Windows.
2. Install the **WSL** extension (`ms-vscode-remote.remote-wsl`).
3. From the Ubuntu shell, open the repo as a WSL-remote window:
   ```bash
   cd ~/ardupilot && code .
   ```
   Confirm the bottom-left corner reads **WSL: Ubuntu**. If it doesn't, you're editing
   over the Windows filesystem and the build won't work — reopen via `code .` from the
   Linux shell.

## 5. Extensions and editor config

The repo ships the recommendation list and default configs.

1. Open the **Extensions** panel → filter **Recommended** → install all. Key ones:
   `ms-vscode.cpptools`, `ms-python.python`, `ardupilot-org.ardupilot-devenv`, plus
   astyle / spell-check / lua. (Source of truth: `.vscode/extensions.json`.)
2. Activate the repo defaults (they're `*.default.json` so they don't clobber a local
   copy):
   ```bash
   cd ~/ardupilot/.vscode
   cp settings.default.json settings.json
   cp launch.default.json   launch.json
   ```

> **Debugging the rocket:** `launch.default.json`'s `AP_SITL` config launches
> `--model +` (a plain multicopter). To F5-debug ArduRocket specifically, change that
> config's `args` to `--model rocket`.

## 6. Build

From the VS Code integrated terminal (which is now a WSL shell):

```bash
./waf configure --board sitl     # once, and again after editing rocket.parm
./waf rocket                     # builds build/sitl/bin/rocket
```

## 7. First run

You now have a working environment. To launch the simulator and fly it, follow
**[`ARDUROCKET_README.md`](ARDUROCKET_README.md) §5** — the short version:

```bash
build/sitl/bin/rocket --model rocket -w
```

then connect QGroundControl over an **explicit TCP link to `127.0.0.1:5760`** (not
auto-connect, which is UDP and doesn't traverse WSL cleanly), install the **Fin Check**
button (README §3), and fly: **Fin Check → ARM → ignition (3 s) → flight**.

---

## Networking notes (WSL2)

- **QGC ↔ SITL** — WSL2 forwards Windows `localhost` into WSL, so a **TCP** link to
  `127.0.0.1:5760` works from QGC on Windows. Use an explicit link; QGC's auto-connect is
  UDP and does not traverse WSL cleanly. (README §3, §5.1.)
- **MATLAB-in-the-loop sim** — the physics UDP link (port 9002) crosses the
  WSL↔Windows boundary. Enable **WSL2 mirrored networking** in `C:\Users\<you>\.wslconfig`:
  ```ini
  [wsl2]
  networkingMode=mirrored
  ```
  then `wsl --shutdown` and reopen. WSL and Windows then share `localhost` for TCP **and**
  UDP. Details and the verification test: `Tools/ArduRocket/matlab/README.md`.
