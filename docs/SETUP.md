# Setup — UR5e framework on your laptop

You work against a **simulated UR5e** on your own machine. Three programs to
install — **Docker Desktop, VS Code, Git** — nothing else. Python, the robot
libraries and the simulator all run inside Docker containers; you never install
Python on your laptop.

Works on Apple Silicon Macs (M1–M4), Intel Macs and Windows 10/11. You need
~6 GB free disk space and an internet connection for the first start (it
downloads ~3 GB once).

Time: 20–40 minutes, most of it downloads.

---

## 1. Install Docker Desktop

1. Download from **https://www.docker.com/products/docker-desktop/**
   - **Mac:** pick **Apple Silicon** (M1/M2/M3/M4) or **Intel chip**. Not sure?
     Apple menu  → *About This Mac* → look at "Chip".
   - **Windows:** the Windows (x86_64 / AMD64) installer.
2. Install:
   - **Mac:** open the `.dmg`, drag **Docker** into **Applications**, open it
     from Applications. Allow the privileged-helper prompt (enter your password).
   - **Windows:** run the installer, keep the defaults (**"Use WSL 2"** stays
     checked). It may ask to install/upgrade WSL 2 and to **restart** — do it.
3. Start Docker Desktop. Skip/close any sign-in or survey — no account needed.
   Wait until the whale icon (menu bar / system tray) is **steady, not
   animating**. Leave Docker Desktop running whenever you use the framework.

   `[screenshot: Docker Desktop running, whale icon steady]`

4. **Mac only, recommended:** Docker Desktop → **Settings → General** → make
   sure **"Use Virtualization framework"** and **"Use Rosetta for x86_64/amd64
   emulation on Apple Silicon"** are enabled (Apple Silicon). This makes the
   simulator noticeably faster. Apply & restart.

   `[screenshot: Docker Desktop settings, Rosetta option]`

## 2. Install VS Code

1. Download from **https://code.visualstudio.com/** and install.
2. Open it once. Install the **Python** extension (Extensions sidebar → search
   "Python" → Microsoft). That's the only extension you need.

## 3. Install Git

- **Mac:** open **Terminal** (Applications → Utilities), type `git --version`,
  press Enter. If macOS offers to install the *Command Line Developer Tools*,
  click **Install** and wait. (Or download from https://git-scm.com/download/mac.)
- **Windows:** download **https://git-scm.com/download/win**, run the installer,
  keep the defaults. (This also gives you "Git Bash", which you can ignore — we
  use the VS Code terminal.)

Check: `git --version` prints a version number.

## 4. Get the repository

Pick a **plain local folder** for your course work — your home folder, Desktop or
Documents.

> ⚠️ **Not inside a cloud-synced folder** (iCloud Drive, OneDrive, Dropbox,
> Google Drive). Docker file sharing does not work reliably there. On many Macs
> *Desktop* and *Documents* are in iCloud — check System Settings → Apple ID →
> iCloud Drive → "Desktop & Documents Folders". If it's on, use your home folder
> (`/Users/<you>/`) instead. On Windows, `C:\Users\<you>\Documents` is often
> OneDrive — use `C:\Users\<you>\` directly or `C:\dev\`.

**Option A — clone with Git (recommended, easy updates):**

Open a terminal (Mac: Terminal; Windows: PowerShell) and run:

```bash
cd ~                 # Mac: your home folder     Windows PowerShell: cd $HOME
git clone https://github.com/reckj/UR5-PCL-EmbodiedInteraction.git
```

You now have a folder `UR5-PCL-EmbodiedInteraction`.

**Option B — download ZIP (no Git needed):**

Open https://github.com/reckj/UR5-PCL-EmbodiedInteraction → green **Code**
button → **Download ZIP** → unzip → move the folder
`UR5-PCL-EmbodiedInteraction-main` to the location you picked above.

`[screenshot: GitHub Code button → Download ZIP]`

## 5. Open the folder in VS Code and open a terminal

1. VS Code → **File → Open Folder…** → choose the repository folder → Open.
   If asked, click **"Yes, I trust the authors"**.
2. Open the built-in terminal: **Terminal → New Terminal** (or `` Ctrl+` ``).
   It opens *in the repository folder* — all commands below are typed there.

   `[screenshot: VS Code with the repo open and the terminal panel]`

Check you are in the right place: type `ls` (Mac) or `dir` (Windows) — you
should see `docker-compose.yml`, `docs`, `examples`, `scripts`, `src`.

> Windows note: the VS Code terminal is PowerShell by default. All `docker`
> commands below work the same in PowerShell.

## 6. First start — download and run the simulator

```bash
docker compose up -d ursim viewer
```

The first time this **downloads ~3 GB** (the simulator ≈ 2.2 GB, the Python
image ≈ 0.6 GB). Depending on your connection this takes 5–20 minutes; you see
progress bars. Later starts take seconds.

Expected final output: three lines ending in `Started` / `Running` for
`ursim-1`, `viewer-1` and the network.

`[screenshot: terminal after docker compose up, containers Started]`

- **Windows:** Docker Desktop may pop up a **file-sharing prompt** ("Docker
  Desktop wants to share drive C:" or "share the folder …"). Click **Share it /
  Allow**. If you miss it: Settings → Resources → File sharing → add your repo
  folder (or `C:\Users`).
- **Mac:** the first run may ask for permission to access your folders —
  **Allow**.

## 7. Power on the simulated robot

1. Open **http://localhost:6080/vnc.html** in your browser → click **Connect**.
   You see the robot's teach pendant software (PolyScope).
   - On Apple Silicon the simulator is emulated and **takes 1–3 minutes to
     boot** the first time. If the page shows "Failed to connect", wait 30 s
     and reload.
2. The very first start shows a **"Confirmation of applied Safety
   Configuration"** table — click **Confirm Safety Configuration**. Close the
   "Getting Started" box if it appears (or tick "Don't show this message again").
3. Bottom-left: click the red **power button** → **ON** → wait → **START**.
   The status turns green ("Normal").

   `[screenshot: PolyScope initialize screen, ON and START]`

4. **Enable Remote Control (once):** ☰ menu (top right) → **Settings** →
   **System** → **Remote Control** → **Enable** → **Exit**. A new
   **Local / Remote** icon appears in the header.
5. **Switch to Remote (every start):** click that header icon → **Remote
   Control**. Scripts send motion over the network and PolyScope only accepts
   that in Remote Control — in Local mode motion scripts fail with
   *"Failed to start RTDE data synchronization"*. If the icon says **Local**
   after a restart, switch it again.

   `[screenshot: Settings → System → Remote Control → Enable]`
   `[screenshot: header icon switched to Remote]`

Leave this tab open.

## 8. Open the 3D viewer

Open **http://localhost:7001/** in a second tab. You see the UR5e in 3D (drag
to orbit, scroll to zoom, right-drag to pan). The *Connection* line in the right
panel should say connected and the robot pose should match the sim.

`[screenshot: viewer at localhost:7001 with the arm]`

If it says "connecting …", the sim is not ON/STARTED yet (step 7) — it
reconnects by itself.

## 9. Run the two check scripts

Open a shell *inside* the dev container (this is where Python lives):

```bash
docker compose run --rm dev bash
```

Your prompt changes to something like `root@abc123:/workspace#`. You are now
"inside". Run:

```bash
python scripts/check_connection.py --target sim
```

Expected: `connected : True`, a robot mode, six joint angles, a TCP pose,
`disconnected cleanly.` — nothing moves.

```bash
python scripts/smoke_test_move.py
```

Expected: the wrist turns 10° and back — **watch it in the viewer tab** —
ending in `smoke test PASS`.

`[screenshot: both scripts' output]`

Type `exit` to leave the container shell. **You are set up.** Continue with
[USAGE.md](USAGE.md).

## Daily routine

```bash
docker compose up -d ursim viewer      # start sim + viewer (seconds after the first time)
#   -> localhost:6080/vnc.html : Connect -> ON -> START -> header icon: Remote Control
#   -> localhost:7001          : viewer
docker compose run --rm dev bash       # shell inside the container
python examples/grid_process.py        # ... run things
exit
docker compose down                    # stop everything when you are done
```

**Double-click launchers (optional convenience):** `Start UR5e Sim.command`
(Mac) / `Start UR5e Sim.bat` (Windows) in the repo folder run
`docker compose up -d ursim viewer` and open both browser tabs. The terminal is
still the primary route — you need it for the dev shell anyway. On a Mac the
first time, macOS may refuse to open the `.command` file: right-click →
**Open**, or System Settings → Privacy & Security → **Open Anyway**. If it says
"permission denied", run once in the terminal: `chmod +x "Start UR5e Sim.command"`.

## Updating

When the instructor announces an update:

- **Cloned:** `git pull` in the VS Code terminal.
- **ZIP:** download the ZIP again and replace the folder (keep your
  `students/<name>/` folder — copy it over).

Then `docker compose down` and `docker compose up -d ursim viewer`. If the
update includes a new container image, `docker compose up` downloads it
automatically (a few hundred MB).

---

## Troubleshooting

**"Cannot connect to the Docker daemon" / "docker: command not found"**
Docker Desktop is not running (or, on Windows, the install needs a restart).
Start it, wait for the steady whale, retry. After installing on Windows, close
and reopen VS Code so the terminal picks up the `docker` command.

**Apple Silicon: the simulator is slow / the pendant UI is sluggish**
Expected. URSim exists only as an Intel (AMD64) program and runs under emulation
— Docker Desktop shows an **AMD64** badge next to the `ursim` container for that
reason. It is functional, just slower: boot 1–3 min, UI a bit laggy. Enable
*Rosetta* (step 1.4) if you haven't. Robot motion speed in the sim is not
affected.

**localhost:6080 shows "Failed to connect to server"**
URSim is still booting. Wait 30–60 s and reload. Check with
`docker compose ps` — `ursim` should be `Up`. `docker compose logs ursim`
shows the boot log.

**`check_connection.py` / `smoke_test_move.py`: "connection refused", "not
connected" or hangs**
The robot is not **ON → START**ed yet (step 7), or not in **Remote Control**
(`smoke_test_move.py` needs it; the receive-only check does not). Fix it in the
pendant tab and rerun. A *blank* robot mode or `connected : False` right after
boot means the same thing — URSim takes a moment after START. `robot mode : 7`
in the check output = running, ready.

**`smoke_test_move.py` says "Failed to start RTDE data synchronization" or
"control interface not connected"**
Pendant is in *Local* mode. Switch the header icon to **Remote Control**
(step 7.5); if there is no such icon, enable Remote Control first (step 7.4).

**The viewer page (localhost:7001) doesn't load**
`docker compose ps` — is `viewer` `Up`? `docker compose logs viewer` shows why
not. "port is already allocated": something else uses port 7001 — usually a
second copy of this project running; `docker compose down` there, or in all
folders, then `up` again.

**Viewer shows the arm but it doesn't move while a script runs**
The *Connection* line shows "last update N s ago" — the viewer reconnects by
itself within a couple of seconds. If it stays stale, `docker compose restart
viewer`.

**Windows: "WSL 2 installation is incomplete" / "Docker Desktop requires a
newer WSL kernel"**
Open PowerShell **as Administrator**, run `wsl --update`, then `wsl --install`
if asked, restart, start Docker Desktop again.

**Windows: files edited in VS Code don't change what runs in the container**
The repo is in a folder Docker isn't allowed to share (see the file-sharing
prompt in step 6) or in OneDrive. Docker Desktop → Settings → Resources → File
sharing, or move the folder (step 4).

**Mac: "Start UR5e Sim.command" won't open**
Gatekeeper: right-click → Open (once). Or just use the terminal route.

**Anything else**
`docker compose down` then `docker compose up -d ursim viewer` fixes most
things. Still stuck: bring the output of `docker compose ps` and
`docker compose logs --tail 50` to the session.
