# D* Lite Dynamic Path Planner

This project implements a D* Lite based dynamic path-planning application.

The submission contains separate versions for Linux and Windows.

---

## Project Structure

```text
DStarLitePlanner/
│
├── design_report_.md
├── experimental_data.md
│
├── Linux/
│   ├── Compilable/
│   │   ├── arial.ttf
│   │   ├── main.cpp
│   │   └── lib/
│   │       ├── libsfml-graphics.so.2.6.2
│   │       ├── libsfml-system.so.2.6.2
│   │       └── libsfml-window.so.2.6.2
│   │
│   └── Runnable/
│       └── DStarLitePlannerLinux.AppImage
│
└── Windows/
    └── Runnable/
        ├── arial.TTF
        ├── DPathPlannerWindows.exe
        ├── sfml-graphics-2.dll
        ├── sfml-system-2.dll
        └── sfml-window-2.dll
```

---

# Linux

The `Linux` folder contains both a **Runnable** version and a **Compilable** version.

Use the `Linux` folder when running the application on a Linux system.

---

## Option 1 — Run the Linux AppImage

The easiest way to run the application is the provided AppImage.

Navigate to:

```text
Linux/Runnable/
```

### Step 1: Give Execute Permission

Open a terminal in the `Linux/Runnable/` directory and run:

```bash
chmod +x DStarLitePlannerLinux.AppImage
```

### Step 2: Run the Application

```bash
./DStarLitePlannerLinux.AppImage
```

No compilation is required when using the AppImage.

---

# Option 2 — Compile the Linux Version

The Linux source code is provided in:

```text
Linux/Compilable/
```

The following steps should be performed in order.

---

## Step 1 — Download SFML 2.6.2

The required **SFML 2.6.2** package must be downloaded separately.

Download it from the provided Google Drive location:

https://drive.google.com/drive/folders/1A3TZg0dbnmYq8AGlUfp1TFHA9F2zNI8C?usp=drive_link

Extract the downloaded SFML package.

Place the complete `SFML-2.6.2` folder inside:

```text
Linux/Compilable/
```

The required location is:

```text
Linux/Compilable/SFML-2.6.2/
```

**Do not place the SFML folder elsewhere.**

The Linux compilation directory should therefore contain approximately:

```text
Linux/
└── Compilable/
    ├── main.cpp
    ├── arial.ttf
    ├── SFML-2.6.2/
    └── lib/
        ├── libsfml-graphics.so.2.6.2
        ├── libsfml-system.so.2.6.2
        └── libsfml-window.so.2.6.2
```

---

## Step 2 — Check the G++ Version

The Linux source code is intended to be compiled using **G++ 13.1.0**.

Check the installed version with:

```bash
g++ --version
```

The compiler version should be **13.1.0**.

---

## Step 3 — Open the Compilable Directory

Open a terminal inside:

```text
Linux/Compilable/
```

For example:

```bash
cd Linux/Compilable
```

---

## Step 4 — Compile the Application

Run:

```bash
g++ -std=c++17 main.cpp -I SFML-2.6.2/include -L SFML-2.6.2/build/lib -lsfml-graphics -lsfml-window -lsfml-system -Wl,-rpath,'$ORIGIN' -o DStarLitePlanner
```

If compilation is successful, an executable named:

```text
DStarLitePlanner
```

will be created inside the `Linux/Compilable/` directory.

---

## Step 5 — Set the SFML Library Path

**This step is required before running the compiled application.**

The required SFML shared libraries are provided in:

```text
Linux/Compilable/lib/
```

From the `Linux/Compilable/` directory, run:

```bash
export LD_LIBRARY_PATH=./lib
```

This tells Linux to look for the required SFML `.so` files inside the local `lib` folder.

---

## Step 6 — Run the Compiled Application

After setting the library path, run:

```bash
./DStarLitePlanner
```

The application should now start.

---

## Complete Linux Compilation Sequence

For convenience, the complete sequence is:

```bash
cd Linux/Compilable

g++ -std=c++17 main.cpp -I SFML-2.6.2/include -L SFML-2.6.2/build/lib -lsfml-graphics -lsfml-window -lsfml-system -Wl,-rpath,'$ORIGIN' -o DStarLitePlanner

export LD_LIBRARY_PATH=./lib

./DStarLitePlanner
```

> **Important:** The `SFML-2.6.2` folder must be placed inside `Linux/Compilable/` before running the compilation command.

---

# Windows

The `Windows` folder contains a **Runnable version only**.

Use the `Windows` folder on a Windows system.

Navigate to:

```text
Windows/Runnable/
```

The folder contains:

```text
DPathPlannerWindows.exe
sfml-graphics-2.dll
sfml-system-2.dll
sfml-window-2.dll
arial.TTF
```

## Running the Windows Version

Open:

```text
Windows/Runnable/
```

and run:

```text
DPathPlannerWindows.exe
```

Alternatively, from PowerShell:

```powershell
./DPathPlannerWindows.exe
```

The required SFML DLL files and font file are included in the same directory as the executable.

---

## Windows Source Compilation

A compilable Windows setup is not included in the submission.

The Windows executable is provided as the tested **Runnable** version. A source-compilation setup was not included because of compatibility issues between the available MinGW/GCC environment and the prebuilt SFML Windows libraries.

The Linux version provides the complete source code and compilation setup.

---

# Which Version Should Be Used?

| Operating System | Folder | Available Version |
|---|---|---|
| Linux | `Linux/` | Runnable + Compilable |
| Windows | `Windows/` | Runnable |

For Linux, the source compilation setup is available under:

```text
Linux/Compilable/
```

For Windows, use:

```text
Windows/Runnable/
```

---

# Important Notes

- Use the `Linux` folder on Linux systems.
- Use the `Windows` folder on Windows systems.
- For Linux source compilation, **SFML 2.6.2 must be downloaded from the provided Google Drive link**.
- Place `SFML-2.6.2` directly inside `Linux/Compilable/`.
- The Linux source setup requires **G++ 13.1.0**.
- The Linux SFML `.so` libraries are provided in `Linux/Compilable/lib/`.
- After compiling the Linux version, **run `export LD_LIBRARY_PATH=./lib` before running `./DStarLitePlanner`**.
- Keep `arial.ttf` / `arial.TTF` with the corresponding application.
- Do not rename the supplied SFML libraries or DLLs.
- Do not mix Linux and Windows files.
