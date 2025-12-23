# Development Setup  

## Supported Platforms

This setup is tested and supported on:

- Linux (Ubuntu / Debian)
- macOS
- Windows 10 / 11

---

## 2 Required System Tools

All tools below must be installed and available in your `PATH`.

---

### 2.1 LLVM Toolchain

LLVM tooling for formatting, static analysis, and editor intelligence is used.

Required tools (part of LLVM):
- `clang-format`
- `clang-tidy`
- `clangd`

#### Linux (Ubuntu / Debian)
```bash
sudo apt update
sudo apt install clang clang-format clang-tidy clangd
```

#### macOS
```bash
brew install llvm
```

#### Windows
```bash
choco install llvm
```

#### Verification
```bash
clangd --version
clang-format --version
clang-tidy --version
```
If any of these commands fail, fix this before proceeding.

## 3. Visual Studio Code
### 3.1 Install VS Code

Download and install VS Code:

https://code.visualstudio.com/

No additional configuration is required at this stage.

### 3.2 Platform IO

Download and install:

https://platformio.org/install

## 4. VS Code Extensions

The project expects exactly one C++ language server: `clangd`.

**Required Extensions**
- PlatformIO IDE
- C/C++ (Microsoft)
- clangd (LLVM)

These are recommended automatically via `.vscode/extensions.json` when the repository is opened.

Extensions You Should **NOT** Install:
- Any additional C++ language servers
- Alternative formatters (AStyle, Uncrustify, etc.)
- IntelliSense extension packs

Installing multiple language servers will cause duplicate diagnostics and unstable behavior.

## 5. VS Code Profile (One-Time User Setup)

Use a dedicated VS Code profile to avoid conflicts with another workflows.

Recommended Profile Name
```
Embedded / C++ (PlatformIO)
```

### Creating the Profile

- Open Command Palette
- Run: `Profiles: Create Profile`
- Choose `Create Empty Profile`
- Switch to the new profile
- Install the required extensions

### Required Profile-Level Settings

Add the following to profile settings (not workspace settings):
```json
{
  "C_Cpp.intelliSenseEngine": "disabled",
  "platformio-ide.useBuiltinIntelliSense": false,
  "C_Cpp.errorSquiggles": "disabled"
}
```
Why This Is Required ?
- Disables Microsoft IntelliSense
- Ensures `clangd` is the only active language server
- Prevents duplicate or conflicting diagnostics
- Avoids PlatformIO IntelliSense interference

These settings are workflow preferences, not project rules, which is why they are user-level.

## 6. Generate Compilation Database (Mandatory)

`clangd` and `clang-tidy` require a compilation database to work correctly.

From the project root run:
```bash
pio run -t compiledb
```

This will generate `compile_commands.json`

When to run it:
- After cloning the repository
- After modifying platformio.ini
- After switching branches with different build flags

Without this file, clangd diagnostics will be incomplete or incorrect.

## 7. Setup verification
### 7.1 Verify **clangd** Is Active

- Open any .cpp file
- Status bar (bottom right) should show **clangd**
- Command Palette contains: `Clangd: Restart language server`

### 7.2 Verify Formatting

- Open a .cpp file
- Save the file
- Expected: Code is automatically formatted and formatting matches `.clang-format`

### 7.3 Verify Static Analysis

Insert a deliberate error:
```c
int* p = nullptr;
*p = 1;
```

Expected:
- Warning in the editor
- Entry in the Problems panel

### 7.4. Building and Uploading
Build
```bash
pio run
```

Upload to Target
```bash
pio run -t upload
```

