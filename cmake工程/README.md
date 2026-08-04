# RM Balance CMake firmware workspace

This directory is isolated from the legacy Keil projects. The initial stage only
validates CMake, Ninja, and the Arm GNU toolchain; no firmware source has been
migrated yet.

Configure and build the placeholder workspace from this directory:

```powershell
cmake --preset debug
cmake --build --preset debug
```

Generated files are written below `build/` and are ignored by Git.
