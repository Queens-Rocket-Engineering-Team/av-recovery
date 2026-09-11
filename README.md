# Avionics Recovery (GPS and Altimter)
> 2023-2026 Season - Recovery Projects

> [!IMPORTANT]
> Make sure to initialize and pull the repository's git submodules to acquire all schematic library and board design dependencies:
> ```bash
> git submodule update --init --recursive
> ```

-  Repo contains board's **firmware**, **schematics**, and related **documentation and datasheets**.
-  The **Main** branch contains current functional hardware/software
-  **Branches** contain board revisions/versions or planned features/fixes.
-  If you're contributing, **make sure you're working in a branch other than Main**

### Folder Structure
- [`/Schematics`](./Schematics/) : PCB design files (KiCad V8)  
- [`/Firmware`](./Firmware/) : Arduino C/C++ code (for STM)
- [`/Datasheets`](./Datasheets/) : Datasheets and reference materials  
