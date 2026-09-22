# VRC-CB Digital Heating Controller

An open-source replacement controller for the **Vaillant VRC-CB** heating controller.

The project replaces the original analogue controller board while retaining the
original plastic controller housing and external wiring. The goal is to provide
a modern, programmable controller without replacing the existing controller
enclosure.

## Main features

* Designed as a replacement PCB for the original VRC-CB controller
* Uses an Arduino Nano ESP32
* Digital control of the original heating-system interface
* Optional room-temperature measurement using an NTC connected to **pins 22 and 23**
* Local temperature/humidity sensing
* Web-based monitoring and control
* Firmware and KiCad hardware design files are provided

## Repository structure

```text
.
├── firmware/                 # Arduino/ESP32 firmware
├── hardware/
│   ├── kicad/                # KiCad schematic and PCB source files
│   ├── gerbers/              # Manufacturing files
│   └── fabrication/          # Drill files, BOM, pick-and-place, etc.
├── docs/                     # Build, installation and technical documentation
├── images/                   # Photos and diagrams
├── LICENSES/                 # License information
├── README.md
└── .gitignore
```

## Hardware

The PCB is intended to fit inside the original VRC-CB plastic housing and to
connect to the existing wiring using the original connector arrangement.

**Important:** Pins 22 and 23 are an optional addition for a room-temperature
NTC. The original controller does not require this connection.

The exact supported NTC type, resistance range and firmware configuration should
be documented in `docs/` before release.

## Software

The firmware is intended for the Arduino Nano ESP32.

## Hardware licence

The KiCad hardware design files are released under:

**CERN Open Hardware Licence Version 2, Strongly Reciprocal
(CERN-OHL-S-2.0)**

See `LICENSES/HARDWARE-LICENSE.md`.

The CERN OHL v2 licence text is maintained by CERN. The definitive licence
text is available from:

https://cern-ohl.web.cern.ch/

## Software license

Software is released under:

**GNU General Public License v3.0 or later (GPL-3.0-or-later)**

See `LICENSES/SOFTWARE-LICENSE.md`.

## Documentation licence

Unless stated otherwise, documentation and original project images are
released under:

**Creative Commons Attribution-ShareAlike 4.0 International
(CC BY-SA 4.0)**

See `LICENSES/DOCUMENTATION-LICENSE.md`.

## Disclaimer

This is an independent hobby/open-hardware project. It is **not affiliated
with, endorsed by, or produced by Vaillant**.

The project is intended for technically competent users who understand the
heating system they are modifying. Working on heating equipment and its
electrical connections can involve serious safety risks.

Do not build or install this controller unless you understand the relevant
electrical, heating-system and local safety requirements. The author provides
the design and software without warranty and accepts no responsibility for
damage, injury or loss resulting from its use.

Before public release, the project documentation should clearly identify the
tested hardware revision, firmware version and supported heating-system
configuration.

## Status

This repository contains a working prototype/controller and is currently being

prepared for its first documented public release.



The hardware and firmware have been tested as a working system. Additional

documentation and manufacturing files will be added incrementally.



The first documented release will include:

* PCB revision
* Firmware version
* Tested Vaillant controller/system configuration
* Arduino Nano ESP32 version
* Sensor configuration
* Known limitations
* Bill of materials
* Build and installation documentation

## Contributions

Issues, suggestions and improvements are welcome. Please open an issue or
pull request on GitHub.

## Parts

A detailed bill of materials will be added to `docs/` or `hardware/` before
the first public release.

Where possible, the exact manufacturer part number should be specified so that
builders can source compatible components independently.

