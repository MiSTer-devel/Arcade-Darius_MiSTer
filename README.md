# Arcade-Darius_MiSTer

FPGA core for **Darius** (Taito Corporation, 1986) targeting the
[MiSTer FPGA](https://github.com/MiSTer-devel) platform (Terasic DE10-Nano).

Darius runs on a **dual 68000 + Z80 ×2 board** with two PC080SN tile layers,
PC090OJ sprites, PC060HA master/slave communication, and **YM2203 ×2 +
MSM5205** audio, driving the game's signature **triple-screen** ultra-wide
display. This core reimplements the hardware in SystemVerilog from MAME
references and hardware observation on original PCBs.

## About the game

**Darius** is Taito's landmark horizontal shoot-'em-up, famous for its
**three-monitor ultra-wide screen** and its all-aquatic-themed mechanical
bosses. You pilot the **Silver Hawk** starfighter through a branching tree of
zones (pick your path at the end of each stage), fighting waves of enemies to
a memorable Zuntata soundtrack, with the iconic "Warning!! A huge battleship
is approaching" boss cadence. The board composes three horizontal panels into
one continuous 864-pixel-wide playfield.

## Status

**Current version: 1.8** (July 2026)

The core runs the full game end-to-end on all supported ROM sets and has been
tested on real MiSTer hardware. Since v1.1 the audio path, the sprite engine
and the video/CRT tooling have been substantially reworked, and an
**experimental savestate** system (16 slots) has been added.

## What's new in v1.8

- **Sprite engine fixes** — the sprite object RAM is now **double-buffered and
  latched at vblank** (buffered sprite RAM, like the original hardware). This
  removes a rare but real defect where a mid-frame CPU update of the sprite
  list produced **inconsistent scanlines** (part of the screen using the old
  sprite data, part the new). Line preparation was also reworked with a
  shadow-scan FIFO and a small sprite-ROM cache to reduce SDRAM contention.
- **Audio mixer rework** — a new **per-channel mixer** (9 channels: FM ×2,
  PSG/SSG ×6, MSM5205 ADPCM) with **per-channel volume and pan** exposed in the
  OSD, a linear/transparent gain chain and a final soft-clip stage sized on the
  real worst case (no more square-wave saturation when many voices sum). The
  ADPCM ROM was moved into BRAM for deterministic latency, and various sound
  path fixes improve accuracy.
- **Experimental savestate — 16 slots** — full video / CPU state plus a
  deterministic audio restore. Select the slot from the OSD. Still
  **experimental**: see Known issues.
- **CRT Adjust module** — a core-side analog CRT alignment tool (validated and
  released, see the dedicated section below), bringing **H-Size**,
  **H-Position** and **V-Shift** together in one line-buffer module that
  **never desyncs the CRT**. Alongside it, an **experimental** set of
  single-screen views for the triple screen — **Screen Jump**, **triple-screen
  wide** and a **follow-cam** mode that tracks the Silver Hawk — opt-in from the
  OSD.
- DSP / BRAM optimizations across the audio mixer and video path.

## Known issues (v1.8)

- **Savestate is experimental.** It works across the 16 slots but is not
  guaranteed solid in every situation; loading a state saved by an older build
  is not supported. Kept under observation.
- **Experimental single-screen views** — the triple-screen views (Screen Jump,
  wide zoom, follow-cam) are opt-in and still being refined. (CRT Adjust itself
  — H-Size / H-Position / V-Shift — is validated and released, not
  experimental.)
- **15 kHz CRT output** — the native 864-pixel triple-panel resolution is
  hard on the MiSTer analog I/O board, but the new **experimental CRT views**
  (wide zoom, Triple CRT Wide, follow-cam) make 15 kHz **usable** — a lot more
  than before. The single-screen follow-cam / wide modes in particular give a
  proper CRT-friendly picture. This is via experimental features and not yet
  guaranteed in every mode.

## Milestones reached

- Boots and plays the full game with accurate video and controls on all sets
- **Dual 68000** main/sub with shared RAM (FX68K core), PC060HA master/slave
  communication reproduced from MAME
- Three horizontal panels composed into the continuous 864-pixel screen
- **Sprite renderer** with priority, flip, 16×16 4bpp tiles, **buffered
  (vblank-latched) sprite RAM**, shadow-scan FIFO and sprite-ROM cache
- FG text / HUD layer with local BRAM text ROM
- Palette with MAME-verified byte order
- **Full audio chain**: YM2203 ×2 (FM + SSG/PSG) + MSM5205 ADPCM, per-channel
  mixer with volume/pan
- **Experimental 16-slot savestate** (video + CPU + deterministic audio)
- **CRT Adjust** (validated / released) — core-side analog geometry (H-Size,
  H-Position, V-Shift) that never desyncs the CRT
- **Experimental single-screen views for a triple-screen game** — wide zoom
  modes (288/320/432), Screen Jump, Triple CRT Wide, and a **follow-cam** that
  tracks the Silver Hawk: play the ultra-wide triple game on one monitor / CRT

## Roadmap / still to implement

- Make the **savestate** solid enough to drop the "experimental" tag
- Finish and stabilize the **single-screen / follow-cam views**
- Optional **high-score saving** (NVRAM) — under evaluation, deliberately
  non-invasive (no CPU pause), not in this release

**Features**
- Dual 68000 main/sub @ 8 MHz with shared RAM (FX68K core)
- Two Z80 sound CPUs (music + ADPCM/FX)
- Three horizontal panels composed into the 864-pixel virtual screen
- Sprite renderer with priority, flip, 16×16 4bpp tiles, **buffered sprite RAM**
- FG text/HUD layer with local BRAM text ROM
- Palette with MAME-verified byte order
- Audio: YM2203 ×2 (FM + PSG) + MSM5205 ADPCM with **per-channel volume/pan**
  mixer
- PC060HA main/sub communication
- SDRAM arbitration for tile/sprite ROM and main/sub CPU fetch
- **Experimental savestate (16 slots)** from the OSD
- **CRT Adjust** (H-Size / H-Position / V-Shift) — validated and released
- **Experimental single-screen views** (Screen Jump, triple wide, follow-cam)
- Inputs (coin, start, 8-way joystick) and full DIP switch support
- MiSTer OSD with video, audio-mix, pause and DIP options
- Pause overlay with project logo, scrolling supporters list and social links
- "Clean Pause" OSD option (pause without dim/logo overlay)

**ROM sets supported**
- Darius (World) — reference set
- Darius (US)
- Darius (Japan rev 1 — dariusj)
- Darius (Japan pure — dariuso)
- Darius Extra Version (dariuse)

## CRT Adjust

**CRT Adjust** is a core-side analog CRT geometry / alignment tool exposed in
the OSD — **validated and released** (not experimental). It is the evolution of
the earlier "Analog H-Size" module: a single always-on line buffer that
provides **three live controls** for lining the image up perfectly on a real
CRT.

| Control        | What it does                                                       |
|----------------|--------------------------------------------------------------------|
| **H-Size**     | Horizontal stretch / squeeze — bidirectional, **integer** (every source pixel is emitted for a whole number of pixel clocks) |
| **H-Position** | Horizontal image shift — moves the picture **content**, not the sync |
| **V-Shift**    | Vertical line shift                                                 |

**Why it never desyncs the CRT.** The picture **content** is shifted and
resized through the line buffer while the horizontal/vertical **sync signals
stay native**. The CRT keeps its lock at all times, so you can slide and resize
the image **live** without the screen rolling or losing hold — unlike moving
the blanking / sync windows.

Because the resize is **integer** (no fractional ratio, no per-pixel
nearest-neighbor) and the output is the byte-exact source pixel, there is **no
shimmering** on moving content and **no blending / blur** — the image stays
sharp while you stretch it.

The module is inserted **core-side** at the video-output boundary, with **zero
`sys/` framework changes** (MiSTer-devel compliant — the framework is never
modified). The trade-off is that the adjustment reaches the analog DAC **and
HDMI follows it too**: leave CRT Adjust **Off** (default) for an untouched HDMI
image, and turn it on when aligning on an analog CRT. Cost is about **1 M10K,
~50 ALM, 0 DSP**.

## Triple-screen views, wide zoom & follow-cam (experimental)

Darius is a **three-monitor** game: its native picture is an 864-pixel-wide
triple panel. On a single modern display that means a very wide, short image.
This core adds a **unique set of display modes** — as far as we know **nothing
else on MiSTer does this** — that reshape the triple screen for a single
monitor **without touching the game logic** and while keeping the native
15.7 kHz line rate (so it stays CRT-friendly and never desyncs).

The picture is captured **after** the compositor, buffered per line
(lossless), and re-emitted into a **wider active window** with **integer pixel
repeat** — so the image genuinely grows on a CRT with no blur and no shimmer.

| Mode              | What you get                                                    |
|-------------------|-----------------------------------------------------------------|
| **Triple**        | Native triple panel (864 px) — bypass, unchanged                |
| **Wide 288 ×4**   | Single-panel window, panel proportions, zoomed                  |
| **Wide 320 ×4**   | "4:3-pushed" window (1280 active slots)                          |
| **Wide 432 ×3**   | ~1.5-screen window (1296 active slots)                           |
| **Screen Jump**   | Single-panel view that **jumps** panel-to-panel with the action |
| **Triple CRT Wide** | Native triple re-emitted at a wider integer rate (864 active over ~1018 slots) so the whole triple screen fills a CRT line — integer 1:1 repeat, no scaling |

- **Follow-cam** — instead of showing all three panels, the view becomes a
  **single zoomed screen that follows the Silver Hawk**: the camera tracks the
  ship (deadzone + smooth slew, updated at end of vblank) and eases back to the
  playfield center when the ship is absent (title / death). It turns the
  ultra-wide triple game into a normal-proportioned single-screen experience
  you can actually play comfortably on one monitor / CRT.

All of the above are **experimental**, opt-in from the OSD, and leave the
default **Triple** view untouched.

## Screenshots

| | |
|---|---|
| ![Title](docs/title.png) | ![Silver Hawk attract](docs/attract_silver_hawk.png) |
| Title screen | Attract — Silver Hawk briefing |
| ![Zone C](docs/gameplay_zone_c.png) | ![Zone I](docs/gameplay_zone_i.png) |
| Zone C | Zone I |
| ![Zone S](docs/gameplay_zone_s.png) | ![Zone G](docs/gameplay_zone_g.png) |
| Zone S | Zone G |

## Hardware emulated

| Component        | Spec                                                     |
|------------------|----------------------------------------------------------|
| Main CPU         | Motorola 68000 @ 8 MHz (FX68K)                           |
| Sub CPU          | Motorola 68000 @ 8 MHz (FX68K)                           |
| Sound CPU        | Z80 ×2 (music + ADPCM/FX)                                |
| Sound chip 1     | Yamaha YM2203 OPN ×2 — FM + SSG/PSG (jt12)               |
| Sound chip 2     | OKI MSM5205 ADPCM (jt5205)                               |
| Tilemaps         | Taito PC080SN ×2 (two scrolling tile layers)             |
| Sprites          | Taito PC090OJ (16×16 4bpp, buffered sprite RAM)          |
| Comms            | Taito PC060HA (master/slave)                             |
| Display          | Triple horizontal panels → 864×224                       |

## Hardware requirements

- Terasic DE10-Nano
- MiSTer I/O board (recommended)
- SDRAM module (128 MB, 64 MB or 32 MB)
- HDMI display (recommended), or HDMI→VGA adapter for 31 kHz VGA monitors

**Note on video output**: The core works on standard HDMI displays. For
31 kHz VGA monitors, use an HDMI→VGA adapter — do **not** enable Direct Video.
**15 kHz CRT output is not currently supported** because the 864×224
triple-panel resolution is incompatible with the MiSTer analog I/O board.

## Build from source

Requires **Quartus Prime Lite 17.0.x** for Cyclone V (5CSEBA6U23I7).

```bash
cd Arcade-Darius_MiSTer
quartus_sh --flow compile Darius -c Darius
```

Output: `output_files/Darius.rbf` (~3.9 MB).

## Running on MiSTer

The [releases/](releases/) folder contains the pre-built bitstream and the
parent MRA for the reference ROM set:

- `Darius_YYYYMMDD.rbf` — pre-built core bitstream
- `Darius (World).mra` — parent MRA (reference set)

Alternative ROM sets are provided in [releases/alternatives/](releases/alternatives/):

- `Darius (US).mra`
- `Darius (Japan).mra`
- `Darius (Japan, rev 1).mra`
- `Darius Extra Version (Japan).mra`

Following the MiSTer-devel convention, the alternative sets are also mirrored
to the official [MRA-Alternatives_MiSTer](https://github.com/MiSTer-devel/MRA-Alternatives_MiSTer)
repository, where they are picked up automatically by **Update_All**.

Steps:

1. Copy the `.rbf` to `_Arcade/cores/` on the MiSTer SD card.
2. Copy the desired `.mra` file(s) to `_Arcade/` on the MiSTer SD card.
3. Provide your legally-owned Darius ROM files where each MRA expects them
   (usually in `games/mame/`).
4. For HDMI displays, no special setup is required. For 31 kHz VGA monitors,
   use an HDMI→VGA adapter (do **not** enable Direct Video). 15 kHz CRT output
   is not currently supported (see Hardware requirements).
5. The core OSD defaults to **Narrower HV-Integer** scale, which gives a proper
   integer-scaled image on modern HDMI displays. Other Scale options
   (V-Integer, HV-Integer) are available for custom setups.

**ROMs are NOT included in this repository.** You must provide them yourself.

## Repository layout

```
Arcade-Darius_MiSTer/
├── rtl/
│   ├── darius/       Darius-specific core RTL (Umberto Parisi)
│   ├── fx68k/        M68000 core (Jorge Cwik)
│   ├── jt12/         YM2203 FM synth (Jose Tejada)
│   ├── jt5205/       MSM5205 ADPCM (Jose Tejada)
│   ├── jtframe/      JTFRAME framework modules (Jose Tejada)
│   ├── t80/          Z80 core
│   ├── pll/          Clock PLL
│   └── sdram.sv      SDRAM controller (Sorgelig)
├── sys/              MiSTer framework (Sorgelig / MiSTer-devel)
├── releases/         Pre-built .rbf + parent MRA
│   └── alternatives/ MRA files for alternate ROM sets
├── docs/             Screenshots
├── Darius.qpf        Quartus project
├── Darius.qsf        Quartus assignments
├── Template.sv       Top-level wrapper
├── Template.sdc      Timing constraints
├── files.qip         HDL file list
├── build_id.v        Build version stamp
├── LICENSE           GNU GPL v3
├── AUTHORS.md        Credits and third-party licenses
└── README.md         This file
```

## License

Distributed under **GNU General Public License v3 or later**. See
[LICENSE](LICENSE) and [AUTHORS.md](AUTHORS.md) for credits and third-party
licensing.

GPL-3 is chosen to stay compatible with upstream GPL-3 dependencies (JTFRAME,
FX68K, Sorgelig's sdram and sys framework).

## Acknowledgements

- **Jorge Cwik** — [FX68K](https://github.com/ijor/fx68k) (M68000 core)
- **Jose Tejada** ([@jotego](https://github.com/jotego)) — JTFRAME, JT12
  (YM2203), JT5205 (MSM5205)
- **Sorgelig** and the **MiSTer-devel team** — framework, SDRAM controller and
  Template
- **Andrea Bogazzi** ([@asturur](https://github.com/asturur)) — help with the
  core-side CRT Adjust / Analog H-Size implementation
- The **MAMEDev team** — invaluable reference on the Taito PC080SN / PC090OJ /
  PC060HA hardware, memory maps and timing

Full list in [AUTHORS.md](AUTHORS.md).

## Support this project

If you enjoy this core and want to support its development:

- [Ko-fi](https://ko-fi.com/ibecerivideoludici) — one-time support
- [Patreon](https://www.patreon.com/IBeceriVideoludici) — monthly support
- [PayPal](https://www.paypal.me/IBeceriVideoludici) — one-time donation

## Follow

- [GitHub](https://github.com/rmonic79)
- [Twitch](https://twitch.tv/ibecerivideoludici) — live streams
- [YouTube](https://www.youtube.com/c/IBeceriVideoludici/playlists) — playlists and videos
- [X / Twitter](https://x.com/rmonic79)

Original *Darius* arcade hardware © Taito Corporation, 1986.
