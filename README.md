# Tactic Ops: Proof of Concept

A small tactics game where every move the characters make is created by AI while you play. Instead of playing
pre-made animations, the game asks an AI model to invent each step, turn, crouch and vault on the spot.

Move your soldier around the yard and stay hidden from the two guards on patrol for as long as you can, using walls
and low walls as cover.

> The code was written by Claude Opus 5.5 (Anthropic's AI model), guided and reviewed by a human.

## How to play

### Download

Get the game from the [Releases page](../../releases/latest). You need a graphics card that supports Vulkan.

- **Windows:** download `TacticsPoC-windows-x64.zip`, unzip it and run `tactics_poc.exe`. If Windows warns that the
  app is unrecognized, click **More info**, then **Run anyway**.
- **Linux:** download `TacticsPoC-x86_64.AppImage`, allow it to run (right-click, Properties, "Allow executing as
  program", or `chmod +x TacticsPoC-x86_64.AppImage`) and open it. Works on most distributions from 2024 onwards.

The first start takes a minute or two while the game prepares its graphics. Later starts are quicker.

### The goal

The guards walk around the yard and look in different directions. The yellow areas on the ground show what they can
see. While a guard sees you, the meter at the top fills up, and when it is full you are caught. The timer shows how
long you have stayed hidden.

Walls block a guard's view completely. Low walls only hide you while you are crouched or taking cover behind them.

### Controls

| Action | Control |
|---|---|
| Move | Left click a tile |
| Choose which way to face when you arrive | Hold the left button on a tile and drag (hold Shift for any angle) |
| Crouch or stand up | C |
| Rotate and tilt the camera | Right drag, or Q / E and R / F |
| Move the camera | Middle drag, or W A S D / arrow keys |
| Zoom | Mouse wheel |
| Center the camera on the soldier | Space |
| Help | F1 |
| Full screen | F11 |
| Quit | Esc |

The soldier takes cover on his own when you move him next to a wall, and vaults over low walls in his way.

## Built with

| What | Used for |
|---|---|
| [MotionBricks](https://nvlabs.github.io/motionbricks/) (NVIDIA) | The AI model that creates the characters' movement as you play |
| Motion (private, not yet shared) | Runs MotionBricks on the graphics card inside the game |
| [Kimodo](https://research.nvidia.com/labs/sil/projects/kimodo) (NVIDIA) | The AI model that made the base poses: standing, crouching, running, taking cover, vaulting |
| [SOMA](https://github.com/NVlabs/SOMA-X) (NVIDIA) | The character skeleton both models use |
| [Wicked Engine](https://github.com/turanszkij/WickedEngine) | The 3D engine that draws the game |
| [SDL2](https://www.libsdl.org) | The window, mouse and keyboard on Linux |
| [MakeHuman](https://static.makehumancommunity.org/assets/assetpacks/makehuman_system_assets.html) with [MPFB](https://extensions.blender.org/add-ons/mpfb/) for [Blender](https://www.blender.org) | The soldier and guard characters |
| [Poly Haven](https://polyhaven.com) | The ground and wall textures |

**Note:** the game relies on Motion, a private library that is not shared yet, so this code can't be built on its
own for now. Motion is still in a rough state: it needs a review, documentation and probably a few more features
before it's ready to share. The ready-to-play download includes everything needed.
