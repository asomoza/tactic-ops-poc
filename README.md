# Tactic Ops: Proof of Concept

A small tactics game where every move the characters make is created by AI while you play. Instead of playing
pre-made animations, the game asks an AI model to invent each step, turn, crouch and vault on the spot.

Sneak your soldier across the yard, using walls and low walls as cover, without being spotted by the two guards on
patrol.

> The code was written by Claude Opus 5.5 (Anthropic's AI model), guided and reviewed by a human.

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
