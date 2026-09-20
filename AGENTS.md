---
name: inf2004-embedded-systems
description: Coursework repository for the INF2004 Embedded Systems module.
---

# INF2004 Embedded Systems

This repository contains INF2004 Embedded Systems coursework. Use the [Agent Navigation Map](docs/AGENT_MAP.md) to find repository entry points and the [documentation index](docs/README.md) to find durable records.

## Read first

1. [Agent Navigation Map](docs/AGENT_MAP.md) — repository structure and map chain.
2. [Documentation index](docs/README.md) — durable records and current repository documentation.
3. [README](README.md) — public orientation and repository usage.

## Commands

```sh
cmake -S . -B build -DPICO_BOARD=pico_w
cmake --build build
python3 scripts/check-docs.py
bash scripts/install-hooks.sh
```

## State

- This is an independent Git repository with no nested repositories or submodules.
- The current workspace checks it out as a submodule of the School Repos superproject. Edit, test, commit, and push changes in this repository before the parent records its updated gitlink.
- The repository contains CMake firmware targets for Pico W diagnostics and Robo Pico hardware calibration.
- Build output belongs in `build/` and is not committed.

## Conventions

- Keep coursework in clearly named task directories when coursework is added.
- Keep standard repository entry points at the repository root.
- Use ASD-STE100 Simplified Technical English for repository instructions.
- Store each durable record—such as an architecture note, plan, runbook, status record, release note, or ADR—in its own aptly named file under `docs/`, and list it in `docs/README.md`.
- Use `git commit --no-verify` only as an intentional escape hatch from the repository-owned pre-commit hook.

## Map freshness

- Update `docs/AGENT_MAP.md` in the same commit as any change that affects paths, entry points, documentation locations, or repository boundaries it describes.
- Before committing documentation or map changes, verify every map path and link. Run `python3 scripts/check-docs.py`; it checks mechanical link and anchor validity, but cannot judge whether a map description is semantically current.
- Run `bash scripts/install-hooks.sh` after a fresh clone and after adding a repository boundary that needs the shared hook.
