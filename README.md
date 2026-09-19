# INF2004 Embedded Systems

This repository contains coursework for the INF2004 Embedded Systems module. It currently contains repository documentation and maintenance tooling; coursework tasks have not been added.

## Repository layout

- [`docs/`](docs/README.md) — durable documentation and the agent navigation map.
- [`scripts/`](scripts/) — repository maintenance scripts.
- [`.githooks/`](.githooks/) — repository-owned Git hooks.

Future coursework belongs in clearly named task directories.

## Setup

This repository has no application dependencies or build setup. After cloning, install the repository-owned Git hooks:

```sh
bash scripts/install-hooks.sh
```

The installer sets this repository's local `core.hooksPath` to `.githooks`. It stops without changing the setting if another custom hook path is active.

## Use

Read the [agent navigation map](docs/AGENT_MAP.md) before changing an unfamiliar area. Check documentation links and heading anchors with:

```sh
python3 scripts/check-docs.py
```

The pre-commit hook runs this check only when relevant files are staged. Use `git commit --no-verify` when you intentionally need to bypass the hook.
