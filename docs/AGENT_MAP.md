# Agent Navigation Map — INF2004 Embedded Systems

Use this map before editing an unfamiliar part of the repository.

## Map chain

| Scope | Map or rule |
| --- | --- |
| Current repository | This file: `docs/AGENT_MAP.md` |
| Parent workspace | [School Repos map](../../../docs/AGENT_MAP.md) |

The parent-workspace link resolves only when this repository is checked out as a School Repos submodule. It does not resolve in a standalone clone.

## Documentation locations

| Content | Location |
| --- | --- |
| Documentation index | [docs/README.md](README.md) |
| Repository instructions | [AGENTS.md](../AGENTS.md) |
| Public orientation and setup | [README.md](../README.md) |
| Parent checkout ledger | [School Repos checkout ledger](../../../docs/README.md#coursework-checkout-ledger) |

## Implementation locations

| Area | Start here |
| --- | --- |
| Coursework orientation | [README.md](../README.md) |
| Documentation checker | [scripts/check-docs.py](../scripts/check-docs.py) |
| Hook installer | [scripts/install-hooks.sh](../scripts/install-hooks.sh) |
| Pre-commit gate | [.githooks/pre-commit](../.githooks/pre-commit) |

## Repository boundaries

| Boundary | Rule |
| --- | --- |
| Current repository | This independent Git repository owns all files below its root. It has no nested repositories or submodules. |
| Parent superproject checkout | The School Repos superproject tracks this repository as a gitlink. Commit and push here first; then update the gitlink in the parent repository. |
