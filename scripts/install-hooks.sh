#!/usr/bin/env bash
# Install this repository's hooks without replacing another custom hook path.
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
hooks_dir="$root/.githooks"

if [ ! -d "$hooks_dir" ]; then
  echo "install-hooks.sh: $hooks_dir does not exist." >&2
  exit 1
fi

current="$(git config --get core.hooksPath || true)"
case "$current" in
  ""|.githooks|"$hooks_dir") ;;
  *)
    echo "install-hooks.sh: core.hooksPath is already '$current'; no change made." >&2
    echo "Review the existing hook setup before replacing it with '$hooks_dir'." >&2
    exit 1
    ;;
esac

git config --local core.hooksPath .githooks
printf 'install-hooks.sh: core.hooksPath is .githooks for %s.\n' "$root"
