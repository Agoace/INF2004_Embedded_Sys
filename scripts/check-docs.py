#!/usr/bin/env python3
"""Check repository Markdown links and GitHub-style heading anchors.

The standard-library-only checker scans Markdown files at the repository root
and below ``docs/``. The repository has no component documentation locations.
Dependency, build, vendor, and Git metadata directories are outside this scope.
The check is mechanical: it does not verify prose, commands, or live state.
"""
from __future__ import annotations

import re
import sys
import unicodedata
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parent.parent
ATX_HEADING_RE = re.compile(r"^\s{0,3}(#{1,6})\s+(.+?)\s*#*\s*$")
SETEXT_RE = re.compile(r"^\s{0,3}(=+|-+)\s*$")
INLINE_LINK_RE = re.compile(
    r"(?<!!)\[[^\]]*\]\(\s*(<[^>]+>|[^\s)]+)"
    r"(?:\s+(?:\"[^\"]*\"|'[^']*'|\([^)]*\)))?\s*\)"
)
REFERENCE_LINK_RE = re.compile(r"^\s{0,3}\[[^\]]+\]:\s*(<[^>]+>|\S+)")
FENCE_RE = re.compile(r"^\s{0,3}(`{3,}|~{3,})")


def github_slug(text: str) -> str:
    """Return the GitHub-style slug needed by this repository's headings."""
    text = re.sub(r"<[^>]+>", "", text)
    text = re.sub(r"!?\[([^\]]*)\]\([^)]*\)", r"\1", text)
    text = text.replace("`", "").strip().lower()
    kept = []
    for character in text:
        category = unicodedata.category(character)
        if character in "-_" or character.isspace() or category[0] in {"L", "N"}:
            kept.append(character)
    return re.sub(r"\s", "-", "".join(kept))


def headings(path: Path) -> set[str]:
    anchors: set[str] = set()
    counts: dict[str, int] = {}
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    in_fence = False
    fence_character = ""
    previous = ""

    for line in lines:
        fence = FENCE_RE.match(line)
        if fence:
            marker = fence.group(1)
            if not in_fence:
                in_fence = True
                fence_character = marker[0]
            elif marker[0] == fence_character:
                in_fence = False
            previous = ""
            continue
        if in_fence:
            continue

        match = ATX_HEADING_RE.match(line)
        heading = match.group(2) if match else ""
        if not heading and previous.strip() and SETEXT_RE.match(line):
            heading = previous.strip()
        if heading:
            base = github_slug(heading)
            count = counts.get(base, 0)
            counts[base] = count + 1
            anchors.add(base if count == 0 else f"{base}-{count}")
        previous = line
    return anchors


def scoped_files() -> list[Path]:
    files = set(ROOT.glob("*.md"))
    docs = ROOT / "docs"
    if docs.is_dir():
        files.update(docs.rglob("*.md"))
    return sorted(path for path in files if path.is_file())


def links(line: str) -> list[str]:
    found = [match.group(1) for match in INLINE_LINK_RE.finditer(line)]
    reference = REFERENCE_LINK_RE.match(line)
    if reference:
        found.append(reference.group(1))
    return found


def check(files: list[Path]) -> list[str]:
    problems: list[str] = []
    anchor_cache: dict[Path, set[str]] = {}

    for document in files:
        in_fence = False
        fence_character = ""
        for line_number, line in enumerate(
            document.read_text(encoding="utf-8", errors="replace").splitlines(), 1
        ):
            fence = FENCE_RE.match(line)
            if fence:
                marker = fence.group(1)
                if not in_fence:
                    in_fence = True
                    fence_character = marker[0]
                elif marker[0] == fence_character:
                    in_fence = False
                continue
            if in_fence:
                continue

            for raw_target in links(line):
                target_text = raw_target.strip("<>")
                parsed = urlsplit(target_text)
                if parsed.scheme or parsed.netloc or target_text.startswith("/"):
                    continue

                target = (
                    (document.parent / unquote(parsed.path)).resolve()
                    if parsed.path
                    else document.resolve()
                )
                if not target.exists():
                    location = document.relative_to(ROOT)
                    problems.append(
                        f"{location}:{line_number}: broken link -> {target_text!r}"
                    )
                    continue

                if parsed.fragment and target.suffix.lower() in {".md", ".mdx"}:
                    available = anchor_cache.setdefault(target, headings(target))
                    fragment = unquote(parsed.fragment).lower()
                    if fragment not in available:
                        location = document.relative_to(ROOT)
                        problems.append(
                            f"{location}:{line_number}: broken anchor -> {target_text!r}"
                        )
    return problems


def main() -> int:
    files = scoped_files()
    problems = check(files)
    if problems:
        print(f"check-docs.py: {len(problems)} problem(s) found:")
        for problem in problems:
            print(f"  {problem}")
        return 1

    print(
        f"check-docs.py: OK — {len(files)} files scanned, "
        "0 broken links, 0 broken anchors."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
