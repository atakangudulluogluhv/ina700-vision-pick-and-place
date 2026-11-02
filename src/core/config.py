"""Loading and dotted-path access for config/settings.yaml.

Keeping every tunable value in one YAML file is what lets the lab session be
spent on calibration instead of on editing source code.
"""

import re
from pathlib import Path
from typing import Any

import yaml

DEFAULT_SETTINGS_PATH = Path(__file__).resolve().parents[2] / "config" / "settings.yaml"

_MISSING = object()


class ConfigError(KeyError):
    """Raised when a requested settings key does not exist."""


def write_scalar_values(path: Path | str, values: dict[str, float], section: str | None = None) -> None:
    """Overwrite numeric leaf values in a settings file in place.

    Line by line instead of yaml.dump, so the comments in the file survive.
    Each key must appear exactly once (within `section`, the block under that
    key, when given).
    """
    settings_path = Path(path)
    full_text = settings_path.read_text(encoding="utf-8")
    head, text, tail = _split_section(full_text, section)
    for key, value in values.items():
        pattern = re.compile(r"^([ \t]*" + re.escape(key) + r":[ \t]*)[-+0-9.eE]+", re.MULTILINE)
        text, count = pattern.subn(lambda match: match.group(1) + f"{float(value):.2f}", text)
        if count != 1:
            raise ConfigError(f"{key} found {count} times in {settings_path}, expected once")
    settings_path.write_text(head + text + tail, encoding="utf-8")


def _split_section(text: str, section: str | None) -> tuple[str, str, str]:
    """(before, block, after): block = the more-indented lines under `section:`."""
    if section is None:
        return "", text, ""
    header = re.search(r"^([ \t]*)" + re.escape(section) + r":[ \t]*(#.*)?$", text, re.MULTILINE)
    if header is None:
        raise ConfigError(f"section {section} not found")
    indent = len(header.group(1))
    start = header.end() + 1
    end = start
    for line in text[start:].splitlines(keepends=True):
        stripped = line.strip()
        if stripped and not stripped.startswith("#") and len(line) - len(line.lstrip()) <= indent:
            break
        end += len(line)
    return text[:start], text[start:end], text[end:]


class Config:
    """Read-only view over the settings tree, addressed by dotted paths."""

    def __init__(self, values: dict[str, Any]) -> None:
        self._values = values

    @classmethod
    def load(cls, path: Path | str | None = None) -> "Config":
        """Read a YAML settings file; falls back to config/settings.yaml."""
        settings_path = Path(path) if path is not None else DEFAULT_SETTINGS_PATH
        if not settings_path.is_file():
            raise ConfigError(f"settings file not found: {settings_path}")
        with settings_path.open("r", encoding="utf-8") as handle:
            values = yaml.safe_load(handle) or {}
        return cls(values)

    def get(self, dotted_key: str, default: Any = _MISSING) -> Any:
        """Return the value at 'a.b.c'; raise unless a default is supplied."""
        node: Any = self._values
        for part in dotted_key.split("."):
            if not isinstance(node, dict) or part not in node:
                if default is _MISSING:
                    raise ConfigError(f"missing settings key: {dotted_key}")
                return default
            node = node[part]
        return node

    def section(self, dotted_key: str) -> "Config":
        """Return a sub-tree as its own Config, so callers can pass it around."""
        node = self.get(dotted_key)
        if not isinstance(node, dict):
            raise ConfigError(f"settings key is not a section: {dotted_key}")
        return Config(node)

    def keys(self) -> list[str]:
        """Top level key names of this (sub-)tree."""
        return list(self._values.keys())

    def as_dict(self) -> dict[str, Any]:
        """Shallow copy of the underlying mapping."""
        return dict(self._values)

    def __contains__(self, dotted_key: str) -> bool:
        return self.get(dotted_key, None) is not None

    def __str__(self) -> str:
        return f"Config(top_level_keys={self.keys()})"
