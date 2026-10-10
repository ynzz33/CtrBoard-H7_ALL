"""Restore migrated source/include directories without rewriting other settings."""
from pathlib import Path


def main():
    config = Path(__file__).resolve().parents[1] / ".eide/eide.yml"
    original = config.read_text(encoding="utf-8")
    lines = []
    for line in original.splitlines(keepends=True):
        item = line.strip()
        if item == "- imcalib/user-lib":
            indent = line[:len(line) - len(line.lstrip())]
            lines.extend(indent + "- imcalib/" + directory + "\n"
                         for directory in ("drivers", "bsp", "common", "config"))
        elif item not in ("- imcalib/Telemetry", "- imcalib/Sysid"):
            lines.append(line)
    updated = "".join(lines)
    if updated != original:
        config.write_text(updated, encoding="utf-8")
    print("eIDE directories updated. Accept the project's reload prompt before building.")


if __name__ == "__main__":
    main()
