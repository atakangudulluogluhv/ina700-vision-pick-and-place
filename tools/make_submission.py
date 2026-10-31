"""Build the hand-in ZIP the assignment demands after the examination (Stage G).

The assignment accepts exactly one file, FinalProject_<StudentName>.zip, holding
the complete program files, the trained AI model and a Pack-and-Go file of the
robot station taken at the examination. A submission that misses any of them
"will be reported as failed. No resubmission is allowed" (T9), so this tool
refuses to write the real archive unless every part is present, and re-opens
what it wrote to prove it.

The Pack-and-Go has to come from the examination itself (T15), which is why it
is a required argument and never picked up from disk by itself.

    python tools/make_submission.py --pack-and-go D:/exam/INA700_station.rspag
    python tools/make_submission.py --dry-run          # rehearsal, no Pack-and-Go

Run it from the copy that matches what ran in the lab: pull the Pi's changes
back to this machine first, or pass --source with the folder holding them.
"""

import argparse
import hashlib
import re
import sys
import zipfile
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.core.config import Config  # noqa: E402

DEFAULT_STUDENT_NAME = "AtakanGudulluoglu"
ARCHIVE_PREFIX = "FinalProject_"
DRY_RUN_SUFFIX = "_DRYRUN"
PACK_AND_GO_SUFFIX = ".rspag"
PACK_AND_GO_FOLDER = "pack_and_go"
MANIFEST_NAME = "MANIFEST.txt"
SETTINGS_RELATIVE_PATH = "config/settings.yaml"
MD5_READ_CHUNK_BYTES = 1 << 20

# The grader runs the Pi program and loads the RAPID side; both need these.
INCLUDED_FILES = ("README.md", "requirements-pi.txt")
INCLUDED_FOLDERS = ("config", "src", "tools", "tests", "models", "docs")
# Only the module that runs on the robot, plus the course socket layer it calls.
# Anything else under robot/ is course reference material, not our program.
ROBOT_FILES = ("robot/MainModule.mod", "robot/ServerCommuncatio.sys")

# Assistant notes, the personal exam-day card and crops of the assignment PDF
# are not program files.
EXCLUDED_NAMES = {"__pycache__", ".pytest_cache", "pi_CLAUDE.md", "CLAUDE.md"}
EXCLUDED_PATTERNS = ("docs/SINAV_GUNU.md", "docs/figures/fig*_*.jpg", "docs/figures/crop_*.png")

# Without these the archive cannot be graded at all; checked inside the ZIP.
REQUIRED_MEMBERS = (
    "src/app/station.py",
    SETTINGS_RELATIVE_PATH,
    "robot/MainModule.mod",
    "robot/ServerCommuncatio.sys",
)
STUDENT_NAME_PATTERN = re.compile(r"^[A-Za-z0-9]+$")


def md5_of(path: Path) -> str:
    """Hex MD5 of a file, read in chunks so the Pack-and-Go fits in memory."""
    digest = hashlib.md5()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(MD5_READ_CHUNK_BYTES), b""):
            digest.update(chunk)
    return digest.hexdigest()


def is_excluded(relative_path: Path) -> bool:
    if any(part in EXCLUDED_NAMES for part in relative_path.parts):
        return True
    return any(relative_path.match(pattern) for pattern in EXCLUDED_PATTERNS)


def collect_program_files(source_root: Path) -> list[Path]:
    """Relative paths of every program file to ship, sorted for a stable ZIP."""
    relative_paths = [Path(name) for name in INCLUDED_FILES + ROBOT_FILES]
    for folder in INCLUDED_FOLDERS:
        for path in (source_root / folder).rglob("*"):
            if path.is_file() and path.suffix != ".pyc":
                relative_paths.append(path.relative_to(source_root))
    kept = [path for path in relative_paths if not is_excluded(path)]
    missing = [str(path) for path in kept if not (source_root / path).is_file()]
    if missing:
        raise FileNotFoundError(f"program files missing under {source_root}: {missing}")
    return sorted(set(kept))


def model_relative_path(source_root: Path) -> str:
    """The ONNX file the program actually loads, so the shipped model is the used one."""
    settings = Config.load(source_root / SETTINGS_RELATIVE_PATH)
    return Path(settings.get("yolo.model_path")).as_posix()


def archive_name(student_name: str, dry_run: bool) -> str:
    if not STUDENT_NAME_PATTERN.match(student_name):
        raise ValueError(f"student name must be ASCII letters/digits only: {student_name!r}")
    suffix = DRY_RUN_SUFFIX if dry_run else ""
    return f"{ARCHIVE_PREFIX}{student_name}{suffix}.zip"


def validate_pack_and_go(path: Path | None, dry_run: bool) -> Path | None:
    if path is None:
        if dry_run:
            return None
        raise ValueError("--pack-and-go is required; the station must be packed at the examination")
    if not path.is_file():
        raise FileNotFoundError(f"Pack-and-Go file not found: {path}")
    if path.suffix.lower() != PACK_AND_GO_SUFFIX:
        raise ValueError(f"expected a RobotStudio {PACK_AND_GO_SUFFIX} file, got {path.name}")
    return path


def build_manifest(entries: list[tuple[str, Path]], source_root: Path) -> str:
    """Archive path + MD5 per file, so the shipped Pi state can be compared later."""
    lines = [
        f"Built {datetime.now().isoformat(timespec='seconds')} from {source_root}",
        "",
        "md5                               archive path",
    ]
    lines += [f"{md5_of(source)}  {member}" for member, source in entries]
    return "\n".join(lines) + "\n"


def write_archive(output_path: Path, entries: list[tuple[str, Path]], manifest: str) -> None:
    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for member, source in entries:
            archive.write(source, member)
        archive.writestr(MANIFEST_NAME, manifest)


def verify_archive(output_path: Path, required: list[str]) -> list[str]:
    """Re-open the written ZIP; return what is wrong with it (empty list = good)."""
    problems = []
    with zipfile.ZipFile(output_path) as archive:
        corrupt_member = archive.testzip()
        if corrupt_member is not None:
            problems.append(f"CRC check failed for {corrupt_member}")
        names = set(archive.namelist())
    problems += [f"missing from archive: {member}" for member in required if member not in names]
    return problems


def plan_entries(source_root: Path, pack_and_go: Path | None) -> tuple[list[tuple[str, Path]], list[str]]:
    """(archive path, source file) pairs plus the members that must be present."""
    program_files = collect_program_files(source_root)
    entries = [(path.as_posix(), source_root / path) for path in program_files]
    required = list(REQUIRED_MEMBERS) + [model_relative_path(source_root)]
    if pack_and_go is not None:
        pack_member = f"{PACK_AND_GO_FOLDER}/{pack_and_go.name}"
        entries.append((pack_member, pack_and_go))
        required.append(pack_member)
    return entries, required


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build the INA700 hand-in ZIP.")
    parser.add_argument("--pack-and-go", type=Path, default=None,
                        help="RobotStudio .rspag saved at the examination")
    parser.add_argument("--source", type=Path, default=PROJECT_ROOT,
                        help="project folder to pack (default: this repository)")
    parser.add_argument("--student", default=DEFAULT_STUDENT_NAME)
    parser.add_argument("--output-dir", type=Path, default=Path.cwd())
    parser.add_argument("--dry-run", action="store_true",
                        help="rehearse without a Pack-and-Go; the ZIP name is marked DRYRUN")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    source_root = args.source.resolve()
    pack_and_go = validate_pack_and_go(args.pack_and_go, args.dry_run)
    entries, required = plan_entries(source_root, pack_and_go)
    model_path = source_root / model_relative_path(source_root)

    output_path = args.output_dir.resolve() / archive_name(args.student, args.dry_run)
    write_archive(output_path, entries, build_manifest(entries, source_root))
    problems = verify_archive(output_path, required)

    print(f"[make_submission] {len(entries)} files -> {output_path}")
    print(f"[make_submission] model    {model_path.name}  md5 {md5_of(model_path)}")
    print(f"[make_submission] settings md5 {md5_of(source_root / SETTINGS_RELATIVE_PATH)}")
    if problems:
        for problem in problems:
            print(f"[make_submission] ERROR {problem}")
        return 1
    if pack_and_go is None:
        print("[make_submission] DRY RUN: no Pack-and-Go inside, do NOT hand this file in")
    else:
        print(f"[make_submission] OK, ready to hand in ({output_path.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
