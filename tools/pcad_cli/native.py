"""Find the CLI registered by the most recent CMake build; never start Studio."""
import json
import os
from pathlib import Path
import subprocess
from .paths import REPOSITORY_ROOT


def locate_cli() -> Path:
    override = os.environ.get("PCAD_EXECUTABLE")
    if override:
        executable = Path(override).expanduser().resolve()
        if executable.is_file():
            return executable
        raise RuntimeError(f"PCAD_EXECUTABLE does not exist: {executable}")
    registration = REPOSITORY_ROOT / "artifacts/cli-runtime.json"
    if registration.is_file():
        try:
            data = json.loads(registration.read_text(encoding="utf-8"))
            executable = Path(data["executable"])
            if data.get("version") == 1 and executable.is_absolute() and executable.is_file():
                return executable
        except (OSError, ValueError, KeyError, TypeError):
            pass
    executable = REPOSITORY_ROOT / "apps/ParamCAD Studio/build/msvc-debug/Release/pcad.exe"
    if executable.is_file():
        return executable
    raise RuntimeError("pcad.exe not found. Build Studio with CAD_CLIENT_BUILD_MODULE_CLIS=ON, or set PCAD_EXECUTABLE.")


def run(arguments) -> int:
    # Preserve the caller's cwd, streams and the native exit code.
    return subprocess.run([str(locate_cli()), *arguments], check=False).returncode
