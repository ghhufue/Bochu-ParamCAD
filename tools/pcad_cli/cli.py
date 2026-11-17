"""Argument parsing and dispatch; each command's implementation lives separately."""

import argparse
from collections.abc import Sequence
import sys

from . import clean, native


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="pcad", description="ParamCAD repository tools")
    commands = parser.add_subparsers(dest="command", required=True)

    cleanup = commands.add_parser("clean", help="clean generated files")
    targets = cleanup.add_subparsers(dest="target", required=True)
    output = targets.add_parser("output", help="remove generated entries from repository outputs/")
    output.add_argument("--dry-run", action="store_true", help="preview cleanup without deleting files")
    output.set_defaults(handler=clean.run)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    try:
        if arguments[:2] == ["clean", "output"]:
            args = build_parser().parse_args(arguments)
            return args.handler(args)
        return native.run(arguments)
    except (OSError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
