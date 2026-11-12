"""Client-owned acquisition renderer worker. Not a separate capture application."""
from structured_light.mitsuba_renderer import main

if __name__ == "__main__":
    raise SystemExit(main())
