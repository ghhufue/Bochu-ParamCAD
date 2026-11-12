"""Image-only structured-light reconstruction, independent of the renderer."""

__version__ = "1.0.0"


class StructuredLightError(ValueError):
    """Invalid acquisition, calibration or reconstruction parameters."""
