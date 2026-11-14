"""Task 0 smoke tests: the package imports and the skeleton is coherent."""

from pathlib import Path

import pytest
import yaml

import rada_align
from rada_align import interfaces

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_package_imports():
    assert rada_align.__version__


@pytest.mark.parametrize(
    "cls",
    [
        interfaces.Camera,
        interfaces.Inclinometer,
        interfaces.Rangefinder,
        interfaces.Actuators,
    ],
)
def test_interfaces_are_abstract(cls):
    with pytest.raises(TypeError):
        cls()


def test_axes_cover_the_four_axis_mount():
    assert set(interfaces.AXES) == {"x", "z", "pan", "tilt"}


def test_config_loads_with_valid_mode():
    config = yaml.safe_load((REPO_ROOT / "config.yaml").read_text(encoding="utf-8"))
    assert config["mode"] in ("sim", "hw")
