"""Run flake8 checks for the mission_control package."""

from pathlib import Path

import pytest
from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Check code style using the package flake8 configuration."""
    package_root = Path(__file__).resolve().parents[1]
    config_path = package_root / 'setup.cfg'

    return_code, errors = main_with_errors(
        argv=[
            '--config',
            str(config_path),
            str(package_root / 'setup.py'),
            str(package_root / 'launch'),
            str(package_root / 'mission_control'),
            str(package_root / 'test'),
        ]
    )

    assert return_code == 0
    assert errors == []