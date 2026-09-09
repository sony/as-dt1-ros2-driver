"""
pytest configuration for hardware-in-the-loop tests.

Tests in this directory require a physical AS-DT1 device. Guard them with
the --hw flag so they are skipped in CI environments without hardware.

Usage:
    pytest test/ --hw                         # run hardware tests
    pytest test/                              # skip hardware tests (default)
"""

import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--hw",
        action="store_true",
        default=False,
        help="Run hardware-in-the-loop tests (requires AS-DT1 device)",
    )


def pytest_collection_modifyitems(config, items):
    if config.getoption("--hw"):
        return
    skip_hw = pytest.mark.skip(reason="Pass --hw to run hardware-in-the-loop tests")
    for item in items:
        # Skip everything in the hardware/ directory unless --hw is passed
        if "hardware" in str(item.fspath):
            item.add_marker(skip_hw)
