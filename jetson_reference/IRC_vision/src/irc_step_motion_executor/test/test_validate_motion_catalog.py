#!/usr/bin/env python3
"""Unit tests for the hardware-free SDK catalog validator."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).parents[1] / "scripts" / "validate_motion_catalog.py"
SPEC = importlib.util.spec_from_file_location(
    "validate_motion_catalog", SCRIPT
)
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


def motor_map(value):
    """Return a complete SDK motor map fixture."""
    return {str(motor_id): value for motor_id in range(23)}


def valid_catalog():
    """Return the smallest catalog satisfying the requested contract."""
    return {
        "motions": [{
            "name": "motion A",
            "completion": {},
            "start_pose": "start",
            "end_pose": "end",
            "frames": [{
                "name": "frame A",
                "angles": motor_map(0.0),
                "torques": motor_map(True),
            }],
        }]
    }


class ValidateMotionCatalogTest(unittest.TestCase):
    """Cover alias, metadata, duplicate, and motor-ID failures."""

    def test_valid_catalog_and_alias_pass(self):
        """Accept a complete catalog with a resolvable alias."""
        errors = VALIDATOR.validate_catalog(
            valid_catalog(), {"forward": "motion A"}
        )
        self.assertEqual(errors, [])

    def test_missing_and_extra_motor_ids_report_motion_and_frame(self):
        """Identify the exact motion/frame for an invalid motor set."""
        catalog = valid_catalog()
        angles = catalog["motions"][0]["frames"][0]["angles"]
        del angles["22"]
        angles["23"] = 0.0

        errors = VALIDATOR.validate_catalog(catalog, {})

        self.assertTrue(any("motion A" in error for error in errors))
        self.assertTrue(any("frame A" in error for error in errors))
        self.assertTrue(any(
            "missing=[22], extra=[23]" in error for error in errors
        ))

    def test_duplicate_metadata_and_missing_alias_target_fail(self):
        """Report duplicates, metadata omissions, and stale aliases."""
        catalog = valid_catalog()
        duplicate = valid_catalog()["motions"][0]
        del duplicate["completion"]
        del duplicate["start_pose"]
        del duplicate["end_pose"]
        catalog["motions"].append(duplicate)

        errors = VALIDATOR.validate_catalog(
            catalog, {"stale": "missing motion"}
        )

        self.assertTrue(any(
            "duplicate motion name" in error for error in errors
        ))
        self.assertTrue(any("missing completion" in error for error in errors))
        self.assertTrue(any("missing start_pose" in error for error in errors))
        self.assertTrue(any("missing end_pose" in error for error in errors))
        self.assertTrue(any(
            "targets missing motion" in error for error in errors
        ))


if __name__ == "__main__":
    unittest.main()
