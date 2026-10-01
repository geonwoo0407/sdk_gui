"""Unit tests for TensorRT backend validation helpers."""

import pytest

from step.tensorrt_backend import _static_shape


def test_static_shape_accepts_positive_dimensions():
    assert _static_shape((1, 3, 640, 640), "images") == (1, 3, 640, 640)


@pytest.mark.parametrize("shape", [(), (-1, 3, 640, 640), (1, 0, 6)])
def test_static_shape_rejects_dynamic_or_empty_dimensions(shape):
    with pytest.raises(RuntimeError, match="must have a static shape"):
        _static_shape(shape, "images")
