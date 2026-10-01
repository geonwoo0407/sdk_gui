"""Minimal native TensorRT inference backend for static-shape engines."""

from __future__ import annotations

import ctypes
import ctypes.util
from pathlib import Path
from typing import Any

import numpy as np


class CudaRuntimeError(RuntimeError):
    """Raised when a CUDA Runtime API call fails."""


class _CudaRuntime:
    """Small ctypes wrapper around the CUDA Runtime API."""

    HOST_TO_DEVICE = 1
    DEVICE_TO_HOST = 2

    def __init__(self) -> None:
        library_path = ctypes.util.find_library("cudart") or "libcudart.so"
        try:
            self.library = ctypes.CDLL(library_path)
        except OSError as exc:
            raise RuntimeError(
                f"Could not load the CUDA Runtime library: {exc}"
            ) from exc

        self.library.cudaGetErrorString.argtypes = [ctypes.c_int]
        self.library.cudaGetErrorString.restype = ctypes.c_char_p
        self.library.cudaMalloc.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_size_t,
        ]
        self.library.cudaMalloc.restype = ctypes.c_int
        self.library.cudaFree.argtypes = [ctypes.c_void_p]
        self.library.cudaFree.restype = ctypes.c_int
        self.library.cudaMemcpyAsync.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        self.library.cudaMemcpyAsync.restype = ctypes.c_int
        self.library.cudaStreamCreate.argtypes = [
            ctypes.POINTER(ctypes.c_void_p)
        ]
        self.library.cudaStreamCreate.restype = ctypes.c_int
        self.library.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.library.cudaStreamSynchronize.restype = ctypes.c_int
        self.library.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.library.cudaStreamDestroy.restype = ctypes.c_int

    def _check(self, result: int, operation: str) -> None:
        if result == 0:
            return
        raw_message = self.library.cudaGetErrorString(result)
        message = (
            raw_message.decode("utf-8", errors="replace")
            if raw_message
            else "unknown CUDA error"
        )
        raise CudaRuntimeError(
            f"{operation} failed with CUDA error {result}: {message}"
        )

    def create_stream(self) -> int:
        stream = ctypes.c_void_p()
        self._check(
            self.library.cudaStreamCreate(ctypes.byref(stream)),
            "cudaStreamCreate",
        )
        if stream.value is None:
            raise CudaRuntimeError("cudaStreamCreate returned a null stream")
        return int(stream.value)

    def destroy_stream(self, stream: int) -> None:
        self._check(
            self.library.cudaStreamDestroy(ctypes.c_void_p(stream)),
            "cudaStreamDestroy",
        )

    def synchronize(self, stream: int) -> None:
        self._check(
            self.library.cudaStreamSynchronize(ctypes.c_void_p(stream)),
            "cudaStreamSynchronize",
        )

    def malloc(self, size: int) -> int:
        device_pointer = ctypes.c_void_p()
        self._check(
            self.library.cudaMalloc(ctypes.byref(device_pointer), size),
            "cudaMalloc",
        )
        if device_pointer.value is None:
            raise CudaRuntimeError("cudaMalloc returned a null pointer")
        return int(device_pointer.value)

    def free(self, device_pointer: int) -> None:
        self._check(
            self.library.cudaFree(ctypes.c_void_p(device_pointer)),
            "cudaFree",
        )

    def copy_to_device(
        self,
        device_pointer: int,
        host_array: np.ndarray,
        stream: int,
    ) -> None:
        self._check(
            self.library.cudaMemcpyAsync(
                ctypes.c_void_p(device_pointer),
                ctypes.c_void_p(host_array.ctypes.data),
                host_array.nbytes,
                self.HOST_TO_DEVICE,
                ctypes.c_void_p(stream),
            ),
            "cudaMemcpyAsync host-to-device",
        )

    def copy_to_host(
        self,
        host_array: np.ndarray,
        device_pointer: int,
        stream: int,
    ) -> None:
        self._check(
            self.library.cudaMemcpyAsync(
                ctypes.c_void_p(host_array.ctypes.data),
                ctypes.c_void_p(device_pointer),
                host_array.nbytes,
                self.DEVICE_TO_HOST,
                ctypes.c_void_p(stream),
            ),
            "cudaMemcpyAsync device-to-host",
        )


def _static_shape(raw_shape: Any, tensor_name: str) -> tuple[int, ...]:
    """Return one positive static TensorRT tensor shape."""
    shape = tuple(int(dimension) for dimension in raw_shape)
    if not shape or any(dimension <= 0 for dimension in shape):
        raise RuntimeError(
            f"TensorRT tensor {tensor_name!r} must have a static shape; "
            f"got {shape}"
        )
    return shape


class TensorRTBackend:
    """Load and run one-input, one-output static TensorRT engines."""

    def __init__(self, model_path: Path) -> None:
        try:
            import tensorrt as trt
        except ImportError as exc:
            raise RuntimeError(
                "TensorRT Python bindings are required for .engine models"
            ) from exc

        self._trt = trt
        self._cuda: _CudaRuntime | None = None
        self._stream: int | None = None
        self._device_pointers: list[int] = []
        self._closed = False

        self._logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self._logger, "")
        self._runtime = trt.Runtime(self._logger)
        serialized_engine = model_path.read_bytes()
        self._engine = self._runtime.deserialize_cuda_engine(
            serialized_engine
        )
        if self._engine is None:
            raise RuntimeError(
                f"TensorRT could not deserialize engine: {model_path}"
            )

        input_names: list[str] = []
        output_names: list[str] = []
        for index in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(index)
            mode = self._engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                input_names.append(name)
            elif mode == trt.TensorIOMode.OUTPUT:
                output_names.append(name)

        if len(input_names) != 1 or len(output_names) != 1:
            raise RuntimeError(
                "TensorRT backend requires exactly one input and one output; "
                f"got inputs={input_names}, outputs={output_names}"
            )

        self.input_name = input_names[0]
        self.output_name = output_names[0]
        self.input_shape = _static_shape(
            self._engine.get_tensor_shape(self.input_name),
            self.input_name,
        )
        self.output_shape = _static_shape(
            self._engine.get_tensor_shape(self.output_name),
            self.output_name,
        )
        self.input_dtype = np.dtype(
            trt.nptype(self._engine.get_tensor_dtype(self.input_name))
        )
        self.output_dtype = np.dtype(
            trt.nptype(self._engine.get_tensor_dtype(self.output_name))
        )

        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError(
                "TensorRT could not create an execution context"
            )

        try:
            self._cuda = _CudaRuntime()
            self._stream = self._cuda.create_stream()
            self._input_device = self._cuda.malloc(
                int(np.prod(self.input_shape)) * self.input_dtype.itemsize
            )
            self._device_pointers.append(self._input_device)
            self._output = np.empty(
                self.output_shape,
                dtype=self.output_dtype,
            )
            self._output_device = self._cuda.malloc(self._output.nbytes)
            self._device_pointers.append(self._output_device)

            if not self._context.set_tensor_address(
                self.input_name,
                self._input_device,
            ):
                raise RuntimeError(
                    f"Could not bind TensorRT input {self.input_name!r}"
                )
            if not self._context.set_tensor_address(
                self.output_name,
                self._output_device,
            ):
                raise RuntimeError(
                    f"Could not bind TensorRT output {self.output_name!r}"
                )
        except Exception:
            self.close()
            raise

    def infer(self, input_array: np.ndarray) -> np.ndarray:
        """Run one synchronous inference and return the reusable output."""
        if self._closed or self._cuda is None or self._stream is None:
            raise RuntimeError("TensorRT backend is closed")
        if tuple(input_array.shape) != self.input_shape:
            raise ValueError(
                f"TensorRT input shape must be {self.input_shape}; "
                f"got {tuple(input_array.shape)}"
            )

        contiguous_input = np.ascontiguousarray(
            input_array,
            dtype=self.input_dtype,
        )
        self._cuda.copy_to_device(
            self._input_device,
            contiguous_input,
            self._stream,
        )
        if not self._context.execute_async_v3(self._stream):
            raise RuntimeError("TensorRT execute_async_v3 failed")
        self._cuda.copy_to_host(
            self._output,
            self._output_device,
            self._stream,
        )
        self._cuda.synchronize(self._stream)
        return self._output

    def close(self) -> None:
        """Release CUDA allocations owned by this backend."""
        if self._closed:
            return
        self._closed = True
        if self._cuda is None:
            return
        if self._stream is not None:
            try:
                self._cuda.synchronize(self._stream)
            except CudaRuntimeError:
                pass
        for device_pointer in reversed(self._device_pointers):
            try:
                self._cuda.free(device_pointer)
            except CudaRuntimeError:
                pass
        self._device_pointers.clear()
        if self._stream is not None:
            try:
                self._cuda.destroy_stream(self._stream)
            except CudaRuntimeError:
                pass
            self._stream = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
