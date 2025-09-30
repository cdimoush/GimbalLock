#!/usr/bin/env python3
"""
PyTorch functionality test with GPU support for GimbalLock project.
Tests basic tensor operations, matrix operations, and gradient computation.
"""

import torch
import sys


def test_pytorch_basic():
    """Test basic PyTorch functionality"""
    print("=" * 60)
    print("PyTorch Environment Test")
    print("=" * 60)

    # Environment info
    print(f"PyTorch version: {torch.__version__}")
    print(f"Python version: {sys.version}")
    print(f"CUDA available: {torch.cuda.is_available()}")

    if torch.cuda.is_available():
        print(f"CUDA version: {torch.version.cuda}")
        print(f"GPU device count: {torch.cuda.device_count()}")
        print(f"Current GPU: {torch.cuda.get_device_name()}")
        device = torch.device("cuda")
    else:
        print("Using CPU device")
        device = torch.device("cpu")

    print(f"Using device: {device}")
    print()


def test_tensor_operations(device):
    """Test basic tensor operations"""
    print("Testing tensor operations...")

    # Basic tensor creation and operations
    x = torch.tensor([1.0, 2.0, 3.0], device=device)
    y = torch.tensor([4.0, 5.0, 6.0], device=device)
    z = x + y

    print(f"Tensor addition: {x} + {y} = {z}")
    print(f"Tensor device: {z.device}")

    # Element-wise operations
    w = torch.sin(x) + torch.cos(y)
    print(f"Sin/cos operations: {w}")
    print()


def test_matrix_operations(device):
    """Test matrix operations"""
    print("Testing matrix operations...")

    # Matrix multiplication
    A = torch.randn(3, 4, device=device)
    B = torch.randn(4, 2, device=device)
    C = torch.mm(A, B)

    print(f"Matrix A shape: {A.shape}")
    print(f"Matrix B shape: {B.shape}")
    print(f"Matrix C = A @ B shape: {C.shape}")
    print(f"Matrix C device: {C.device}")

    # Batch matrix multiplication
    batch_A = torch.randn(10, 3, 4, device=device)
    batch_B = torch.randn(10, 4, 2, device=device)
    batch_C = torch.bmm(batch_A, batch_B)

    print(f"Batch matrix multiplication shape: {batch_C.shape}")
    print()


def test_gradients(device):
    """Test gradient computation"""
    print("Testing gradient computation...")

    # Single variable
    x = torch.tensor(2.0, requires_grad=True, device=device)
    y = x**2 + 3*x + 1
    y.backward()

    print(f"f(x) = x² + 3x + 1")
    print(f"f'(2) = {x.grad} (expected: 7)")

    # Multi-variable with neural network-like computation
    x1 = torch.tensor([[1.0, 2.0]], requires_grad=True, device=device)
    x2 = torch.tensor([[3.0, 4.0]], requires_grad=True, device=device)

    # Simple "neural network" forward pass
    w = torch.tensor([[0.5, 0.3], [0.2, 0.7]], requires_grad=True, device=device)
    b = torch.tensor([[0.1, 0.2]], requires_grad=True, device=device)

    h1 = torch.mm(x1, w) + b  # Hidden layer
    h2 = torch.mm(x2, w) + b
    output = torch.sum(h1 * h2)  # Simple loss

    output.backward()

    print(f"Neural network-like computation output: {output.item():.4f}")
    print(f"Weight gradients shape: {w.grad.shape}")
    print(f"Bias gradients: {b.grad}")
    print()


def test_gpu_memory_operations(device):
    """Test GPU memory operations if CUDA is available"""
    if device.type != "cuda":
        print("Skipping GPU memory tests (CPU mode)")
        return

    print("Testing GPU memory operations...")

    # Large tensor operations
    large_tensor = torch.randn(1000, 1000, device=device)
    result = torch.mm(large_tensor, large_tensor.t())

    print(f"Large matrix multiplication completed: {result.shape}")
    print(f"GPU memory allocated: {torch.cuda.memory_allocated() / 1024**2:.2f} MB")
    print(f"GPU memory cached: {torch.cuda.memory_reserved() / 1024**2:.2f} MB")

    # Clean up
    del large_tensor, result
    torch.cuda.empty_cache()
    print("GPU memory cleaned up")
    print()


def main():
    """Main test function"""
    test_pytorch_basic()

    # Determine device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Run tests
    test_tensor_operations(device)
    test_matrix_operations(device)
    test_gradients(device)
    test_gpu_memory_operations(device)

    print("=" * 60)
    print("All PyTorch tests completed successfully! ✅")
    print("=" * 60)


if __name__ == "__main__":
    main()