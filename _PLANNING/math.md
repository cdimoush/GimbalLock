# Fitting the data sheet

The linear fit works because it uses the **closed-form least-squares solution** for fitting a line `y = m*x + b` to data points.

Here's the mathematical breakdown:

## The Least-Squares Formula

Given points `(x₁, y₁), (x₂, y₂), ..., (xₙ, yₙ)`, the slope and intercept that minimize squared error are:

```
m = Σ[(xᵢ - x̄)(yᵢ - ȳ)] / Σ[(xᵢ - x̄)²]
b = ȳ - m·x̄
```

where `x̄` and `ȳ` are the means of x and y values.

## Why This Works

**Geometrically**: This finds the line that minimizes the sum of squared vertical distances from each point to the line.

**Algebraically**: The numerator `Σ[(xᵢ - x̄)(yᵢ - ȳ)]` is the **covariance** between x and y, and the denominator `Σ[(xᵢ - x̄)²]` is the **variance** of x. So:

```
m = Cov(x,y) / Var(x)
```

This is the **standard formula for simple linear regression**.

## In Your Code

```python
x_mean = x.mean()
y_mean = y.mean()
m = torch.sum((x - x_mean) * (y - y_mean)) / torch.sum((x - x_mean) ** 2)
b = y_mean - m * x_mean
```

This directly implements the formulas above using PyTorch operations, computing everything on-device (GPU or CPU) in a vectorized way.

## Edge Cases I Handle

1. **Single point**: Can't fit a slope, so `m=0, b=y[0]` (horizontal line through the point)
2. **Vertical points** (all x values identical): `denom ≈ 0`, so default to `m=0` to avoid division by zero
3. **Degenerate mapping**: Return zeros

The fit happens **on-the-fly every time** `_compute_gap_target` is called, which is slightly inefficient but flexible. If the mapping is constant, you could cache `m` and `b` in `_initialize_impl()` for better performance.