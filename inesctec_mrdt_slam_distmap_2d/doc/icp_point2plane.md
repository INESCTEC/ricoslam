# ICP: Point-to-Plane Error Formulation

## State Space

- State domain:
  - $\textbf{X} \in SE\left(2\right)$ or $SE\left(3\right), $
    $\left[\textbf{R}|\textbf{t}\right]$
  - Homogeneous transformation of the moving w.r.t. the fixed point cloud
- Euclidean parametrization of the state perturbation:
  - $\Delta \textbf{x} = \left[\Delta x,\Delta y,\Delta \theta\right]^T \in $
    $\mathbb{R}^3$ or
    $\left[\Delta x,\Delta y,\Delta z, \Delta \alpha_x,\Delta \alpha_y,\Delta \alpha_z\right]^T$
    $\in \mathbb{R}^6$
- Convert the perturbation into the state domain:
  $\Delta \textbf{X} = v2t\left(\Delta \textbf{x}\right) = $
  $\left[\Delta \textbf{R} | \Delta \textbf{t}\right]$
- Boxplus operator: $\textbf{X} \boxplus \Delta \textbf{x}$
  - Left-sided perturbation update: $\textbf{X} \boxplus \Delta \textbf{x} = $
    $v2t\left(\Delta \textbf{x}\right) \cdot \textbf{X} = $
    $\Delta \textbf{X} \cdot \textbf{X}$
  - Right-sided perturbation update: $\textbf{X} \boxplus \Delta \textbf{x} = $
    $\textbf{X} \cdot v2t\left(\Delta \textbf{x}\right) = $
    $\textbf{X} \cdot \Delta \textbf{X}$

## Measurements Space

- Measurement domain:
  - $\textbf{d}_i \in \mathbb{R}^2$ or $\mathbb{R}^3$
    (fixed point obtained from the environment, e.g., through a range sensor,
    such as a 2D or 3D LiDAR)
  - $\textbf{n}_i \in \mathbb{R}^2$ or $\mathbb{R}^3$
    (normal vector, where $\lVert\textbf{n}_i\rVert=1$, computed or perceived at
    the fixed point $\textbf{d}_i$ obtained from the environment)
- Error formulation:
  - $\textbf{e}^{\left[i\right]}\left(\textbf{X}\right) =$
    $\left(X\textbf{p}^{\left[i\right]} - \textbf{d}_i\right)\bullet \textbf{n}_i =$
    $\textbf{n}_i^T\left(X\textbf{p}^{\left[i\right]} - \textbf{d}_i\right)$
    $\in \mathbb{R}^1$
  - $\textbf{p}^{\left[i\right]}$ is the moving point obtained from the
    environment, e.g., through a range sensor (2D or 3D LiDAR), from the latest
    scan (assuming a known association between the moving and fixed points)

## Jacobian

$$
\textbf J^{\left[i\right]}\left(\textbf{X}\right) =
\left.
\frac{\partial \textbf{e}^{\left[i\right]}\left(\textbf{X} \boxplus \Delta \textbf{x}\right)}{\partial \Delta \textbf{x}}
\right|_{\textbf{X}=\textbf{X}^*, \Delta \textbf{x} = 0}
$$

$$
\frac{\partial \textbf{e}^{\left[i\right]}\left(\cdot\right)}{\partial \Delta \textbf{x}} = \textbf n_i^T \cdot \textbf J_{\text{ICP}_\text{pt2pt}}
$$
