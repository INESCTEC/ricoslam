# Distance Map 2D: Point-to-Point Error Formulation

## State Space

- State domain:
  - $\textbf{X} \in SE\left(2\right), \left[\textbf{R}|\textbf{t}\right]$
  - Homogeneous transformation of the moving w.r.t. the fixed point cloud
- Euclidean parametrization of the state perturbation:
  - $\Delta \textbf{x} = \left[\Delta x,\Delta y,\Delta \theta\right]^T \in $
    $\mathbb{R}^3$
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
  - $dmap\left(\cdot\right)$ (2D distance map computed based on the fixed point
    cloud perceived through a 2D laser scanner)
- Error formulation:
  - $\textbf e^{\left[i\right]}\left(\textbf X\right) =$
    $dmap\left( \textbf X \textbf p^{\left[i\right]} \right) =$
    $dmap\left( \tilde{\textbf p}^{\left[i\right]} \right)$
    $\in \mathbb{R}^1$
  - $\textbf{p}^{\left[i\right]}$ is the moving point obtained from the
    environment, e.g., through a range sensor (2D or 3D LiDAR), from the latest
    scan (assuming a known association between the moving and fixed points)
  - $\textbf e^{\left[i\right]}\left(\cdot\right)$ is the distance of the
    moving point projected onto the fixed coordinate frame
  - $dmap\left(\cdot\right)$ provides the association to the nearest neighbor

## Jacobian

$$
\textbf J^{\left[i\right]}\left(\textbf{X}\right) =
\left.
\frac{\partial \textbf{e}^{\left[i\right]}\left(\textbf{X} \boxplus \Delta \textbf{x}\right)}{\partial \Delta \textbf{x}}
\right|_{\textbf{X}=\textbf{X}^*, \Delta \textbf{x} = 0}
$$

$$
\frac{\partial \textbf e^{\left[i\right]}\left(\cdot\right)}{\partial \Delta \textbf x} =
\left(\frac{\partial dmap\left(\textbf y\right)}{\partial \textbf y}
\vert_{\textbf y = \tilde{\textbf p}^{\left[i\right]}}\right)
\cdot \textbf J_{\text{ICP}_\text{pt2pt}}
$$

_Note:_ 1st-derivative of the distance map can be precomputed as a lookup table
(as the distance map itself).
