# Distance Map 2D: Point-to-Plane Error Formulation

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
  - $\textbf n^{\left[i\right]} \in \mathbb{R}^2$
    (normal vector, where $\lVert\textbf n^{\left[i\right]}\rVert=1$, computed
    or perceived at the **moving point** $\textbf p^{\left[i\right]}$ obtained
    from the environment)
- Error formulation:
  - $\textbf e^{\left[i\right]}\left(\textbf X\right) =$
    $\left(\nabla dmap\left( \tilde{\textbf p}^{\left[i\right]} \right) dmap\left( \tilde{\textbf p}^{\left[i\right]} \right)\right) \bullet \tilde{\textbf n}^{\left[i\right]} =$
    ${\tilde{\textbf n}^{\left[i\right]}}^T \cdot \nabla dmap\left( \tilde{\textbf p}^{\left[i\right]} \right)dmap\left( \tilde{\textbf p}^{\left[i\right]} \right) $
  - $\tilde{\textbf n}^{\left[i\right]}$ is the normal vector of the moving
    point w.r.t. the fixed coordinate frame (same as the distance map)
    - _Note:_ normal vector is in fact a direction. Consequently, only the
      rotation part of homogeneous transformations affect the transformation of
      the normal vector
  - $\textbf{p}^{\left[i\right]}$ is the moving point obtained from the
    environment, e.g., through a range sensor (2D or 3D LiDAR), from the latest
    scan (assuming a known association between the moving and fixed points)
  - $dmap\left(\cdot\right)$ provides the association to the nearest neighbor
  - $\nabla dmap\left(\cdot\right)$ represents the direction vector of the
    1st-derivative on the distance map reference frame
    (where $\lVert\nabla dmap\left(\cdot\right)\rVert=1$)

## Jacobian

$$
\textbf J^{\left[i\right]}\left(\textbf{X}\right) =
\left.
\frac{\partial \textbf{e}^{\left[i\right]}\left(\textbf{X} \boxplus \Delta \textbf{x}\right)}{\partial \Delta \textbf{x}}
\right|_{\textbf{X}=\textbf{X}^*, \Delta \textbf{x} = 0}
$$

_Note:_ both 1st and 2nd-derivatives of the distance map can be precomputed as
lookup tables (as the distance map itself).

### Left-sided perturbation update (2D)

$$
\frac{\partial \textbf e^{\lbrack i\rbrack}\left(\cdot\right)}{\partial \Delta \textbf x} =
{\tilde{\textbf n}^{\lbrack i\rbrack}}^T \cdot
\lbrack
  \left(
    \begin{pmatrix}
    0_{2\times 2} & \lfloor \nabla dmap\left(\cdot\right) \rfloor
    \end{pmatrix} +
    \frac{\partial \nabla dmap\left(\cdot\right)}{\partial \textbf y} \cdot \textbf J_{\text{ICP pt2pt}}
  \right) \cdot dmap\left(\cdot\right) +
  \nabla dmap\left(\cdot\right) \cdot \frac{\partial dmap\left(\cdot\right)}{\partial \textbf y} \cdot \textbf J_{\text{ICP pt2pt}}
\rbrack
$$

### Right-sided perturbation update (2D)

$$
\frac{\partial \textbf e^{\lbrack i\rbrack}\left(\cdot\right)}{\partial \Delta \textbf x} =
{\textbf n^{\lbrack i\rbrack}}^T \cdot
\begin{pmatrix}
0_{2\times 2} & \lfloor \textbf R^T \nabla dmap\left(\cdot\right) dmap\left(\cdot\right) \rfloor
\end{pmatrix} +
{\tilde{\textbf n}^{\lbrack i\rbrack}}^T \cdot
\left(
  \frac{\partial \nabla dmap\left(\cdot\right)}{\partial \textbf y} \cdot dmap\left(\cdot\right) +
  \nabla dmap\left(\cdot\right) \cdot \frac{\partial dmap\left(\cdot\right)}{\partial \textbf y}
\right) \cdot \textbf J_{\text{ICP pt2pt}}
$$
