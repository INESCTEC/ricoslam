# ICP: Point-to-Point Error Formulation

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
  - $\textbf{z} \in \mathbb{R}^2$ or $\mathbb{R}^3$
  - Fixed point obtained from the environment, e.g., through a range sensor
    (2D or 3D LiDAR)
- Measurement prediction:
  - $\textbf{h}^{\left[i\right]}\left(\textbf{\textbf{X}}\right) =$
    $\textbf{R}\textbf{p}^{\left[i\right]}+\textbf{t} = $
    $\textbf{X}\textbf{p}^{\left[i\right]}=\tilde{\textbf{p}}^{\left[i\right]}$
  - $\textbf{p}^{\left[i\right]}$ is the moving point obtained from the
    environment, e.g., through a range sensor (2D or 3D LiDAR), from the latest
    scan (assuming a known association between the moving and fixed points)
  - $\textbf{h}^{\left[i\right]}\left(\textbf{X}\boxplus\Delta\textbf{x}\right)=$
    - _(left-sided)_ $=\Delta\textbf{X}\cdot\textbf{X}\textbf{p}^{\left[i\right]}=$
      $\Delta\textbf{X} \cdot \tilde{\textbf{p}}^{\left[i\right]} =$
      $\Delta \textbf{R}\tilde{\textbf{p}}^{\left[i\right]} + \Delta \textbf{t}$
    - _(right-sided)_ $= \left[\textbf{R} \Delta \textbf{R} | \textbf{R}\Delta \textbf{t} + \textbf{t} \right]$
      $\cdot \textbf{p}^{\left[i\right]} =$
      $\textbf{R} \Delta \textbf{R} \textbf{p}^{\left[i\right]} +$
      $\textbf{R}\Delta \textbf{t} + \textbf{t}$
- Euclidean parametrization of the measurement: already in the Euclidean space
- Boxminus operator: $\textbf{e}^{\left[i\right]}\left(\textbf{X}\right) =$
  $\textbf{h}^{\left[i\right]}\left(\textbf{\textbf{X}}\right)\boxminus\textbf{z}_i$
  - $\textbf{e}^{\left[i\right]}\left(\textbf{X}\right) =$
    $\textbf{h}^{\left[i\right]}\left(\textbf{\textbf{X}}\right)-\textbf{z}_i$
    (because the measurement is already in an Euclidean parametrization)

## Jacobian

$$
J^{\left[i\right]}\left(\textbf{X}\right) =
\left.
\frac{\partial \textbf{e}^{\left[i\right]}\left(\textbf{X} \boxplus \Delta \textbf{x}\right)}{\partial \Delta \textbf{x}}
\right|_{\textbf{X}=\textbf{X}^*, \Delta \textbf{x} = 0} =
\frac{\partial \textbf{h}^{\left[i\right]}\left(\cdot\right)}{\partial \Delta \textbf{x}}
$$

### Left-sided perturbation update (2D)

$$
\textbf{h}^{\left[i\right]}\left(\textbf{X} \boxplus \Delta \textbf{x}\right) =
\textbf{R}\left(\Delta \theta\right)\tilde{\textbf{p}}^{\left[i\right]} + \Delta \textbf{t}
$$

$$
\frac{\partial \textbf{h}^{\left[i\right]}\left(\cdot\right)}{\partial \Delta \textbf{x}} =
\begin{bmatrix}
\textbf{I}_{2\times2} & -\lfloor \tilde{\textbf{p}}^{\left[i\right]} \rfloor
\end{bmatrix}
$$

### Right-sided perturbation update (2D)

$$
\textbf{h}^{\left[i\right]}\left(\textbf{X} \boxplus \Delta \textbf{x}\right) =
\textbf{R}\left(\textbf{R}\left(\Delta \theta\right) \textbf{p}^{\left[i\right]} +
\Delta \textbf{t}\right) + \textbf{t}
$$

$$
\frac{\partial \textbf{h}^{\left[i\right]}\left(\cdot\right)}{\partial \Delta \textbf{x}} =
R \cdot \left(
\begin{bmatrix}
\textbf{I}_{2\times2} & -\lfloor \textbf{p}^{\left[i\right]} \rfloor
\end{bmatrix}
\right)
$$
