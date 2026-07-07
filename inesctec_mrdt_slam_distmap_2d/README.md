# inesctec_mrdt_slam_distmap_2d

## Aligner

- debug (`bool = false`): enable debug mode (OpenGL-based visualization)
- solver_verbose (`bool = false`):
  enable verbose mode of the solver _(show last iteration stats)_
- solver_damping (`float = 1.f`):
  damping parameter for the iteration algorithm when computing the new state
  _(avoid undetermined optimization problem)_
- solver_max_iterations (`int = 25`):
  maximum number of iterations
- solver_criteria_type (`string = "stepnorm"`):
  type of termination criteria
  (`"simple"|"gradient"|"stepnorm"|""`)
  _(empty to force max iterations)_
- solver_criteria_epsilon (`float = 1e-5f`):
  epsilon threshold for the solver's termination criteria
- solver_robustifier_type (`string = "cauchy"`):
  type of robustifier (`"saturated"|"cauchy"|"clamp"|"huber"|""`)
  _(empty not used)_
- solver_robustifier_num_iterations_coarse (`int = 10`):
  number of iterations for coarse matching
  _(0 to disable coarse + fine; if coarse matching disabled,_
  _fine matching threshold is the one considered during alignment)_
- solver_robustifier_threshold_coarse (`float = 0.025f`):
  robustifier threshold during coarse matching
- solver_robustifier_threshold_fine (`float = 0.0025f`):
  robustifier threshold during fine matching
- failure_ratio_inliers (`float = 0.35f`):
  maximum inliers ratio (#inlier / #moving pts) threshold to trigger the
  failure status of the aligner
  _(< 0 disables the alignment failure check on the inliers ratio value)_
- failure_chi_inliers (`float = -1.f`):
  mean inliers chi error threshold to trigger the failure status of the aligner
  _(< 0 disables the alignment failure check on the chi inliers value)_
- dbg_ogl_interval (`int64_t = 0`):
  sleep interval between consecutive visualizations of the OpenGL-based canvas
  (ms) _(≥ 0, if debug enabled)_
- dbg_ogl_mode (`string = ""`):
  type of debug visualization mode (`"iterationend"|"computeend"`)
- dist_map_res (`float = 0.01f`):
  resolution (m/px)
- dist_map_size (`float = 10.0f`):
  size (m)
  _(square width / 2, ≤ 0 to assume the maximum range in the point cloud)_
- dist_map_max_dist (`float = 0.25f`):
  maximum distance considered to compute / expand the distance map (m)
  _(≤ 0 to compute the whole distance map)_

### Robustifiers

- $\Omega$: information matrix on the error factor
- $\chi$: original chi error, where $\chi = e^T \cdot \Omega \cdot e$
- $k$: robustifier chi threshold
- $\chi_{\text{kernelized}}$: chi error after kernelizing with the robustifier
- _Notes:_
  - considering that $e$ represents a distance error
  - except for huber robustifier, you should consider the robustifier threshold
    as the squared distance from which the moving points are kernelized!
    (thus, not counting for the update step in the solver optimization)
  - in the case of the huber robustifier, the threshold is directly the distance

**`cauchy`**

```math
\chi_{\text{kernelized}} =
\begin{cases}
\chi & , \chi \le k\\
k    & , \chi \gt k
\end{cases}
```

**`clamp`**

```math
\chi_{\text{kernelized}} = k \cdot \log\left(\frac{\chi}{k} + 1\right)
```

**`huber`**

```math
\chi_{\text{kernelized}} =
\begin{cases}
\chi                                            & , \sqrt{\chi} \le k\\
2 \cdot k \cdot \left|\sqrt{\chi} - k\right|    & , \sqrt{\chi} \gt k
\end{cases}
```

_Note:_ the SRRG implementation on Huber kernel uses the residual norm...
(unlike on the other robustifiers with the chi error)

**`saturated`**

```math
\chi_{\text{kernelized}} =
\begin{cases}
\chi & , \chi \le k\\
k    & , \chi \gt k
\end{cases}
```
