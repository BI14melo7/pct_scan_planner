# PCT Planner — Tunable Parameters

All parameters are grouped by where they live and when their effect takes hold.

---

## 1. Tomography parameters

**File:** `tomography/config/scene_clinic.py`

> **Important:** changing any parameter in this section requires re-running tomography to regenerate the pickle file:
> ```bash
> cd tomography/scripts
> python3 run_standalone.py --scene Clinic
> ```

### 1.1 Map geometry

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `map.resolution` | `0.10` m | Grid cell size. Smaller = more detail, slower GPU; larger = coarser, faster. |
| `map.ground_h` | `-13.1` m | Minimum z height to include. Set to just below the lowest floor to clip underground noise. |
| `map.slice_dh` | `0.5` m | Vertical window used to detect floor separation. Increase if floors are being merged; decrease if a floor is being split. |

### 1.2 Agent dimensions

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `trav.kernel_size` | `7` (cells) | Neighbourhood window for traversability analysis — effectively the agent's footprint width. Must be an **odd integer**. |
| `trav.safe_margin` | `0.4` m | Saturated obstacle inflation distance — acts as the agent's **hard collision radius**. Cells in this core receive `cost_barrier`. |
| `trav.inflation` | `0.3` m | Soft cost zone beyond `safe_margin`. The default inflated cost reaches zero about 0.70 m from an obstacle. |
| `trav.interval_min` | `0.50` m | Minimum vertical clearance the agent requires to pass under overhangs. |
| `trav.interval_free` | `0.65` m | Preferred vertical clearance. Cells with headroom between `interval_min` and this value are traversable but carry a cost penalty. |

### 1.3 Climb and step limits

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `trav.slope_max` | `0.40` rad (~23°) | **Maximum climbable slope.** Increase to allow steeper ramps; decrease to keep the robot on flatter ground. `0.52` ≈ 30°, `0.70` ≈ 40°. |
| `trav.step_max` | `0.17` m | **Maximum step height** the agent can cross in a single cell transition. Raise for wheeled robots with large wheels; lower for flat-floor only navigation. |
| `trav.standable_ratio` | `0.20` | Fraction of the kernel window that must be flat (locally horizontal) for a cell to be considered standable. |
| `trav.cost_barrier` | `50.0` | Traversability cost value assigned to impassable cells (acts as infinity for the planner). |

---

## 2. Planner parameters

**File:** `planner/config/param.py`

> These take effect on the **next planning call** — no rebuild or re-tomography needed.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `use_quintic` | `True` | Use quintic (5th-order) spline for trajectory optimisation. Produces smoother paths at the cost of a little more compute. Set `False` for cubic splines if planning speed matters. |
| `max_heading_rate` | `10` | Maximum allowed angular rate of heading change along the trajectory. Lower values force gentler turns. |
| `a_star_cost_threshold` | `20.0` | Hard A* traversability gate. Cells above this inflated/terrain cost are rejected. |
| `a_star_cost_weight` | `0.2` | Weight of the PCT inflated cost in A*'s accumulated edge cost. Increase to prefer more obstacle clearance; decrease to prefer shorter routes. |
| `trajectory_safe_cost` | `15.0` | Cost above which the trajectory smoother applies an obstacle penalty. Lower values keep smoothing farther from obstacles, but values much below 15 can make GPMP overly stiff. |

---

The planner passes these three values from `ConfigPlanner` into the C++
`init_map(...)` call; they are no longer hidden numeric literals in the wrapper.

---

## 4. Quick recipes

| Goal | What to change |
|------|---------------|
| Robot fits through narrower gaps | Decrease `trav.safe_margin` and/or `trav.kernel_size` |
| Allow steeper ramps | Increase `trav.slope_max` (e.g. `0.60` ≈ 34°) |
| Allow higher steps | Increase `trav.step_max` |
| Separate two floors that are merging | Decrease `map.slice_dh` |
| Merge floors that are incorrectly split | Increase `map.slice_dh` |
| More resolution / detail | Decrease `map.resolution` (e.g. `0.05`) — costs GPU memory & time |
| Planner finds paths through tight spaces | Raise `a_star_cost_threshold` |
| Global path stays farther from obstacles | Raise `a_star_cost_weight` and/or lower `trajectory_safe_cost` |
| Smoother trajectory | Keep `use_quintic = True` and lower `max_heading_rate` |
| Faster planning, less smooth path | Set `use_quintic = False` |
