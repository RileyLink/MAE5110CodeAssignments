# Running Assignment 2

Run commands from the repository root. `uv run` installs the Python dependencies
from `pyproject.toml`; Pandoc and XeLaTeX are separate requirements for the PDF.
The main report is [assignment_2.md](assignment_2.md).

## Source files and generated artifacts

Following the assignment instructions, Git contains the Python source, Markdown,
configuration, and a small text results summary. Generated PNG figures, GIF
animations, NPZ data, and the compiled PDF are kept locally and ignored by Git.
The report keeps relative image paths for local preview and PDF compilation;
GitHub will not display those images because they are not stored in the repository.

After a fresh clone, run the generation commands below before previewing the
report or building its PDF. Submit the locally compiled PDF to Canvas. The PDF
builder overwrites `assignment_2/assignment_2.pdf`, so run it only when you intend
to regenerate that document; removing artifacts from Git does not require it.

## Start here

```sh
uv run python assignment_2.py --theta 0 --omega 4
uv run python assignment_2.py --theta -0.2 --omega 1 --no-animation
uv run python assignment_2.py --help
uv run python -m pytest assignment_2/codes/tests/test_assignment_2.py -q
```

The root entry point calls `codes/assignment_2.py`. That module owns simulation,
control, event handling, and animation; `codes/models/inverted_pendulum_walker.py`
owns the physical dynamics, impact reset, and pose drawing. Simulation output is
saved under `figures/` as GIF, JSON summary, and NPZ arrays. `--no-animation`
skips only the GIF. Use `--output /tmp/walker.gif` for a separate experiment.

A stored landing angle always lies in the inclusive interval `[pi/8, pi/7]`.
At capture the swing leg is held clear and hidden in the animation, so the final
single-leg drawing does **not** mean `alpha=0`. Upright standing means `theta=0`.
The JSON status distinguishes confirmed standing, falls, invalid initial poses,
step limits, and timeouts. Increase `--duration` to investigate a timeout; it is
not equivalent to a proven failure to converge.

## Reproduce the report

Each script has one main role. Commands below are listed in dependency order;
the larger reference-grid search can take substantially longer than a trajectory.

| Report content | Command after `uv run python` | Main outputs under `figures/` |
| --- | --- | --- |
| Geometry sketches | `assignment_2/codes/sketch_assignment_2.py` | `sketch_*.png` |
| Analytical RoA | `assignment_2/codes/plot_analytical_capture.py` | `analytical_capture_region.png` |
| Analytical gain checks | `assignment_2/codes/verify_capture_gains.py` | Console inequality margins |
| Numerical RoA and six examples | `assignment_2/codes/validate_roa.py` | RoA/trajectory PNGs, summary JSON, per-state and trajectory CSVs, NPZ |
| Fine state-action map | `assignment_2/codes/build_lookup_policy.py` | `lookup_policy.npz`, summary, policy/map figures |
| Refined reference policies | `assignment_2/codes/search_policy_grid.py` | `reference_*.npz`, grid-search JSON |
| Final 2-node versus 3-node check | `assignment_2/codes/validate_uniform_velocity.py` | Uniform-policy CSV, validation JSON/NPZ |
| Final uniform-grid figure | `assignment_2/codes/plot_uniform_velocity.py` | `lookup_state_action.png` |
| Minimum/maximum footsteps | `assignment_2/codes/final_trajectories.py` | Two trajectory figures and summary |
| Walking animation | `assignment_2.py --theta 0 --omega 4` | `walker.gif`, JSON, and NPZ |
| PDF from current Markdown | `assignment_2/codes/build_pdf.py` | `assignment_2/assignment_2.pdf` |

The final controller uses three uniform velocity nodes and two endpoint landing
angles. `two_interval_policy.py` supplies analytical thresholds and shared
validation helpers; its own `main()` and `build_nonuniform_policy.py` preserve
historical alternatives. `plot_return_map.py` is an optional diagnostic, not a
required report figure. Keeping the analysis modules separate avoids mixing
large numerical sweeps with every animation run.

## Numerical conventions

- Touchdown: `theta = gamma + alpha`; the policy section is instead
  `theta = 0, omega > 0`.
- Torque bounds are inclusive; the two curved RoA boundaries are excluded.
  `CAPTURE_MARGIN=1e-10` rad/s is a numerical boundary buffer.
- Standing requires both state magnitudes below `1e-6` for 0.5 s; this is distinct
  from entering the RoA. The numerical grid uses 15 s; the six examples use 6 s.
- `numerical_roa_trials.csv` records all initial states, predicted membership,
  observed convergence, escape flags, terminal states, onset times, and margins.
  Boolean columns use 0/1. `NaN` onset means no retained settling interval;
  terminal states for escaped trials are their stopped states, not 15 s states.
- `roa_example_trajectories.csv` records time, angle, velocity, and torque for
  each example. Large generated CSV/NPZ data are ignored by Git and reproducible.

The response to the peer review is recorded in [review_notes.md](review_notes.md).
