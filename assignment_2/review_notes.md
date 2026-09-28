# Assignment 2 peer-review follow-up

This records the changes and technical explanations for Jonathan Distler's
[review](https://github.com/xuyilian/MAE5110CodeAssignments/pull/2#pullrequestreview-5249798219).
It is a local response record; it does not mark GitHub conversations resolved.

## Model and guards

- `dynamics` now names the gravitational and torque accelerations separately.
  `params.get("ankle_torque", 0.0)` returns the actual stored value, including a
  nonzero torque; 0.0 is only the fallback for a missing key. Scalar and batched
  dynamics use the same equation.
- `event_guard` is the touchdown guard at `theta=gamma+alpha`. The policy section
  is `theta=0, omega>0`, located separately in the simulation. The strict angle
  increase is retained: `previous_theta <= target <= next_theta` alone also
  accepts `previous_theta == next_theta == target`. A regression assertion now
  covers that case.
- Energy variables use `mass` and `gravity`. The impact test predicts dissipated
  energy as `0.5*mass*length**2*omega_before**2*sin(2*alpha)**2`, with both energies
  expressed in the same world-height reference.

## Standing angle and physical bounds

The main visual concern is a distinction between `theta` and `alpha`. Upright
standing is `theta=0`, not `alpha=0`. Landing angles remain within the inclusive
interval `[pi/8, pi/7]` in every stored sample, including after capture. Ankle
torque also obeys inclusive limits `[-0.1*m*g*l, 0.05*m*g*l]`.

Once capture occurs, the swing leg is held clear and omitted from the drawing;
its angle is not reset to zero. The last permitted landing angle remains in the
trajectory data. This is the report's ideal single-support stance assumption,
not a model of a second foot placed next to the stance foot. The animation now
labels the hidden swing leg explicitly, and the tests check all recorded alpha
values. Existing Section 1 sketches provide static poses in addition to the GIF.

The curved RoA boundaries are excluded. `CAPTURE_MARGIN=1e-10` rad/s removes a
small numerical ambiguity region around them; it does not certify robustness to
model error. The distinct `STANDING_TOLERANCE=1e-6` applies to both state
components and must hold for 0.5 s. The analytical boundary function is now
documented with its constant-torque energy derivation and report reference.

## Simulation and policy explanations

- Feasible initial actions satisfy forward-touchdown geometry and passive energy
  conditions. This differs from the Froude-based section speed design range.
- Event bisection uses a named `EVENT_BISECTIONS=35`: a default 0.001 s bracket
  shrinks to roughly 2.9e-14 s. This is event localization within the numerical
  trajectory, not a claim of that accuracy for the integrated physical solution.
- The simulation documents capture latching, earliest-event selection, and the
  uninterrupted standing hold. A finite timeout is retained because arbitrary
  initial conditions are not all recoverable; it is explicitly distinguished
  from a fall. An edge case now recognizes standing when the 0.5 s confirmation
  finishes exactly at the requested simulation horizon.
- The uniform policy stores real landing angles, not integer actions. Linear
  interpolation would change the policy that was validated, so nearest-node
  selection and the higher-node tie rule are retained.
- `transition` starts on the forward upright section and follows passive stance,
  one impact, and either capture or a section return. `rollout` executes the
  selected landing angles at the actual successor velocities until capture;
  only the action lookup uses a grid index. These functions now explain their
  inputs, outputs, numerical capture buffer, and failure result.
- Velocity/action node counts, return velocities, analytical interval bounds,
  and stopping-count thresholds have more descriptive names. In the historical
  interval script, intervals refer to section velocities, not angles or time.

## Tests, CSV data, and reproduction

The continuous energy/power test uses a small time-like perturbation along the
stance vector field to estimate the energy derivative. Its state is away from
touchdown, so it does not straddle an impact discontinuity. `pytest.approx`
allows floating-point tolerance. The stopping cases cover already-captured,
reverse-motion, off-section, and one-/two-/three-impact initial states; the
section cases correspond to the analytically derived stopping intervals.

RoA validation runs the ankle controller from both interior and exterior grid
states and near-boundary probes, compares sustained convergence with analytical
membership, then repeats at half the timestep. It now additionally exports
`numerical_roa_trials.csv` and `roa_example_trajectories.csv`, so the per-state
results and example time histories can be inspected without opening NPZ files.

The root entry point now includes run examples and points to
[README.md](README.md), which lists the execution order, outputs, and script
roles. Separate modules are retained to keep the physical model, simulator,
expensive analysis, and document generation focused. Historical alternatives
are labeled explicitly; they are not required to run the main animation.

The report's substantive analysis and six-second example duration are retained.
The numerical RoA sweep, lookup validation, trajectory comparisons, tests, and
PDF are regenerated or checked after these changes. Positive review comments
require no code changes; optional interpolation, removing timeouts, and merging
all scripts into two files are addressed above by explaining the retained design.

## Verification after the revision

- 16 model/controller tests pass; lint checks pass for the revised Python files.
- The 2,401-node fine policy has zero rollout failures and step-count mismatches;
  its 2,400 midpoint checks also have zero failures.
- The three-node policy has zero failures and step-count disagreements on 100,087
  test velocities; the two-node counterexample remains reproducible.
- All 3,340 interior RoA samples converge at both timesteps, with no classification
  changes; CSV exports contain 22,869 trials and 36,006 example time samples.
- The minimum/maximum example still uses 3/5 footstrikes. The updated GIF retains
  the previous `(-0.2, 10)` initial state and reaches standing after 5 footstrikes.
- GitHub Markdown preprocessing preserves all 280 formulas, which also parse in
  MathJax; no prohibited `operatorname` macro is used.
- The 22-page PDF was rebuilt and visually checked. Figure placement now prevents
  tall plots and following text from overflowing pages in the local TeX setup.
