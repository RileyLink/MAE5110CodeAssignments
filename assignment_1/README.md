# Running Assignment 1

Run commands from the repository root after [installing the project dependencies](../README.md#installation).
The main report is [assignment_1.md](assignment_1.md), and the original
[assignment instructions](../assignments/assignment_1.md) are in `assignments/`.

## Files

- [assignment_1.md](assignment_1.md): report.
- [codes/](codes/): simulation and analysis scripts, model, and RK4 integrator.
- [figures/](figures/): figures and saved numerical results.

## Reproduce the report

```console
uv run python assignment_1/codes/sanity_checks.py
uv run python assignment_1/codes/state_space_basins.py
uv run python assignment_1/codes/phase_portrait.py
uv run python assignment_1/codes/compare_attractors.py
uv run python assignment_1/codes/poincare_map.py
uv run python -u assignment_1/codes/parameter_study.py
```

To redraw the parameter-study figures from the saved results:

```console
uv run python assignment_1/codes/parameter_study.py --plot-only
```

The optional slope-sweep visualization is available with
`uv run python assignment_1/codes/gamma_sweep.py`.
All scripts save their results in `assignment_1/figures/`, regardless of the
working directory.
