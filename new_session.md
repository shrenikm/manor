Hi! We've been working on this project together for a long time. To get up to
speed, please read the README under `src/manor/common/aegis/README.md` for the
aegis stack, and `src/manor/manipulators/lite6/README.md` for the empirical
notes on the Ufactory Lite6 / xarm-python-sdk hardware bring-up.

Where we are right now (as of 2026-05-01):

- We've been doing a deep refactor of the project. aegis (the new stack) is in
  good shape; the old code coexists with it in the repo and will get nuked once
  the new path is fully working end-to-end.
- The active branch is `aegis`.


Quick orientation by directory:

- `src/manor/common/aegis/` — the new runtime (Metis / Kyber / Talos /
  Helios / Gaia / GaiaAdvancer + LCM adapters + CLI + per-block runners).
- `src/manor/common/aegis/metis/policies/` — policy implementations + the
  `MetisPolicyManager` factory.
- `src/manor/common/aegis/kyber/controllers/` — controller implementations
  + the `KyberControllerManager` factory.
- `src/manor/manipulators/lite6/` — Lite6 model, driver, hardware-bringup
  CLI (`lite6_cli.py`) used to characterise the xarm SDK.
- `configs/aegis/` — the new split config layout (base + policies/ +
  controllers/).
- `models/` — URDFs / SDFs for environment objects (tables, etc).
- `robot_models/` — git submodule with robot URDFs (Lite6 etc).

Feel free to read whatever else you need to get a feel for the project.
