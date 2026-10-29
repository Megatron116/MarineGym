# Spheroid AUV: parameter reference

Prolate spheroid hull, 1.0 m × 0.5 m, 4 vertical + 4 vectored-horizontal T200 thrusters, no fins.
Uses the standard `UnderwaterVehicle` class.

**Status legend**

| Status | Meaning |
|---|---|
| Analytic | Closed-form formula, exact for the ideal shape |
| Estimate | Engineering estimate (drag coefficients, strip theory) |
| Placeholder | No real source; replace with your own value |
| Pending | Not created yet |

**Where things live**

| What | File |
|---|---|
| Geometry, link masses, inertia, thruster poses | `marinegym/robots/assets/urdf/Spheroid/Spheroid.urdf` |
| Hull mesh | `marinegym/robots/assets/urdf/Spheroid/meshes/spheroid.obj` |
| Added mass, damping, buoyancy, thruster model | `marinegym/robots/assets/usd/Spheroid/Spheroid.yaml` |
| Robot class | `marinegym/robots/drone/Spheroid.py` |
| USD (vehicle, fixed-base marker) | `marinegym/robots/assets/usd/Spheroid/Spheroid.usd`, `.../fixed_usd/Spheroid/Spheroid.usd` (Pending) |

**Conventions**

- URDF / Isaac body frame: +X forward, +Y port (left), +Z up. Origin at the hull's geometric center.
- MarineGym's hydrodynamics flips the signs of Y, Z, roll-y and yaw-z internally (to a NED-style frame);
  diagonal coefficients are unaffected, so enter all of them as positive numbers.
- Water density in the buoyancy code: ρ = 997 kg/m³, with g = 9.8 for buoyancy and 9.81 for gravity.

---

## 1. Geometry and mass

| Parameter | Symbol | Formula | Value | Unit | File / field | Status | Your value |
|---|---|---|---|---|---|---|---|
| Semi-major axis (length/2) | a | — | 0.5 | m | URDF mesh | Chosen | |
| Semi-minor axis (radius) | b | — | 0.25 | m | URDF mesh | Chosen | |
| Aspect ratio | a/b | — | 2.0 | — | — | Chosen | |
| Eccentricity | e | √(1 − (b/a)²) | 0.8660 | — | — | Analytic | |
| Displaced volume | V | 4/3 π a b² | 0.130900 | m³ | YAML `volume` | Analytic | |
| Displaced fluid mass | ρV | 997 · V | 130.507 | kg | — | Analytic | |
| Total mass (neutral) | m | ρV · 9.8 / 9.81 | 130.374 | kg | URDF (sum of links) | Analytic | |
| `base_link` mass | m_base | m − 8 · m_rotor | 130.294 | kg | URDF `base_link` | Analytic | |
| Rotor link mass (each) | m_rotor | — | 0.01 | kg | URDF `rotor_N` | Chosen (near-massless) | |
| CG below CB | z_g | — | 0.02 | m | URDF inertial origin z = −0.02; YAML `coBM` | Chosen | |
| Roll inertia (about CG) | I_xx | 2/5 · m_base · b² | 3.2573 | kg·m² | URDF `ixx` | Analytic | |
| Pitch inertia (about CG) | I_yy | m_base · (a² + b²) / 5 | 8.1434 | kg·m² | URDF `iyy` | Analytic | |
| Yaw inertia (about CG) | I_zz | m_base · (a² + b²) / 5 | 8.1434 | kg·m² | URDF `izz` | Analytic | |
| Products of inertia | I_xy, I_xz, I_yz | symmetric body | 0 | kg·m² | URDF | Analytic | |

Inertias assume a uniform solid spheroid. If the real mass distribution is not uniform
(batteries, ballast), replace them with measured or CAD values.

Rigid-body mass matrix about the hull center (Fossen form, r_g = (0, 0, −z_g) in z-up):
M_RB = [[m·I₃, −m·S(r_g)], [m·S(r_g), I_cg − m·S(r_g)²]].
Only the CG offset creates off-diagonal terms (m·z_g = 2.607 kg·m).

## 2. Added mass (Lamb's coefficients)

| Parameter | Symbol | Formula | Value | Unit | YAML index | Status | Your value |
|---|---|---|---|---|---|---|---|
| Shape integral | α₀ | 2(1−e²)/e³ · (½ ln((1+e)/(1−e)) − e) | 0.3471 | — | — | Analytic | |
| Shape integral | β₀ | 1/e² − (1−e²)/(2e³) · ln((1+e)/(1−e)) | 0.8264 | — | — | Analytic | |
| Axial coefficient | k₁ | α₀ / (2 − α₀) | 0.2100 | — | — | Analytic | |
| Lateral coefficient | k₂ | β₀ / (2 − β₀) | 0.7042 | — | — | Analytic | |
| Rotational coefficient | k′ | e⁴(β₀−α₀) / ((2−e²)(2e² − (2−e²)(β₀−α₀))) | 0.2394 | — | — | Analytic | |
| Surge | m₁₁ (X_u̇) | k₁ ρV | 27.4084 | kg | `added_mass[0]` | Analytic | |
| Sway | m₂₂ (Y_v̇) | k₂ ρV | 91.9044 | kg | `added_mass[1]` | Analytic | |
| Heave | m₃₃ (Z_ẇ) | k₂ ρV | 91.9044 | kg | `added_mass[2]` | Analytic | |
| Roll | m₄₄ (K_ṗ) | body of revolution | 0 | kg·m² | `added_mass[3]` | Analytic | |
| Pitch | m₅₅ (M_q̇) | k′ ρV (a² + b²)/5 | 1.9529 | kg·m² | `added_mass[4]` | Analytic | |
| Yaw | m₆₆ (N_ṙ) | k′ ρV (a² + b²)/5 | 1.9529 | kg·m² | `added_mass[5]` | Analytic | |

These are about the hull center. MarineGym applies hydrodynamic forces at the CG, which is 2 cm lower;
set z_g = 0 if you want the two reference points to coincide exactly (the vehicle then won't self-right).
MarineGym only accepts a diagonal added-mass matrix; for this shape the exact matrix is diagonal anyway.

## 3. Damping

Neither row has an analytic source. Quadratic terms are drag estimates; linear terms are placeholders.

| DOF | Linear (YAML `linear_damping`) | Status | Your value | Quadratic (YAML `quadratic_damping`) | How estimated | Status | Your value |
|---|---|---|---|---|---|---|---|
| Surge (u) | 5.0 N·s/m | Placeholder | | 19.576 N·s²/m² | ½ ρ C_d π b², C_d = 0.2 | Estimate | |
| Sway (v) | 15.0 N·s/m | Placeholder | | 117.456 N·s²/m² | ½ ρ C_d π a b, C_d = 0.6 | Estimate | |
| Heave (w) | 15.0 N·s/m | Placeholder | | 117.456 N·s²/m² | same as sway | Estimate | |
| Roll (p) | 0.5 N·m·s | Placeholder | | 0.5 N·m·s² | guess (smooth hull, skin friction only) | Placeholder | |
| Pitch (q) | 1.0 N·m·s | Placeholder | | 2.4925 N·m·s² | strip theory: ½ ρ C_d · (8/15) b a⁴, C_d = 0.6 | Estimate | |
| Yaw (r) | 1.0 N·m·s | Placeholder | | 2.4925 N·m·s² | same as pitch | Estimate | |

Possible sources: CFD, tow-tank or free-decay tests, or published data for 2:1 prolate spheroids.

## 4. Thrusters

Thrust is applied along each rotor link's local **+X** axis; the joint spins about that same axis.
Positions are relative to the hull center.

| Rotor | Role | x (m) | y (m) | z (m) | Thrust direction (body) | Spin dir. | Your value |
|---|---|---|---|---|---|---|---|
| `rotor_0` | Vertical, fwd-port | 0.40 | 0.15 | 0 | (0, 0, 1) | +1 | |
| `rotor_1` | Vertical, fwd-starboard | 0.40 | −0.15 | 0 | (0, 0, 1) | −1 | |
| `rotor_2` | Vertical, aft-starboard | −0.40 | −0.15 | 0 | (0, 0, 1) | +1 | |
| `rotor_3` | Vertical, aft-port | −0.40 | 0.15 | 0 | (0, 0, 1) | −1 | |
| `rotor_4` | Horizontal, fwd-port | 0.25 | 0.2165 | 0 | (0.707, −0.707, 0) | +1 | |
| `rotor_5` | Horizontal, fwd-starboard | 0.25 | −0.2165 | 0 | (0.707, 0.707, 0) | −1 | |
| `rotor_6` | Horizontal, aft-starboard | −0.25 | −0.2165 | 0 | (0.707, −0.707, 0) | +1 | |
| `rotor_7` | Horizontal, aft-port | −0.25 | 0.2165 | 0 | (0.707, 0.707, 0) | −1 | |

The thruster model ignores spin torque (`moments = ... * 0` in `t200.py`), so spin direction has no effect.

Control authority: the 6 × 8 allocation matrix has rank 6 (all six DOF controllable).
Singular values: 2.000, 1.415, 1.415, 0.800, 0.660, 0.300; the weakest combination is mostly roll.

**T200 model parameters** (YAML `rotor_configuration`, one entry per rotor)

| Field | Value | Meaning | Status | Your value |
|---|---|---|---|---|
| `num_rotors` | 8 | must equal the number of `rotor_*` links | Fixed | |
| `force_constants` | 4.4e-07 | scales the T200 thrust curve; 4.4e-07 = stock (~64 N fwd / ~52 N rev) | Stock T200 | |
| `time_constants` | 0.01 s | rpm response lag | Copied from BlueROV | |
| `max_rotation_velocities` | 3900 rpm | rpm clamp | Stock T200 | |
| `moment_constants` | 1.368e-09 | unused (moments zeroed) | Copied | |
| `directions` | ±1 alternating | spin direction, unused (moments zeroed) | Copied | |

Total authority: about 256 N up or down (4 vertical), about 180 N forward or sideways (4 horizontal at 45°).

## 5. Isaac Sim import and remaining steps

| Setting | Vehicle USD | Fixed-base USD |
|---|---|---|
| Output folder | `assets/usd/Spheroid/` | `assets/usd/Spheroid/fixed_usd/Spheroid/` |
| Fix Base | Off | On |
| Merge Fixed Joints | On | On |
| Self Collision | Off | Off |
| Import Inertia Tensor | On | On |
| Joint Drive Type | None | None |
| Create Instanceable Asset | On | On |
| Create Physics Scene | Off | Off |

- [ ] Import vehicle USD (Fix Base off)
- [ ] Import fixed-base USD (Fix Base on)
- [ ] Check `/Spheroid/base_link` and `rotor_0` … `rotor_7` exist as separate links
- [ ] Replace placeholder damping values
- [ ] Confirm or replace inertias if mass is not uniform
- [ ] Set `drone_model.name: Spheroid` in a task config and run Hover (zero commands should hold roughly still)
