# SPEEDI

SPEEDI is a smart pathing algorithm designed for FTC. It can follow Bezier splines with an
LQR-based follower (feedforward + optimal state feedback + PID trim), drive straight to points with
physical braking, avoid no-go zones, and it comes with two localizers: GoBilda Pinpoint and Two
Odometry Wheels.

This guide takes you from a robot with **nothing configured** to a robot that is **ready for
autonomous**. Follow the steps in order. Every step tells you the exact OpMode name on the Driver
Station and the exact constant to fill in [`Constants.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Util/utils/Constants.java).

---

## Before you start: the FTC Dashboard

Almost every tuner here uses the **FTC Dashboard**. Connect your laptop to the robot's WiFi and open:

```
http://192.168.43.1:8080/dash
```

On the right side you will see the **Config** panel with all the classes from `Constants.java`.
You can change any value there **in real time** while an OpMode runs — the robot reacts instantly.

> **IMPORTANT:** values changed on the dashboard are **NOT saved**! They reset when you restart the
> app. When you find a value you like, copy it into `Constants.java` in Android Studio and re-deploy.

---

## Step 0 — Hardware configuration

On the Driver Station, create a configuration with these names (or change them in
`Constants.DeviceNames`):

| Hardware | Config name |
|---|---|
| Left front motor | `LFM` |
| Left back motor | `LBM` |
| Right front motor | `RFM` |
| Right back motor | `RBM` |
| GoBilda Pinpoint (I2C) | `odo` |

Then fill in the robot size in `Constants`:

- `robotWidthInCMs` — left-to-right size, in cm
- `robotLengthInCMs` — front-to-back size, in cm

## Step 1 — Motor directions

Run **`MotorConfigTest`**. Each motor spins for 5 seconds, one at a time, and telemetry says which
one *should* be moving forward.

- If a wheel spins **backwards**, flip its `shouldReverse...Motor` flag in
  `Constants.MecanumChassisConstants` and re-deploy.
- Repeat until all four wheels drive the robot forward.

(If you just want to test raw motors without the drivetrain config, there is also **`MotorTest`**,
which uses generic names `M1`–`M4` and the gamepad buttons.)

## Step 2 — Localizer (Pinpoint)

1. Measure with a ruler (in **mm**, from the robot's center of rotation) and fill in
   `Constants.LocalizerConstants` — these follow the official Pinpoint convention:
   - `xPodOffsetInMM` — how far **sideways** the **X pod** (the one that measures forward motion)
     is: **left** of center is positive, right is negative
   - `yPodOffsetInMM` — how far **forward** the **Y pod** (the one that measures strafing) is:
     in **front** of center is positive, behind is negative
   - `typeOfEncoders` — your pod type (4-bar / swingarm / custom)

   (Optional, more precise: run **`Odometry Pod Offsets Tuner`** *after* finishing point 2 below —
   the robot spins in place and calculates both offsets, signs included.)

2. Run **`PinPointLocalizerTest`** and push/drive the robot around:
   - Drive **forward** → `Y` must increase
   - Drive **right** → `X` must increase
   - Turn **counter-clockwise** → heading must increase
   - If a direction is flipped, change `shouldReverseForwardEncoder` / `shouldReverseLateralEncoder`
   - The frequency shown must be between 900 and 2000, otherwise the Pinpoint has a problem

Everything in SPEEDI is in **cm** and **radians**, the robot starts at (0, 0) facing "forward" (+Y).

## Step 3 — Drive model (the heart of the LQR follower)

The LQR follower needs to know how your robot responds to power:
`power = kS·sign(v) + kV·v + kA·a` (kS = friction, kV = speed cost, kA = acceleration cost).

Run **`LQR Drive Model Tuner`** three times, changing `axis` on the dashboard:

| axis | what it measures | copy into `LQRSplineConstants` |
|---|---|---|
| 0 | forward | `forwardKS`, `forwardKV`, `forwardKA` |
| 1 | strafe | `strafeKS`, `strafeKV`, `strafeKA` |
| 2 | rotation (in place, safe anywhere) | `headingKS`, `headingKV`, `headingKA` |

Place the robot in the **middle of a clear lane (~1.5 m each way)**. The runs alternate direction
and are capped at `maxRunDistance`, so it stays around its starting spot — you do **not** need a
long runway. The true max velocity is *predicted* from the model (`(1 - kS) / kV`) instead of being
measured with a full-field sprint, and the follower computes it automatically per path (forward and
strafe limits are different — strafing is slower on a mecanum, and SPEEDI knows that).

Also copy from the same telemetry (from the axis 0 run):

- `maxAcceleration` ≈ 80% of the shown max acceleration
- `maxDeceleration` ≈ 80% of the shown max deceleration

## Step 4 — LQR gains

Run **`LQR Gain Calculator`**. The robot does **not** move — it solves the LQR (Riccati) equation
from the model you just measured and prints the optimal gains. Copy into `LQRSplineConstants`:

`alongKPosition`, `alongKVelocity`, `crossKPosition`, `crossKVelocity`, `headingKPosition`, `headingKVelocity`

Want a different personality? Change the weights on the dashboard and watch the gains update live:

- bigger `qPosition` → hugs the path harder
- bigger `rEffort` → gentler, smoother, less power used for corrections

## Step 5 — Verify the LQR follower

Run **`Test LQR Spline`**. The robot follows a two-curve spline (drawn on the dashboard field view).
Press **A** to run it again, **B** to stop.

Knobs to play with (all live on the dashboard, in `LQRSplineConstants`):

- `profilePowerBudget` — **THE speed knob**. 0.9 means the path is planned with 90% of full power,
  leaving 10% headroom for corrections. Raise towards 1.0 for maximum speed.
- `slowdownDemandThreshold` — the robot keeps full speed until corrections use this fraction of
  their allowed power; only a *significant* error slows it down. Higher = faster but sloppier.
- `xyPIDCoeff_LQR`, `hPIDCoeff_LQR` — small PID trim on top of the LQR. If the robot consistently
  stops 1–2 cm short or holds a small heading error, raise the **I** term slightly. Keep these small:
  the LQR does the heavy lifting.

## Step 6 — Go-to-point (friction + PID)

The point-to-point mode uses friction compensation and a PID, tuned in this order:

1. **`MinimumPowerToOvercomeFrictionDrivetrainTuner`** — finds the minimum power each wheel needs to
   start moving. Copy the four values (WITH voltage correction) into
   `minPowersToOvercomeStaticFriction`.
2. **`Kinetic Kstatic Tuner`** — finds the power that keeps the robot barely moving. Copy the value
   from "LOOK AT ME" into `minPowerToOvercomeKineticFriction`.
3. **`SWITCH_FROM_STATIC_TO_KINETIC_FRICTIONTuner`** — measures how long the robot needs to start
   moving at minimum power. Copy into `SWITCH_FROM_STATIC_TO_KINETIC_FRICTION`.
4. **`ChassisPIDTuner`** — the robot drives to a target and back, repeatedly. Tune
   `tPIDCoeff_GoToPoint` / `hPIDCoeff_GoToPoint` live on the dashboard: raise `p` until it's fast
   without oscillating. `tPIDCoeff_finalAdj` / `hPIDCoeff_finalAdj` are the slow precise gains used
   for the last few cm.

## Step 7 — Physical braking (predictive stopping)

With `shouldUsePhysicalBraking = true`, go-to-point cuts power early and lets the robot glide into
the target, using `glide = v² / (2 · deceleration)`. Getting the deceleration right is what makes
this accurate — and it always needs a final empirical check:

1. Run **`Braking Deceleration Tuner`** with `axis = 0`, copy the average into
   `forwardDeceleration`; then `axis = 1` → `lateralDeceleration`. It measures the glide with the
   same formula the predictor uses, alternating direction so a small field is enough.
2. Run **`Test Predicted Pose`** — drive around, **release the sticks**, and let the robot stop.
   Telemetry shows the error between the predicted and the actual stop point:
   - stopped **before** the prediction → **increase** the deceleration constant (live, on the dashboard)
   - glided **past** the prediction → **decrease** it
   - repeat until the error stays ~0 for forward and sideways releases, then copy the final values
     into `Constants.java`.

## Done — your robot is ready for autonomous

- Follow splines at speed: `drive.setSpline_withLQR(spline)` (see
  [`TestLQRSpline`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/SpeediTuningAndTesting/Testing/ChassisTests/TestLQRSpline.java)
  for the full pattern)
- Precise point-to-point with braking: `drive.setTargetPose(pose, true)`
- Chained targets: `drive.setTargetsList(queue, smoothingDistance)` (**`QueuePoseTest`**)
- No-go zones: `drive.setNoGoZone(topLeft, bottomRight, smoothing)` (**`TestNoGoZone`**)
- Record field positions for your auto by pushing the robot around: **`PositionsCreator`**
  (it prints ready-to-paste `Pose` code lines)

In your auto loop, call `drive.update()` every cycle and check `drive.isDone()`.

---

## Bonus — the vectorial (centripetal) spline follower

An alternative geometric follower (`setSpline_withInstantHeadingChange`,
`setSpline_withSlowerHeadingChange`, `setSpline_withTangentialHeadingChange`). Tune it only if you
want to use it specifically:

1. **`SplinePIDTuner`** — tunes `tPIDCoeff_SplineFollower` (in `FollowerConstants`) the same way as
   the chassis PID.
2. Set `TotalMassOfRobot` (kg) in `FollowerConstants`, then run **`CentripetalForceTuner`** — the
   robot drives a circle; raise `CentripetalScalingFactor` on the dashboard until it stops drifting
   outward in curves.
3. Verify with **`TestSpline`** / **`TestBezier`**.

## Troubleshooting

- **Robot drives away/spins on a spline** → re-check Step 1 (motor directions) and Step 2 (localizer
  directions). Every following problem starts with a bad localizer.
- **LQR robot lags behind then catches up** → that's the profile pausing for a large along-track
  error; raise `maxAcceleration` realism (Step 3) or lower `profilePowerBudget`.
- **Wobbling on straight lines** → gains too hot: raise `rEffort` in the gain calculator and re-copy
  the K values, or reduce the PID trim.
- **Stops short of the final point** → raise the I term of `xyPIDCoeff_LQR` slightly.
- **Go-to-point overshoots** → re-run Step 7; also check battery voltage (tune with a charged one).
- **Nothing appears on the dashboard** → the OpMode must be INIT-ed or running, and
  `useDashboard = true` in `Constants`.
