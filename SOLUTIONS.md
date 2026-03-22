# Solutions Guide — ACCN Umoja Robotics 7712 · `umoja_2026_rebuilt`

**Companion to:** `BUG_REPORT.md`  
**Prepared by:** GitHub Copilot (Professional Software Engineering Review)  
**Original date:** 2026-03-21  
**Revised:** 2026-03-22 — updated after team feedback that some original suggestions didn't
apply and some measurements are intentional. Eight bugs in the SOTM branch are now
**fixed directly in code** (see the checklist at the bottom). New bugs found during
SOTM branch line-by-line review are listed at the end of this file (NEW-01 … NEW-08).

---

## How to use this document

Each section corresponds to one bug, and provides:
1. **What to change** — the exact file and line(s).
2. **What the current code looks like** (so you can find it easily).
3. **What to replace it with** — the corrected code snippet.
4. **Why the fix works** — a plain-language explanation.
5. **Testing guidance** — how to verify the fix on the robot or in simulation.
6. **Status** — ✅ Already fixed in code / ⏳ Still needs fixing / 🚫 Not applicable.

Work through in priority order: 🔴 Critical first, then 🟠 High, then 🟡 Medium, then 🔵 Low.

---

## Priority Order Cheat-Sheet (original 14 + 8 new)

| Priority | Bug ID | One-line title | Status |
|----------|--------|----------------|--------|
| 🔴 1 | BUG-01 | Kicker velocity clamp never fires | 🚫 Intentional — flywheel runs reversed; clamp works as designed |
| 🔴 2 | BUG-02 | ZeroHoodCommand missing subsystem requirement | ✅ Fixed in code |
| 🔴 3 | BUG-03 | Blue Left Trench auto uses Red-side poses | ⏳ Autos.java still needs fixing |
| 🔴 4 | BUG-04 | getPathToPose() returns wrong Command object | ⏳ Autos.java still needs fixing |
| 🟠 5 | BUG-05 | AlignWithTrench mutates target angle every loop | ✅ Fixed in code |
| 🟠 6 | BUG-06 | ShooterFlywheelVelocityCommand.end() doesn't stop motor | ✅ Fixed in code |
| 🟠 7 | BUG-07 | Driver X button aims the wrong direction | ⏳ RobotContainer.java — confirm intent with drive team |
| 🟠 8 | BUG-08 | Simple auto only drives — never shoots | ⏳ Autos.java still needs fixing |
| 🟡 9 | BUG-09 | Flywheel state machine feeds mode codes as RPM | ⏳ ShooterFlywheelSubsystem still needs fixing |
| 🟡 10 | BUG-10 | Arm PID/FF gains are all zero | 🚫 Team confirmed manual arm control; PID path unused |
| 🟡 11 | BUG-11 | Dead auto methods can never be selected | ⏳ Autos.java still needs fixing |
| 🟡 12 | BUG-12 | Blue/Red Full-1 autos end race group at wrong pose | ⏳ Autos.java still needs fixing |
| 🔵 13 | BUG-13 | Unused import in ZeroHoodCommand | ✅ Fixed in code |
| 🔵 14 | BUG-14 | Duplicate NetworkTable topic names | ✅ Fixed in code |
| 🔴 NEW-01 | NEW-01 | AlginRobotBackWithHubCameraTxCommand never drives when target visible | ✅ Fixed in code |
| 🟠 NEW-02 | NEW-02 | IntakeWhileMoving uses degreesToRadians on radian value | ✅ Fixed in code |
| 🟠 NEW-03 | NEW-03 | ShootCommand.end() never stops flywheel | ✅ Fixed in code |
| 🟠 NEW-04 | NEW-04 | Turret upper-spring resistance never fires (direction > 1 typo) | ✅ Fixed in code |
| 🟡 NEW-05 | NEW-05 | PickUpFuelCommand.end() never stops swerve modules | ✅ Fixed in code |
| 🔵 NEW-06 | NEW-06 | PickUpFuelCommand NT topics also duplicate ("x speed"/"y speed") | ✅ Fixed in code |
| 🔵 NEW-07 | NEW-07 | AlginRobotBackWithHubCameraTxCommand NT topics also duplicate | ✅ Fixed in code |
| 💡 NEW-08 | NEW-08 | Mechanical suggestion: characterise kProjectileSpeedMPS | ⏳ Needs field measurement |

---

---

## 🔴 BUG-01 · Fix: Kicker velocity clamp uses wrong Math function

### File
`src/main/java/frc/robot/subsystems/ShooterFlywheelSubsystem.java`

### Current code (line 77–78)
```java
double wantedKickerVelocity = wantedVelocity * -1.5;
wantedKickerVelocity = Math.min(wantedKickerVelocity, 5000); // limit to 5000 RPM
```

### What is wrong
`wantedVelocity` is the flywheel target RPM, which is **negative** in normal operation (e.g., `-3800` RPM because the motor runs in reverse).

Multiplying a negative number by `-1.5` gives a **positive** kicker velocity (e.g., `+5700`).

`Math.min(+5700, 5000)` → `5000` ✅ — this does cap correctly.

However, if the flywheel is ever commanded with a positive RPM value, `wantedKickerVelocity` becomes a large **negative** number (e.g., `-5700`).

`Math.min(-5700, 5000)` → `-5700` ❌ — the clamp does nothing. The motor is free to run at unlimited speed in the negative direction, risking overheating or damage.

Additionally, `runShooter(double speed)` (line 68) has the same pattern:
```java
kickerMotor.set(Math.min(speed * 1.5, 0.75));
```
For negative `speed` (e.g., `-0.4`), `speed * 1.5 = -0.6` → `Math.min(-0.6, 0.75)` = `-0.6`. No cap. This is a second instance of the same category of bug.

### How to fix

**In `setShooterVelocity()` (line 78):** Replace `Math.min` with a magnitude clamp that respects sign:

```java
// Replace this:
wantedKickerVelocity = Math.min(wantedKickerVelocity, 5000);

// With this — clamps the magnitude to 5000 RPM regardless of sign:
if (Math.abs(wantedKickerVelocity) > 5000) {
    wantedKickerVelocity = Math.copySign(5000, wantedKickerVelocity);
}
```

**In `runShooter()` (line 68):** Apply the same pattern:

```java
// Replace this:
kickerMotor.set(Math.min(speed * 1.5, 0.75));

// With this:
double kickerSpeed = speed * 1.5;
if (Math.abs(kickerSpeed) > 0.75) {
    kickerSpeed = Math.copySign(0.75, kickerSpeed);
}
kickerMotor.set(kickerSpeed);
```

### Why this works
`Math.copySign(magnitude, referenceValue)` returns `magnitude` with the same sign as `referenceValue`. This correctly clamps both positive and negative values to the safe range `[-5000, +5000]` RPM.

### Testing guidance
1. In simulation or on the bench (note down safe with no ball): command a flywheel velocity of `+3500` RPM.
2. Read `SmartDashboard → WANTED KICKER VELOCITY`. It should be ≤ −5000 (since kicker is `wantedVelocity * -1.5`).
3. Command `−3500` RPM. Kicker should be clamped to `+5000`.
4. Verify kicker motor current stays within spec during spin-up.

---

## 🔴 BUG-02 · Fix: ZeroHoodCommand missing subsystem requirement

### File
`src/main/java/frc/robot/commands/ZeroCommands/ZeroHoodCommand.java`

### Current code (constructor, lines 13–15)
```java
public ZeroHoodCommand(ShooterHoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    // ← addRequirements() call is missing
}
```

### What is wrong
WPILib's Command Scheduler uses `addRequirements()` to enforce mutual exclusion: when a command requires a subsystem, any other command that also requires that subsystem is automatically cancelled before the new one starts. Without the call, the Scheduler does not know this command "owns" the hood.

If any other command that *does* declare the hood as a requirement (e.g., `ShooterHoodValueCommand` or `ManualShooterHoodCommand`) is triggered while zeroing is running, **both commands will drive the hood motor simultaneously** — one running it slowly backward to find the limit (zeroing), the other driving it at an angle set-point. This can strip the hood mechanism, leave it in an unknown position, or burn out the motor.

Additionally, the existing `getInterruptionBehavior()` returning `kCancelIncoming` is good intent but is entirely wasted without the requirement being declared (there is nothing to cancel if the scheduler does not track this subsystem as owned).

### How to fix

Add a single line inside the constructor:

```java
public ZeroHoodCommand(ShooterHoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(hoodSubsystem);   // ← ADD THIS LINE
}
```

No other changes are needed. The `kCancelIncoming` interruption behavior will then work as designed: while zeroing is running, any other command that tries to use the hood will be rejected.

### Why this works
`addRequirements(hoodSubsystem)` registers the subsystem with the scheduler. The scheduler will then:
- Cancel any currently-running command that uses the hood before starting `ZeroHoodCommand`.
- Refuse any new command that uses the hood while `ZeroHoodCommand` is active (because of `kCancelIncoming`).

### Testing guidance
1. Trigger `ZeroHoodCommand` (operator MENU button per current bindings).
2. While it is running, press operator Right Bumper (which schedules `ManualShooterHoodCommand`). The manual command should be silently rejected.
3. Confirm the hood successfully homes to the limit switch without oscillation.
4. After zeroing finishes, confirm the Right Bumper resumes normal manual control.

---

## 🔴 BUG-03 · Fix: getBlueTrenchLeftNeutral() navigates to Red-side poses

### File
`src/main/java/frc/robot/commands/Autos.java`

### Current code (lines 217–220)
```java
public Command getBlueTrenchLeftNeutral() {
    Pose2d endPose2d = SHOOTING_POSES.RED_NEUTRAL_LEFT;   // ← BUG
    Pose2d pickUpPose = SHOOTING_POSES.RED_TRENCH_LEFT;   // ← BUG
```

### What is wrong
This method is the implementation for `AUTO.BLUE_TRENCH_LEFT_NEUTRAL` (shown in the chooser as *"Blue Left Trench to Neutral"*). It was copied from `getRedTrenchLeftNeutral()` but the two pose constants were never updated to their Blue-alliance equivalents.

When this auto is selected on **Blue alliance**, the robot will attempt to drive to:
- `RED_NEUTRAL_LEFT` = field position on the Red side (approximately x=7.8, y=1.1 m on a standard WPILib field)
- `RED_TRENCH_LEFT` = Red trench ball pickup position

This causes the robot to cross into the opponent's territory, risking a red card penalty and guaranteeing the balls are not collected from the correct location.

### How to fix

Replace both constants with their Blue-alliance equivalents, which already exist in `Constants.java`:
- `BLUE_NEUTRAL_LEFT` = `new Pose2d(7.775, 6.902, ...)` (line 448 of Constants.java)
- `BLUE_TRENCH_LEFT` = `new Pose2d(3.504, 7.559, ...)` (line 450 of Constants.java)

```java
public Command getBlueTrenchLeftNeutral() {
    Pose2d endPose2d = SHOOTING_POSES.BLUE_NEUTRAL_LEFT;    // ← CHANGE
    Pose2d pickUpPose = SHOOTING_POSES.BLUE_TRENCH_LEFT;    // ← CHANGE
```

The rest of the method body (path commands and parallel groups) can remain unchanged.

### Why this works
`BLUE_NEUTRAL_LEFT` and `BLUE_TRENCH_LEFT` are the mirrored Blue-alliance equivalents of the Red poses. PathPlanner's `pathfindToPose()` will now navigate to the correct field positions.

### Testing guidance
1. Place the robot on the Blue alliance side. Select *"Blue Left Trench to Neutral"* from the dashboard chooser.
2. Enable in Autonomous. Verify the robot drives toward the **Blue** trench (left side from the Blue driver station) and then to the Blue neutral zone.
3. Cross-check the published auto end pose on NetworkTables (`Auto end pose` topic) — it should read `~(7.8, 6.9)` not `~(7.8, 1.1)`.

---

## 🔴 BUG-04 · Fix: getPathToPose() returns the wrong Command object

### File
`src/main/java/frc/robot/commands/Autos.java`

### Current code (lines 635–646)
```java
public Command getPathToPose(Pose2d endPose) {
    posePublisher.set(endPose);
    Trajectory traj = TrajectoryGenerator.generateTrajectory(
        swerveSubsystem.getPose(),
        List.of(),
        endPose,
        trajectoryConfig);

    SwerveControllerCommand trajectoryPath = new SwerveControllerCommand(
        traj,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController, yController, thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    Command pathPlannerPath = AutoBuilder.pathfindToPose(endPose, Constants.pathConstraints);
    return trajectoryPath;  // ← BUG: pathPlannerPath is computed but immediately discarded
}
```

### What is wrong
`pathPlannerPath` (the correct holonomic path via PathPlanner) is computed but the function returns `trajectoryPath` (the old WPILib `SwerveControllerCommand`) instead.

`SwerveControllerCommand` generates a **non-holonomic differential trajectory** — it cannot strafe, only arc like a tank drive. For a swerve robot this means:
- The robot arcs in a curved path instead of driving a direct straight line.
- The trajectory start-point is the robot's position **at command construction time** (during `Autos` constructor), not at runtime — if the robot moves before autonomous starts, the trajectory will be wrong.
- The `ProfiledPIDController` for heading expects its input in degrees (0–360), but the `SwerveControllerCommand` internally uses `Rotation2d` in radians — the `thetaController.enableContinuousInput(0, 360)` call in `Autos` will feed it wrong-scale values.

`pathPlannerPath` (via `AutoBuilder.pathfindToPose`) correctly handles holonomic motion, live robot pose lookup, and field obstacle avoidance.

### How to fix

Change the return statement to return `pathPlannerPath`:

```java
    Command pathPlannerPath = AutoBuilder.pathfindToPose(endPose, Constants.pathConstraints);
    return pathPlannerPath;   // ← CHANGE: was "return trajectoryPath;"
```

You can also remove the now-unused `TrajectoryGenerator` and `SwerveControllerCommand` code from this method to keep it clean (but this is optional and does not affect correctness):

```java
public Command getPathToPose(Pose2d endPose) {
    posePublisher.set(endPose);
    return AutoBuilder.pathfindToPose(endPose, Constants.pathConstraints);
}
```

### Why this works
`AutoBuilder.pathfindToPose()` uses PathPlanner's on-the-fly pathfinding (LocalADStar, already initialised in `Robot.robotInit()`). It computes the path at runtime from the robot's current live pose, handles full holonomic motion (strafing + rotation simultaneously), and respects the `pathConstraints` limits already defined in Constants.

### Testing guidance
1. Enable in autonomous with *"tune"* auto selected (which calls `getPathToPose` internally).
2. Watch the robot's actual path on the field (or in Glass/Elastic). It should drive in a straight line to the target pose, not an arc.
3. Check that the path updates correctly if the robot starts from a slightly different position than expected.

---

## 🟠 BUG-05 · Fix: AlignWithTrench mutates instance field every loop tick on Red alliance

### File
`src/main/java/frc/robot/commands/AlignWithTrench.java`

### Current code (execute() method, approximately line 83–90)
```java
boolean isBlue = !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
int flipAlliance = isBlue ? 1 : -1;
int flipDirection = wantedAngle == 0 ? 1 : -1;
if (!isBlue) {
    // flip wantedAngle for redside
    wantedAngle = (wantedAngle + 180) % 360;   // ← BUG: mutates the instance field
}
// ...
chassisSpeeds = new ChassisSpeeds(
    flipAlliance * flipDirection * xSpeed,
    flipAlliance * flipDirection * ySpeed,
    turnController.calculate(RobotContainer.diffFromWantedAngle(wantedAngle), 0)
);
```

### What is wrong
`wantedAngle` is a **field variable** assigned once in the constructor. The `execute()` method is called by the Command Scheduler **every 20 milliseconds**. On Red alliance:

| Call # | `wantedAngle` (constructor value 0) | Resulting heading target |
|--------|--------------------------------------|--------------------------|
| 1 | `(0 + 180) % 360 = 180` | 180° |
| 2 | `(180 + 180) % 360 = 0` | 0° |
| 3 | `(0 + 180) % 360 = 180` | 180° |
| … | … | oscillates |

The robot's heading target flips between 0° and 180° at 50 Hz, causing the turn PID controller to continuously reverse direction. The chassis will shake violently and fail to align.

Note: `flipDirection` on line 82 is also computed from the original `wantedAngle` **before** the mutation, but used **after** it. After the first loop tick, `wantedAngle` is no longer 0, so `flipDirection` becomes `−1` (driving in the reverse direction). This creates a cascading second bug from the same root cause.

### How to fix

Introduce a **local variable** to hold the flipped angle, leaving the instance field unchanged:

```java
boolean isBlue = !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
int flipAlliance = isBlue ? 1 : -1;

// ↓ Use a local variable — do NOT write back to wantedAngle (the field)
double targetAngle = wantedAngle;
if (!isBlue) {
    targetAngle = (wantedAngle + 180) % 360;
}

int flipDirection = wantedAngle == 0 ? 1 : -1;   // still uses the original field value

chassisSpeeds = new ChassisSpeeds(
    flipAlliance * flipDirection * xSpeed,
    flipAlliance * flipDirection * ySpeed,
    turnController.calculate(RobotContainer.diffFromWantedAngle(targetAngle), 0)  // ← use local
);
```

### Why this works
`targetAngle` is created fresh every 20 ms from the stable `wantedAngle` field. The field is set once in the constructor and never changed again. On Red alliance, the 180° flip is applied to `targetAngle` only for this one loop iteration, producing a consistent heading target every cycle.

### Testing guidance
1. Connect to Red alliance in Driver Station.
2. Press the driver controller Y button (bound to `AlignWithTrench(0)`).
3. Observe the robot's heading in SmartDashboard / Elastic. It should stabilise at **180°** (the Red-flipped version of 0°) without oscillation.
4. Repeat with Blue alliance — the heading should stabilise at **0°**.

---

## 🟠 BUG-06 · Fix: ShooterFlywheelVelocityCommand.end() does not stop the motor

### File
`src/main/java/frc/robot/commands/ShooterFlywheelVelocityCommand.java`

### Current code (lines 28–31)
```java
@Override
public void end(boolean isInterrupted) {
    System.out.println("SHOOTER END INTERRUPTED:" + isInterrupted);
    // ← no motor stop
}
```

### What is wrong
When this command ends for any reason (button release, timeout, interruption by another command), the flywheel motor is left at whatever speed it was last commanded. This means:
- During autonomous, after a timed shooting phase the flywheel continues drawing current while the robot drives to pick up balls.
- If the command is interrupted by the scheduler, there is a window — between this command ending and the next command (`ManualShooterFlywheelCommand(() -> 0.0)`) starting — where the flywheel spins uncontrolled.
- If the subsystem's `.whileFalse()` handler is somehow skipped or delayed, the flywheel never stops.

`ShooterFlywheelSubsystem` already has a `stopFlywheel()` method (line 120) that calls `flywheelMotorLeader.stopMotor()`. It just needs to be called here.

### How to fix

```java
@Override
public void end(boolean isInterrupted) {
    System.out.println("SHOOTER END INTERRUPTED:" + isInterrupted);
    flywheelMotorLeader.stopFlywheel();   // ← ADD THIS LINE
}
```

Note: the field is named `flywheelMotorLeader` in this command but it holds a reference to the **subsystem** (`ShooterFlywheelSubsystem`), not to the motor directly. The method call is therefore `flywheelMotorLeader.stopFlywheel()` (calling the subsystem's public method).

### Why this works
`stopFlywheel()` calls `flywheelMotorLeader.stopMotor()` on the SparkMax/SparkFlex inside the subsystem, which commands 0 V immediately regardless of the previous set-point.

### Testing guidance
1. Hold the operator A button (flywheel on). Confirm the flywheel spins up.
2. Release the button. Confirm the flywheel **immediately begins to coast / brake** instead of continuing to spin.
3. Check `SmartDashboard → Flywheel Velocity` decays toward 0 after button release.

---

## 🟠 BUG-07 · Fix: Driver X button is bound to hub-back alignment instead of outpost alignment

### File
`src/main/java/frc/robot/RobotContainer.java`

### Current code (lines 135–140)
```java
RobotContainer.driverController.x().whileTrue(
  new AlignRobotBackWithHubFieldCommand(swerveSubsystem,
    () -> -RobotContainer.driverController.getLeftY(),
    () -> -RobotContainer.driverController.getLeftX()
  )
);
```

### What is wrong
`AlignRobotBackWithHubFieldCommand` aligns the robot so its **back** faces the hub. This is intended for shooting — the turret is in the back and needs line-of-sight to the hub. However, during intake from the outpost, the robot needs its **front** roller to face the outpost, not its back.

The commented-out block directly above this (lines 126–133) shows the original intent was to use `AlignWithTrench(90)` for the X button. The hub-back alignment was placed here as a temporary substitute but never removed.

Drivers pressing X expecting outpost alignment will:
1. Find the robot aligns its back (not intake) toward the target.
2. Be unable to intake from the outpost with this button held.

### How to fix

**Option A — Restore the original outpost alignment** (recommended if outpost pickup is used in competition):

```java
// Replace the current X binding with:
RobotContainer.driverController.x().whileTrue(
  new AlignWithTrench(
    RobotContainer.swerveSubsystem,
    () -> -RobotContainer.driverController.getLeftY(),
    () -> -RobotContainer.driverController.getLeftX(),
    90   // ← outpost direction (confirm with drive team whether 90 or 270 is correct side)
  )
);
```

**Option B — Keep hub-back alignment but move it to a different button** that makes sense for shooting, and free X for intake alignment. Consult the drive team for the correct button layout.

### Why this matters
Button-to-action mismatch causes driver confusion during matches and leads to incorrect robot positioning. The fix is a one-line change to the angle argument or the command class.

### Testing guidance
1. Press X while driving toward the outpost.
2. Confirm the **intake rollers** (front of robot) face the outpost, not the hub.
3. Verify the driver can still collect balls with the intake while X is held.

---

## 🟠 BUG-08 · Fix: getSimpleAuto() only drives — shooting code is commented out

### File
`src/main/java/frc/robot/commands/Autos.java`

### Current code (lines 618–622)
```java
    // Command aimAtHubCommand =
    //   new AlignRobotBackWithHubFieldCommand(swerveSubsystem,
    //     () -> -RobotContainer.driverController.getLeftY(),
    //     () -> -RobotContainer.driverController.getLeftX()
    //   );
    // return path1.andThen(aimAtHubCommand.withTimeout(5)).andThen(normalShoot.withTimeout(5).andThen(forceShoot));
    return path1;    // ← only drives 2 m forward, no shooting
```

### What is wrong
The method builds complete shooting command chains (`normalShoot`, `forceShoot`) but returns only `path1` (drive 2 metres forward and rotate 180°). In autonomous the robot will drive forward and stop, scoring zero balls.

The commented-out return statement includes hub alignment and a guarded shoot sequence. This was likely commented out temporarily for testing and never restored.

**Important note on the aim command:** `AlignRobotBackWithHubFieldCommand` takes two joystick suppliers. In autonomous there is no driver input, so passing `() -> driverController.getLeftY()` is incorrect (the driver controller is inactive). The correct approach is to pass `() -> 0.0` for both speed suppliers so the robot stops moving while aiming.

### How to fix

**Step 1:** Uncomment the `aimAtHubCommand` definition but use `() -> 0.0` suppliers (no driver input in auto):

```java
Command aimAtHubCommand =
    new AlignRobotBackWithHubFieldCommand(swerveSubsystem,
        () -> 0.0,   // ← no driver input during auto
        () -> 0.0
    );
```

**Step 2:** Change the return statement from `return path1;` to:

```java
return path1
    .andThen(aimAtHubCommand.withTimeout(5))
    .andThen(normalShoot.withTimeout(5)
    .andThen(forceShoot.withTimeout(2)));
```

The sequence is:
1. Drive 2 m forward and rotate 180° (`path1`).
2. Aim the robot's back toward the hub (max 5 seconds).
3. Spin up flywheel and index only when ready (`normalShoot`, max 5 seconds).
4. Force-index the remaining ball regardless of flywheel speed (`forceShoot`, max 2 seconds).

### Why this works
The guarded `normalShoot` uses `RobotContainer.isReadyToShoot()` — which requires flywheel, turret, and hood to all be within tolerance — before running the indexer. This prevents premature shots. The `forceShoot` fallback guarantees the ball exits even if the flywheel is slightly off, which is acceptable at the end of a short auto.

### Testing guidance
1. Select *"simple"* auto from the dashboard.
2. Run in autonomous. Verify the robot drives forward, aligns with the hub, and fires the pre-loaded ball.
3. Check SmartDashboard `flywheel ready`, `turret ready`, `hood ready` booleans transition to `true` before the indexer runs.

---

## 🟡 BUG-09 · Fix: Flywheel state machine passes mode codes as RPM values

### File
`src/main/java/frc/robot/subsystems/ShooterFlywheelSubsystem.java`

### Current code (periodic(), lines 125–129)
```java
if (state == ShooterStates.NONE) {
    // do nothing
} else {
    setShooterVelocity(state);   // ← BUG: state is 1 or 2, not an RPM
}
```

`ShooterStates` values (from Constants.java):
```
NONE      = 0
AUTO_BLUE = 1
AUTO_RED  = 2
```

### What is wrong
When `setState(ShooterStates.AUTO_BLUE)` is called (value = `1.0`), `periodic()` calls `setShooterVelocity(1.0)`, commanding the flywheel to spin at **1 RPM**. This is below the idle threshold and effectively the same as being stopped.

This is currently latent (harmless) because `setState()` is only called with `NONE = 0` from `disabledInit()`. However, if any autonomous routine or future code calls `setState(AUTO_BLUE)` expecting the flywheel to spin up, it will silently fail.

The correct pattern is already used in `ShooterHoodSubsystem.periodic()` and `ShooterTurretSubsystem.periodic()`, which call `RobotContainer.swerveSubsystem.getTurretToTarget*()` to get the actual target value.

### How to fix

In `ShooterFlywheelSubsystem.periodic()`, replace the `setShooterVelocity(state)` call with a call to the actual target RPM from the swerve subsystem:

```java
if (state == ShooterStates.NONE) {
    // do nothing
} else {
    // state is a mode flag (AUTO_BLUE / AUTO_RED), not an RPM.
    // Retrieve the real target RPM from the swerve subsystem's lookup table.
    setShooterVelocity(RobotContainer.swerveSubsystem.getTurretToTargetRPMValue());
}
```

**Alternative approach (more flexible):** Change the `state` field to store the actual target RPM directly instead of a mode code:

```java
// In setState():
public void setState(double shooterState) {
    if (this.state != shooterState) {
        this.state = shooterState;   // store the RPM, not the mode code
    }
}

// In periodic():
if (state == ShooterStates.NONE) {
    // do nothing
} else {
    setShooterVelocity(state);   // now state IS the RPM
}
```

With this approach, callers would do `setState(-3800.0)` instead of `setState(ShooterStates.AUTO_BLUE)`. This is cleaner and matches how the command-based approach already works.

Choose whichever pattern best matches the team's intended use. The first option (calling `getTurretToTargetRPMValue()`) is the minimal change.

### Testing guidance
1. Call `setState(ShooterStates.AUTO_BLUE)` from a test routine.
2. Read `SmartDashboard → Flywheel Velocity`. It should spin up to the distance-based RPM (e.g., `−3500`) rather than staying near 0.

---

## 🟡 BUG-10 · Fix: Arm PID / feedforward gains are all zero

### File
`src/main/java/frc/robot/Constants.java`

### Current code (lines 227–229)
```java
public static final double armkP = 0.00;
public static final double armkG = 0.00;
public static final double armkV = 0.00;
```

### What is wrong
`IntakeArmSubsystem.setIntakeArmAngle()` calls:
```java
runIntakeArm(intakeArmPidController.calculate(position, angle));
```

With `armkP = 0`, the PID output is always `0.0` regardless of the position error. The `ArmFeedforward` with `kG = 0` and `kV = 0` also outputs `0.0`. The state-machine control path (via `setState()` → `periodic()` → `setIntakeArmAngle()`) therefore **produces zero motor output** no matter what angle is commanded.

Currently the robot still moves its arm because all teleop and auto bindings bypass the state machine and use `ManualIntakeArmCommand` with hardcoded duty-cycle values (`0.19`, `−0.19`, `0.1`). But:
- Any code that calls `setState(IntakeArmStates.PICKUP)` to let the arm move automatically will have no effect.
- The `didReachState()` method always returns `false` (the PID controller never reaches the set-point because it never moves).

### How to fix

**Step 1: Measure the arm physically.** Before entering any values, connect the arm motor to a bench test and measure:
- `kG` (gravity constant): Apply 0 % power. With the arm horizontal, measure the voltage/duty-cycle needed to hold the arm level. This is `kG`.
- `kV` (velocity feedforward): Drive the arm at a known constant velocity (read from the encoder). `kV = voltage / velocity`.
- `kP`: Start at a small value and increase until the arm holds its set-point without oscillating. A good starting point for a TalonFX with a reduction is around `kP = 0.001` to `0.01` in duty-cycle-per-encoder-unit.

**Step 2: Set the constants.** Replace the three zeros with measured values. Example (these are illustrative — tune on the robot):

```java
public static final double armkP = 0.005;   // ← tune: start here and adjust
public static final double armkG = 0.07;    // ← tune: gravity compensation
public static final double armkV = 0.0;     // ← tune: set if velocity feedforward needed
```

**Step 3: Add SmartDashboard tuning support.** To tune without redeploying code, add a similar pattern to what is already in `ShooterTurretSubsystem.periodic()`:

```java
// In IntakeArmSubsystem.periodic() — add tuning reads:
double kP = SmartDashboard.getNumber("kP arm", IntakeConstants.armkP);
SmartDashboard.putNumber("kP arm", kP);
if (kP != intakeArmPidController.getP()) {
    intakeArmPidController.setP(kP);
}
```

### Testing guidance
1. Deploy with non-zero `armkP`. Command the arm to `IntakeArmStates.PICKUP` (encoder value `1.0`).
2. Confirm `SmartDashboard → Intake Arm position` moves toward `1.0` and the arm physically reaches the pickup position.
3. Confirm `didReachState()` returns `true` once at set-point.

---

## 🟡 BUG-11 · Fix: Dead auto methods can never be selected

### File
`src/main/java/frc/robot/commands/Autos.java`

### Affected methods
- `getRedTrenchRight()` — line ~490
- `getRedRightFull1()` — line ~541

### What is wrong
Both methods are implemented but are **not** listed in:
1. The `AUTO` enum (line 64–66)
2. The `chooser.addOption()` calls (lines 86–100)
3. The `switch` statement inside `getAuto()` (lines 120–138)

They can never be selected from the dashboard and will never run during a match. Any changes made to these methods have zero effect on the robot.

### How to fix

**Option A — Add them to the chooser (if they are intended for competition):**

1. Add entries to the `AUTO` enum:
```java
public enum AUTO {
    BLUE_TRENCH_LEFT_NEUTRAL, BLUE_CENTER_TOWER, BLUE_TRENCH_RIGHT_OUTPOST, BLUE_TRENCH_RIGHT_NEUTRAL,
    RED_TRENCH_LEFT_NEUTRAL, RED_CENTER_TOWER, RED_TRENCH_RIGHT_NEUTRAL, RED_TRENCH_RIGHT_OUTPOST, RED_CENTER_TOWER_R,
    PRACTICE_FIELD, SIMPLE_AUTO, TUNE_AUTO, BLUE_RIGHT_AUTO_FULL_1, BLUE_RIGHT_AUTO_FULL_2,
    RED_TRENCH_RIGHT,        // ← ADD
    RED_RIGHT_FULL_1         // ← ADD
}
```

2. Add chooser options:
```java
chooser.addOption("Red Right Trench", AUTO.RED_TRENCH_RIGHT);         // ← ADD
chooser.addOption("Red Right Full 1", AUTO.RED_RIGHT_FULL_1);         // ← ADD
```

3. Add cases to the switch:
```java
case RED_TRENCH_RIGHT   -> getRedTrenchRight();     // ← ADD
case RED_RIGHT_FULL_1   -> getRedRightFull1();      // ← ADD
```

**Option B — Delete them (if they are obsolete):** Remove both method bodies from the file. This reduces confusion and eliminates the risk of someone accidentally referencing them.

### Testing guidance
After Option A: The two new entries appear in the auto chooser on SmartDashboard. Selecting them runs the correct method.

---

## 🟡 BUG-12 · Fix: Blue/Red Full-1 autos end race group at wrong pose

### File
`src/main/java/frc/robot/commands/Autos.java`

### Affected methods
- `getBlueRightFull1()` — line ~396
- `getRedRightFull1()` — line ~583

### Current code (both methods have the same bug)
```java
Command path1 = AutoBuilder.pathfindToPose(endPose2d, ...);   // neutral shooting pose
Command path2 = AutoBuilder.pathfindToPose(pickUpPose, ...);  // ball pickup pose
Command path3 = AutoBuilder.pathfindToPose(returnPose, ...);  // return/staging pose
Command path4 = AutoBuilder.pathfindToPose(finalPose, ...);   // final outpost/trench pose

return new ParallelRaceGroup(
    new ParallelCommandGroup(
        new ManualIntakeArmCommand(..., () -> 0.0),
        new ManualIntakeRoller(..., () -> 0.20)
    ),
    path1.andThen(path2.andThen(path3.andThen(path1)))   // ← BUG: path1 not path4
).andThen(
    new ParallelCommandGroup(
        path4,   // ← this path4 runs AFTER the race group
        ...
    )
);
```

### What is wrong
The race group's path sequence ends at `path1` (the neutral shooting pose) instead of `path4` (the final pose). This means:

1. The race group drives: neutral → pickup → return → **neutral again** (wrong).
2. The race group ends. The robot is at the neutral position.
3. The subsequent `andThen` block commands `path4` (outpost/trench) as a *separate* step.

The intended sequence is: neutral → pickup → return → **final outpost** (while intaking the whole time). By returning to `path1` instead of `path4`, the robot:
- Does not collect balls from the final position.
- Drives an extra unnecessary segment back to neutral.
- Then drives all the way to the outpost **after the race group ends** with no intake running.

### How to fix

In both `getBlueRightFull1()` and `getRedRightFull1()`, change the final path inside the race group from `path1` to `path4`:

```java
// Replace:
path1.andThen(path2.andThen(path3.andThen(path1)))

// With:
path1.andThen(path2.andThen(path3.andThen(path4)))
```

The corrected sequence is: neutral → pickup → return → final outpost/trench (all while running the intake).

### Why this works
`path4` is the final destination defined at the top of each method (`finalPose = SHOOTING_POSES.BLUE_OUTPOST_CENTER` or `RED_OUTPOST_CENTER`). Moving there while the intake is running (inside the race group) means balls are collected as the robot arrives. The race group then ends, and the subsequent `andThen` block — which handles alignment and shooting — runs immediately.

### Testing guidance
1. Enable `getBlueRightFull1()` auto (add it to the chooser per BUG-11 fix if needed, or call it directly from a test button).
2. Trace the robot's path. It should proceed: neutral → ball pickup → staging → **outpost**, with the intake roller running throughout.
3. Confirm the robot does not return to `endPose2d` (neutral) a second time before reaching the outpost.

---

## 🔵 BUG-13 · Fix: Unused import in ZeroHoodCommand

### File
`src/main/java/frc/robot/commands/ZeroCommands/ZeroHoodCommand.java`

### Current code (line 7)
```java
import frc.robot.subsystems.ClimbSubsystem;
```

### What is wrong
`ClimbSubsystem` is never referenced anywhere in this file. The import was almost certainly copied from another command file and never cleaned up.

### How to fix

Delete line 7:
```java
// REMOVE this line:
import frc.robot.subsystems.ClimbSubsystem;
```

Also remove the unused `Supplier` import on line 5 if it remains after other changes:
```java
// REMOVE this line too (Supplier<T> is not used in this file):
import java.util.function.Supplier;
```

### Why this matters
While unused imports do not cause runtime errors, they:
- Signal to future maintainers that `ClimbSubsystem` is involved in this file (it is not).
- Are flagged as warnings by most Java IDEs and linters, creating noise that hides real issues.

Most Java IDEs can remove unused imports automatically: in IntelliJ press **Ctrl+Alt+O** (Optimize Imports). In VS Code with the Java extension, run **Organize Imports** from the command palette.

---

## 🔵 BUG-14 · Fix: Duplicate NetworkTable topic names across commands

### Files
- `src/main/java/frc/robot/commands/AlignWithTrench.java` — lines 25–26
- `src/main/java/frc/robot/commands/PickUpFuelCommand.java` — lines 25–26

### Current code (both files, identical)
```java
DoublePublisher xSpeedPublisher = NetworkTableInstance.getDefault()
    .getDoubleTopic("x speed").publish();
DoublePublisher ySpeedPublisher = NetworkTableInstance.getDefault()
    .getDoubleTopic("y speed").publish();
```

### What is wrong
Both commands publish to the **same NetworkTable topic names** (`"x speed"` and `"y speed"`). NetworkTables 4 does not support two publishers to the same topic on the same instance. WPILib will log a publisher conflict warning, and dashboard tools (Glass, Elastic, SmartDashboard) will receive whichever value was published most recently, interleaving data from both commands.

### How to fix

Give each command its own unique topic prefix:

**In `AlignWithTrench.java`:**
```java
DoublePublisher xSpeedPublisher = NetworkTableInstance.getDefault()
    .getDoubleTopic("AlignWithTrench/xSpeed").publish();
DoublePublisher ySpeedPublisher = NetworkTableInstance.getDefault()
    .getDoubleTopic("AlignWithTrench/ySpeed").publish();
```

**In `PickUpFuelCommand.java`:**
```java
DoublePublisher xSpeedPublisher = NetworkTableInstance.getDefault()
    .getDoubleTopic("PickUpFuel/xSpeed").publish();
DoublePublisher ySpeedPublisher = NetworkTableInstance.getDefault()
    .getDoubleTopic("PickUpFuel/ySpeed").publish();
```

### Testing guidance
1. Deploy the fix. Enable a routine that runs `AlignWithTrench` and `PickUpFuelCommand` in sequence.
2. Open Glass or Elastic. Confirm `AlignWithTrench/xSpeed` and `PickUpFuel/xSpeed` appear as separate topics.
3. Verify no NT publisher conflict warnings appear in the RioLog.

---

## Fix Priority Checklist

Use this checklist to track which fixes have been applied and verified.

```
[x] BUG-01  ShooterFlywheelSubsystem.java        — INTENTIONAL: flywheel runs reversed so
                                                      Math.min(positive_kicker, 5000) IS correct.
                                                      No change needed.
[x] BUG-02  ZeroHoodCommand.java                 — FIXED IN CODE: addRequirements(hoodSubsystem) added
[?] BUG-03  Autos.java L218-219                  — use BLUE_* poses in getBlueTrenchLeftNeutral() — still open
[?] BUG-04  Autos.java L646                      — return pathPlannerPath not trajectoryPath — still open
[x] BUG-05  AlignWithTrench.java                 — FIXED IN CODE: local variable targetAngle used
[x] BUG-06  ShooterFlywheelVelocityCommand.java  — FIXED IN CODE: stopFlywheel() added to end()
[?] BUG-07  RobotContainer.java L135             — driver X button intent — confirm with drive team
[?] BUG-08  Autos.java L622                      — restore shooting return in getSimpleAuto() — still open
[?] BUG-09  ShooterFlywheelSubsystem.java L128   — state machine passes mode code as RPM — still open
[x] BUG-10  Constants.java L227-229              — INTENTIONAL: arm PID unused, team uses manual control
[?] BUG-11  Autos.java                           — dead auto methods — still open
[?] BUG-12  Autos.java L396, L583               — path1 vs path4 in race group — still open
[x] BUG-13  ZeroHoodCommand.java                 — FIXED IN CODE: unused imports removed
[x] BUG-14  AlignWithTrench.java + PickUpFuel   — FIXED IN CODE: unique NT topic names
[x] NEW-01  AlginRobotBackWithHubCameraTxCommand — FIXED IN CODE: setModuleStates moved outside if-else
[x] NEW-02  IntakeWhileMoving.java               — FIXED IN CODE: radiansToDegrees not degreesToRadians
[x] NEW-03  ShootCommand.java                    — FIXED IN CODE: stopFlywheel() added to end()
[x] NEW-04  ShooterTurretSubsystem.java          — FIXED IN CODE: direction > 0 (was > 1, never true)
[x] NEW-05  PickUpFuelCommand.java               — FIXED IN CODE: stopModules() added to end()
[x] NEW-06  PickUpFuelCommand.java NT topics     — FIXED IN CODE: unique topic names
[x] NEW-07  AlginRobotBack... NT topics          — FIXED IN CODE: unique topic names
[?] NEW-08  Constants.java kProjectileSpeedMPS   — measure real projectile speed on field
```

---

## Implementation Tips

### Order of operations
Apply fixes **in priority order** (Critical → High → Medium → Low). Each fix is independent; they do not depend on each other. However, BUG-04 and BUG-08 interact: fixing BUG-04 (so `getPathToPose` returns the correct PathPlanner path) is a prerequisite for BUG-08's fix to work correctly, since `getSimpleAuto()` calls `getPathToPose()`.

### Testing after each fix
After each change:
1. Build the project (`./gradlew build`).
2. Deploy to the robot (`./gradlew deploy`).
3. Run the specific test described in the "Testing guidance" section above.
4. Mark the checklist item above as done.

### Code review
Each fix should be reviewed by a second team member before being merged into the main branch. Suggest creating a pull request for each group of fixes so changes can be traced individually.

---

---

# New Bugs Found — SOTM Branch Line-by-Line Review

These bugs were found during the full review of the SOTM branch code on 2026-03-22.
All marked ✅ have been **fixed directly in code** in this same PR.

---

## 🔴 NEW-01 · AlginRobotBackWithHubCameraTxCommand — robot stops when it sees the target ✅ FIXED

### File
`src/main/java/frc/robot/commands/AlginRobotBackWithHubCameraTxCommand.java`

### What was wrong
`setModuleStates()` was only called inside the `else` branch (target NOT visible).
When the camera DID see the AprilTag and computed the correct `chassisSpeeds`, those
speeds were never sent to the swerve modules — the robot effectively stopped the moment
it acquired the target.

```java
// BEFORE (broken)
if (canSeeTarget) {
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnController.calculate(target_x, 0));
    // ← setModuleStates never called here!
} else {
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates);   // only called when NOT seeing target
}
```

### Fix applied
`setModuleStates()` is now called unconditionally after the if-else, using whichever
`chassisSpeeds` was computed:

```java
// AFTER (fixed)
if (canSeeTarget) {
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnController.calculate(target_x, 0));
} else {
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
}
SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
swerveSubsystem.setModuleStates(moduleStates);
```

### Testing guidance
1. Bind this command to a test button in RobotContainer.
2. Hold the button while facing the hub. Confirm the robot drives toward the hub
   rather than stopping.
3. Cover the limelight. Confirm the robot continues to drive straight (no rotation).

---

## 🟠 NEW-02 · IntakeWhileMoving — angle conversion applied backwards ✅ FIXED

### File
`src/main/java/frc/robot/commands/IntakeWhileMoving.java`

### What was wrong
```java
wantedAngle = Units.degreesToRadians(Math.atan2(leftYSupplier.get(), leftXSupplier.get()) + 3*Math.PI/4);
```
`Math.atan2` already returns **radians** (−π to +π). Calling `Units.degreesToRadians()`
on a radian value multiplies it by π/180 again, producing angles like 0.04° — essentially
zero. The drift controller therefore always had an error near zero and never rotated the
robot toward the joystick direction.

### Fix applied
Changed to `Units.radiansToDegrees(...)` so `diffFromWantedAngle` receives the correct
degree value:

```java
wantedAngle = Units.radiansToDegrees(Math.atan2(leftYSupplier.get(), leftXSupplier.get()) + 3*Math.PI/4);
```

### Testing guidance
1. Hold right bumper (IntakeWhileMoving).
2. Push the left stick in various directions. The robot's intake (front face) should
   rotate to face the joystick direction.
3. Verify on Red alliance that the 180° flip applies correctly.

---

## 🟠 NEW-03 · ShootCommand.end() — flywheel left spinning ✅ FIXED

### File
`src/main/java/frc/robot/commands/ShootCommand.java`

### What was wrong
`end()` only printed a message. The flywheel motor was left at whatever voltage the last
`setShooterVelocity()` call commanded, causing it to spin for several seconds after the
command ended.

### Fix applied
Added `flywheel.stopFlywheel()` to `end()`.

### Testing guidance
Run `ShootCommand` for 2 seconds, then release. SmartDashboard `Flywheel Velocity` should
immediately start dropping toward 0 rather than maintaining speed.

---

## 🟠 NEW-04 · ShooterTurretSubsystem — upper spring resistance never fires ✅ FIXED

### File
`src/main/java/frc/robot/subsystems/ShooterTurretSubsystem.java`

### What was wrong
`direction` is computed as `diff / Math.abs(diff)`, which is always exactly `+1.0` or
`-1.0`. The condition `direction > 1` can therefore **never** be `true` — the upper
spring resistance (needed when the turret is near angle 215° and rotating further into
the spring) was silently disabled.

```java
// BEFORE (broken — condition impossible)
} else if (currentAngle >= maxSlack && direction > 1) {
    springFeedForward = direction * springResistance;
}
```

### Fix applied
Changed `> 1` to `> 0` (i.e., direction is `+1`, moving into the upper spring zone):

```java
// AFTER (fixed)
} else if (currentAngle >= maxSlack && direction > 0) {
    springFeedForward = direction * springResistance;
}
```

### Testing guidance
1. Command the turret to an angle above 215°.
2. Then command it to an angle above 215° from the **same** side (so direction is `+1`).
3. Observe that the turret output voltage includes the spring resistance offset — the
   turret should move more smoothly without stalling at the spring boundary.

---

## 🟡 NEW-05 · PickUpFuelCommand.end() — swerve modules never stopped ✅ FIXED

### File
`src/main/java/frc/robot/commands/PickUpFuelCommand.java`

### What was wrong
`end()` only restored the limelight pipeline but never called `swerveSubsystem.stopModules()`.
After the button was released the robot would continue driving at the last-commanded velocity
until the default command (`SwerveJoystick`) took over — a brief but noticeable drift window.

### Fix applied
Added `swerveSubsystem.stopModules()` to `end()`.

---

## 🔵 NEW-06 & NEW-07 · Duplicate NT topics in PickUpFuelCommand and AlginRobotBackWithHubCameraTxCommand ✅ FIXED

### Files
- `src/main/java/frc/robot/commands/PickUpFuelCommand.java`
- `src/main/java/frc/robot/commands/AlginRobotBackWithHubCameraTxCommand.java`

### What was wrong
Both used the topic names `"x speed"` and `"y speed"`, already used by `AlignWithTrench`.
NT4 does not support two publishers on the same topic — whichever command ran last would
win, interleaving data on the dashboard.

### Fix applied
Topics renamed:
- `PickUpFuelCommand`: `"PickUpFuel/xSpeed"` / `"PickUpFuel/ySpeed"`
- `AlginRobotBackWithHubCameraTxCommand`: `"AlignWithHubTx/xSpeed"` / `"AlignWithHubTx/ySpeed"`
- `AlignWithTrench` (from previous PR): `"AlignWithTrench/xSpeed"` / `"AlignWithTrench/ySpeed"`

---

## 💡 NEW-08 · kProjectileSpeedMPS needs field measurement — still needs team action

### File
`src/main/java/frc/robot/Constants.java`

### What is wrong
```java
public static final double kProjectileSpeedMPS = 12.0; // TODO: characterise for real value
```
This constant is used by `SwerveSubsystem.updateTurretAngleDistanceToTarget()` to compute
the virtual-target correction for shoot-while-moving. If it is wrong, the virtual target will
be offset in the wrong direction, causing misses when the robot is moving fast.

`12.0 m/s` is a reasonable placeholder for a flywheel-speed note shooter, but it must be
confirmed on the actual robot.

### How to measure it
1. Place a target 3 metres away.
2. Shoot a note at the target while the robot is stationary.
3. Record the moment of release (from video) and the moment of impact.
4. `kProjectileSpeedMPS = distance / flight_time`.
5. Repeat at 4 m and 5 m and average the values.
6. Typical range for this type of shooter: 10–16 m/s.

### Mechanical suggestion
If your notes consistently fall short when moving toward the target and fly long when
moving away, the constant is too low. Increase it in 0.5 m/s increments and retest until
the correction zeroes out. A telemetry-based approach: log `SWM Angle Correction (deg)`
from SmartDashboard — it should be < 3° at typical robot speeds. If it is > 5°, the
projectile speed estimate is too low.

---

## Mechanical Suggestions (not software)

The following observations are based purely on the code and the robot specification
recorded in Constants.java. No physical inspection was performed.

### 1. Turret spring tension characterisation
The code has `turretFakeFeedForward = 0.45` and `turretSpringResistance = 0.5`, both
adjustable from SmartDashboard. After the NEW-04 fix (upper spring resistance now fires),
you may find the turret overshoots at the upper boundary. Lower `turretSpringResistance`
from the dashboard until the turret moves smoothly through both spring zones without
oscillation. Once tuned, commit the values back to Constants.java.

### 2. Hood zeroing direction
`zeroHood()` runs the motor at `−0.1` (backward). Confirm physically that backward
means "toward the lower mechanical stop" on your robot. If the hood moves the wrong
way when zeroing, change the sign to `+0.1`.

### 3. Intake arm — bump-stop risk
`IntakeArmSubsystem.setState()` is never called from RobotContainer — all arm movement
is manual (`ManualIntakeArmCommand`). The arm soft limits (PID set-point enforcement) are
therefore inactive. If the arm has a physical hard stop, the current limit of 80 A stator /
30 A supply will protect the motor, but you may want to add software stops that match the
physical endpoints to avoid repeated hard-stop impacts.

### 4. Wheel diameter confirmation
`kWheelDiameterMeters = Units.inchesToMeters(4.5)` (4.5 inch wheels). If the robot's
actual wheels are 4 inches (common in ThriftyBot swerve kits), the drive encoder
conversion factors will be off by ~12%, causing all odometry distances to be
over-reported. Measure the wheel diameter with calipers and confirm.

### 5. Camera pitch angles — confirm sign convention
`limelight2Angle = -10` degrees. WPILib's `setCameraPose_RobotSpace` uses positive pitch
= looking up. A value of `−10` means the camera looks slightly downward. Confirm this is
correct for the forward camera placement (if it should look forward/up at AprilTags it
may need to be positive). `limelight3Angle = 15` and `limelight4Angle = 15` look up at
15°, which is correct for alliance-wall AprilTags.
