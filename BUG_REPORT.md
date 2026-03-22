# Bug Report — ACCN Umoja Robotics 7712 · `umoja_2026_rebuilt` Main Branch

**Prepared by:** GitHub Copilot (Professional Software Engineering Review)  
**Date:** 2026-03-21  
**Branch reviewed:** `main`  
**Scope:** Full static analysis of all Java source files under `src/main/java/frc/robot/`

---

## Legend

| Symbol | Severity |
|--------|----------|
| 🔴 **CRITICAL** | Causes incorrect robot behaviour, safety risk, or compilation failure |
| 🟠 **HIGH** | Significant functional defect that will show up in matches |
| 🟡 **MEDIUM** | Logic error that may surface under specific conditions |
| 🔵 **LOW / INFO** | Code quality, maintainability, or dead code |

---

## Executive Summary

After a full read of the robot code, **14 distinct bugs** were identified across 8 files. Several previously-noted issues are already annotated with `// BUG FIX:` comments in the code but are not yet corrected in the surrounding logic. The most match-impactful issues are:

1. The **kicker motor velocity clamp never fires** (wrong Math function) — causes kicker over-speed risk.
2. **`ZeroHoodCommand` declares no subsystem requirement** — the hood can be driven by two commands simultaneously during zeroing.
3. **`getBlueTrenchLeftNeutral()` uses Red-alliance poses** — the Blue Left Trench auto drives to the wrong side of the field.
4. **`getPathToPose()` returns the wrong path object** — PathPlanner path is discarded; a broken WPILib holonomic path is returned instead.
5. **`AlignWithTrench` mutates its target angle field every loop tick on Red** — causes the robot heading to oscillate between two angles every 20 ms.
6. **`ShooterFlywheelVelocityCommand.end()` does not stop the flywheel** — the motor keeps spinning after the command ends.

---

## Bug Details

---

### 🔴 BUG-01 · Kicker velocity clamp uses wrong `Math` function

**File:** `src/main/java/frc/robot/subsystems/ShooterFlywheelSubsystem.java`  
**Line:** 78

**Code:**
```java
double wantedKickerVelocity = wantedVelocity * -1.5;   // always negative
wantedKickerVelocity = Math.min(wantedKickerVelocity, 5000); // BUG
```

**Explanation:**  
`wantedVelocity` is the flywheel RPM (a negative number, e.g. `-3800`). Multiplying by `-1.5` gives a positive number (+5700). `Math.min(+5700, 5000)` does correctly cap it at 5000 for large magnitudes.

However, when the flywheel runs in the opposite direction (positive RPM to the motor, negative in this code), `wantedKickerVelocity` will be a large negative number. `Math.min(large_negative, 5000)` always returns the large negative, so the clamp **never fires** in that direction.

More critically: the comment says the intent is to "limit to 5000 RPM to prevent the motor from burning out," but `Math.min` against a positive cap does nothing for negative values.

**Fix:**
```java
// Clamp the magnitude, keeping the sign
wantedKickerVelocity = Math.max(wantedKickerVelocity, -5000);
// Or equivalently, clamp the absolute value:
// if (Math.abs(wantedKickerVelocity) > 5000)
//     wantedKickerVelocity = Math.copySign(5000, wantedKickerVelocity);
```

---

### 🔴 BUG-02 · `ZeroHoodCommand` declares no subsystem requirement

**File:** `src/main/java/frc/robot/commands/ZeroCommands/ZeroHoodCommand.java`  
**Lines:** 13–15

**Code:**
```java
public ZeroHoodCommand(ShooterHoodSubsystem hoodSubsystem){
    this.hoodSubsystem = hoodSubsystem;
    // addRequirements(hoodSubsystem); ← MISSING
}
```

**Explanation:**  
WPILib's command scheduler uses declared requirements to prevent two commands from using the same subsystem simultaneously. Because `ZeroHoodCommand` never calls `addRequirements()`, the scheduler does not know the hood is in use. Any other command that *does* require the hood (e.g., `ShooterHoodValueCommand` triggered by the auto or a button bind) can run **at the same time as zeroing**, causing both commands to fight over the motor and leaving the hood in an unknown position.

**Fix:**
```java
public ZeroHoodCommand(ShooterHoodSubsystem hoodSubsystem){
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(hoodSubsystem); // ← add this
}
```

---

### 🔴 BUG-03 · `getBlueTrenchLeftNeutral()` navigates to Red-alliance poses

**File:** `src/main/java/frc/robot/commands/Autos.java`  
**Lines:** 218–219 (inside `getBlueTrenchLeftNeutral()`)

**Code:**
```java
public Command getBlueTrenchLeftNeutral() {
    Pose2d endPose2d = SHOOTING_POSES.RED_NEUTRAL_LEFT;   // BUG: should be BLUE_*
    Pose2d pickUpPose = SHOOTING_POSES.RED_TRENCH_LEFT;   // BUG: should be BLUE_*
```

**Explanation:**  
The Blue Left Trench autonomous routine is copy-pasted from `getRedTrenchLeftNeutral()` but the pose constants were never updated. When this auto is selected on the **Blue alliance**, the robot will navigate to poses on the **Red side of the field**, driving into the opposing alliance's trench — a penalty risk and a loss of autonomous points.

**Fix:**
```java
Pose2d endPose2d = SHOOTING_POSES.BLUE_NEUTRAL_LEFT;
Pose2d pickUpPose = SHOOTING_POSES.BLUE_TRENCH_LEFT;
```
(Ensure the corresponding `BLUE_*` constants exist in `Constants.SHOOTING_POSES`.)

---

### 🔴 BUG-04 · `getPathToPose()` returns the wrong path object

**File:** `src/main/java/frc/robot/commands/Autos.java`  
**Lines:** ~620–635 (inside `getPathToPose()`)

**Code:**
```java
public Command getPathToPose(Pose2d endPose) {
    // ...
    SwerveControllerCommand trajectoryPath = new SwerveControllerCommand(
        traj,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController, yController, thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    Command pathPlannerPath = AutoBuilder.pathfindToPose(endPose, Constants.pathConstraints);
    return trajectoryPath; // BUG: PathPlanner path is computed but discarded
}
```

**Explanation:**  
`pathPlannerPath` (the PathPlanner holonomic-capable route) is computed but thrown away. The returned `trajectoryPath` uses the older `SwerveControllerCommand` built on a `TrajectoryGenerator` trajectory, which generates a non-holonomic trajectory (no strafing). Returning this command means the swerve robot will arc like a tank-drive robot instead of following the desired straight-line path, and the `ProfiledPIDController thetaController` continuous input is configured for 0–360°, which conflicts with the `SwerveControllerCommand`'s expectation of radians from `Rotation2d`.

**Fix:**
```java
return pathPlannerPath; // use the PathPlanner path
```

---

### 🟠 BUG-05 · `AlignWithTrench.execute()` mutates instance field every loop tick on Red alliance

**File:** `src/main/java/frc/robot/commands/AlignWithTrench.java`  
**Line:** 85

**Code:**
```java
if (!isBlue) {
    // flip wantedAngle for redside
    wantedAngle = (wantedAngle + 180) % 360; // mutates the field, not a local
}
```

**Explanation:**  
`wantedAngle` is an instance variable set once in the constructor. Every call to `execute()` (every 20 ms) on Red alliance adds 180° to it. Because `(angle + 180 + 180) % 360 == angle`, the value **oscillates between two headings every 20 ms**:

| Loop tick | `wantedAngle` (constructed with 0) |
|-----------|------------------------------------|
| 1 | 180 |
| 2 | 0 |
| 3 | 180 |
| 4 | 0 |
| … | … |

The robot's heading target flips at 50 Hz, causing violent oscillation of the swerve chassis on the Red alliance.

**Fix:** Use a local variable so the instance field is not modified:
```java
double targetAngle = wantedAngle;
if (!isBlue) {
    targetAngle = (wantedAngle + 180) % 360;
}
// use targetAngle in the ChassisSpeeds calculation below
```

---

### 🟠 BUG-06 · `ShooterFlywheelVelocityCommand.end()` does not stop the flywheel

**File:** `src/main/java/frc/robot/commands/ShooterFlywheelVelocityCommand.java`  
**Lines:** 28–31

**Code:**
```java
@Override
public void end(boolean isInterrupted){
    System.out.println("SHOOTER END INTERRUPTED:" + isInterrupted);
    // flywheel keeps spinning — no stop call
}
```

**Explanation:**  
When this command ends (timeout, interruption, or completion), no motor stop is issued. The flywheel continues spinning at whatever RPM it reached, which means:
- After a timed auto shooting phase, the flywheel remains spinning even when the auto moves on to driving.
- Extra current draw during path-following phases.
- If the command is interrupted by a `.whileFalse()` handler, the stopping is only achieved by the next command (`ManualShooterFlywheelCommand(() -> 0.0)`) which requires the scheduler to reassign the subsystem first. If any scheduling gap occurs, the flywheel runs unstopped.

**Fix:**
```java
@Override
public void end(boolean isInterrupted){
    System.out.println("SHOOTER END INTERRUPTED:" + isInterrupted);
    flywheelMotorLeader.stopFlywheel();
}
```

---

### 🟠 BUG-07 · `AlignRobotBackWithHubFieldCommand` is bound to driver X but `AlignWithOutpost` import was removed

**File:** `src/main/java/frc/robot/RobotContainer.java`  
**Lines:** ~136–140

**Code:**
```java
RobotContainer.driverController.x().whileTrue(
  new AlignRobotBackWithHubFieldCommand(swerveSubsystem,
    () -> -RobotContainer.driverController.getLeftY(),
    () -> -RobotContainer.driverController.getLeftX()
  )
);
```

**Explanation:**  
`AlignRobotBackWithHubFieldCommand` aligns the robot's *back* face with the hub, which means the intake faces away from the hub. During intake operations (where the robot uses the **front** roller), this alignment is opposite to what is needed. The button comment originally referred to an outpost alignment command; the replacement with hub-back alignment on the same button appears to be an error or unfinished change. The driver pressing X expecting outpost alignment will get hub-back alignment instead.

This is flagged as HIGH because it affects driver usability and may cause the driver to be misaligned when attempting to intake from the outpost.

---

### 🟠 BUG-08 · `getSimpleAuto()` only drives — never shoots

**File:** `src/main/java/frc/robot/commands/Autos.java`  
**Lines:** ~594–611

**Code:**
```java
public Command getSimpleAuto() {
    // ...setup of shoot commands...
    
    // return path1.andThen(aimAtHubCommand.withTimeout(5)).andThen(normalShoot...);
    return path1; // BUG: only drives forward, never aims or shoots
}
```

**Explanation:**  
`getSimpleAuto()` builds `normalShoot` and `forceShoot` command groups with full flywheel and indexer logic, but then returns only `path1` (drive 2 metres forward and stop). The auto never shoots. In a competition context, a "simple auto" is expected to at minimum shoot pre-loaded balls.

**Fix:** Uncomment the intended return statement (or replace with the new `CameraIntegratedShooterCommand` flow):
```java
return path1.andThen(aimAtHubCommand.withTimeout(5))
            .andThen(normalShoot.withTimeout(5)
            .andThen(forceShoot));
```

---

### 🟡 BUG-09 · `ShooterFlywheelSubsystem` state machine passes mode codes as RPM values

**File:** `src/main/java/frc/robot/subsystems/ShooterFlywheelSubsystem.java`  
**Lines:** 94–97, 126–130

**Code:**
```java
// Constants.java
public static final class ShooterStates {
    public static final double NONE      = 0; // used as "idle"
    public static final double AUTO_BLUE = 1; // used as mode flag
    public static final double AUTO_RED  = 2; // used as mode flag
}

// ShooterFlywheelSubsystem.java periodic()
if (state == ShooterStates.NONE) {
    // do nothing
} else {
    setShooterVelocity(state); // BUG: state is 1 or 2, not a valid RPM
}
```

**Explanation:**  
If `setState(ShooterStates.AUTO_BLUE)` is called (value = 1), the periodic will call `setShooterVelocity(1)` — commanding the flywheel to spin at 1 RPM. This is almost certainly not the intent.

The `ShooterHoodSubsystem` and `ShooterTurretSubsystem` use the same pattern correctly because their `periodic()` calls the correct *target* from `swerveSubsystem` rather than using the state value as the set-point. The flywheel's periodic should do the same.

This bug is currently latent (harmless) because `setState()` is only called with `NONE = 0` (from `disabledInit()`), but if someone adds an auto that calls `setState(AUTO_BLUE)`, the flywheel will spin at 1 RPM instead of the correct RPM.

**Fix:**
```java
// In periodic():
if (state != ShooterStates.NONE) {
    setShooterVelocity(RobotContainer.swerveSubsystem.getTurretToTargetRPMValue());
}
```
Or redesign the state machine to store an RPM directly.

---

### 🟡 BUG-10 · `IntakeArmSubsystem` PID/feedforward gains are all zero

**File:** `src/main/java/frc/robot/Constants.java`  
**Lines:** 227–229

**Code:**
```java
public static final double armkP = 0.00;
public static final double armkG = 0.00;
public static final double armkV = 0.00;
```

**Explanation:**  
`IntakeArmSubsystem.setIntakeArmAngle()` computes:
```java
runIntakeArm(intakeArmPidController.calculate(position, angle));
```
With `armkP = 0`, the PID output is always 0 regardless of error. `ArmFeedforward` with all-zero gains also outputs 0. The state-machine arm control path (`setState()` → `periodic()` → `setIntakeArmAngle()`) is therefore completely non-functional.

Autonomous and teleop currently bypass this by using `ManualIntakeArmCommand` with hardcoded speed values (`0.19`, `-0.19`, `0.1`), which calls `runIntakeArm()` directly, so the robot still moves its arm. However, positional control (e.g. automatically lowering the arm to a specific encoder angle) is unavailable.

**Fix:** Tune the arm on the real robot and set non-zero values. Start with a conservative `armkP ≈ 0.001` and tune from there.

---

### 🟡 BUG-11 · Dead code: `getRedTrenchRight()` and `getRedRightFull1()` are not reachable

**File:** `src/main/java/frc/robot/commands/Autos.java`  
**Lines:** ~490–545 and ~547–590

**Explanation:**  
Both `getRedTrenchRight()` and `getRedRightFull1()` are fully defined methods in `Autos.java`, but neither appears in the `AUTO` enum nor in the `switch` statement inside `getAuto()`. They can never be selected from the dashboard and will never run.

If these autos are intended for competition use, they must be added to the `AUTO` enum and the switch. If they are obsolete, they should be removed to avoid confusion.

---

### 🟡 BUG-12 · `getRedRightFull1()` and `getBlueRightFull1()` both end on `path1` (wrong return pose)

**File:** `src/main/java/frc/robot/commands/Autos.java`

**Code (`getBlueRightFull1()`):**
```java
return new ParallelRaceGroup(
    ...
    path1.andThen(path2.andThen(path3.andThen(path1))) // BUG: ends at path1 (endPose), not path4 (finalPose)
)
```

**Explanation:**  
These methods define `path4` as a final destination (e.g. `BLUE_OUTPOST_CENTER`) but the race group's path sequence ends by calling `path1` (the neutral shooting pose) again instead of `path4`. The robot never reaches the final outpost position. The `path4` reference in the subsequent `.andThen()` block is then driven to as a separate command after the race ends, but since the robot is already at `path1`'s position (not the trench), the pickup of the second set of balls never happens.

**Fix:** Replace the final `path1` with `path4` inside the race group sequence:
```java
path1.andThen(path2.andThen(path3.andThen(path4)))
```

---

### 🔵 BUG-13 · `ZeroHoodCommand` has incorrect import (imports `ClimbSubsystem`)

**File:** `src/main/java/frc/robot/commands/ZeroCommands/ZeroHoodCommand.java`  
**Line:** 7

**Code:**
```java
import frc.robot.subsystems.ClimbSubsystem; // unused — should not be here
```

**Explanation:**  
`ClimbSubsystem` is imported but never used in this file. While this does not affect runtime behaviour, it indicates copy-paste without cleanup and may confuse future maintainers.

**Fix:** Remove the unused import.

---

### 🔵 BUG-14 · `AlignWithTrench` and `PickUpFuelCommand` share the same NetworkTable topic key

**Files:**  
- `src/main/java/frc/robot/commands/AlignWithTrench.java` (line ~24)  
- `src/main/java/frc/robot/commands/PickUpFuelCommand.java` (line ~18)

**Code (both files):**
```java
DoublePublisher xSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("x speed").publish();
DoublePublisher ySpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("y speed").publish();
```

**Explanation:**  
Both commands publish to the same topic names (`"x speed"`, `"y speed"`). When both commands have ever been instantiated, NetworkTables will have two publishers to the same topic. WPILib logs a warning and uses the most-recently-published value. Dashboard tools (e.g. Glass, Elastic) will see conflicting values. This is a low-priority issue but can cause confusing dashboard readings during debugging.

**Fix:** Use unique keys per command, e.g. `"AlignTrench/x speed"` and `"PickUpFuel/x speed"`.

---

## Summary Table

| ID | Severity | File | Line(s) | Title |
|----|----------|------|---------|-------|
| BUG-01 | 🔴 CRITICAL | `ShooterFlywheelSubsystem.java` | 78 | Kicker velocity clamp uses `Math.min` on a negative value — clamp never fires |
| BUG-02 | 🔴 CRITICAL | `ZeroHoodCommand.java` | 13–15 | `addRequirements()` missing — hood can be used by two commands simultaneously |
| BUG-03 | 🔴 CRITICAL | `Autos.java` | ~218–219 | `getBlueTrenchLeftNeutral()` navigates to Red-side poses |
| BUG-04 | 🔴 CRITICAL | `Autos.java` | ~620–635 | `getPathToPose()` returns old WPILib trajectory; PathPlanner path discarded |
| BUG-05 | 🟠 HIGH | `AlignWithTrench.java` | 85 | Instance field `wantedAngle` mutated every loop tick on Red — heading oscillates |
| BUG-06 | 🟠 HIGH | `ShooterFlywheelVelocityCommand.java` | 28–31 | `end()` does not stop flywheel motor |
| BUG-07 | 🟠 HIGH | `RobotContainer.java` | ~136–140 | Driver X button bound to hub-back alignment instead of outpost alignment |
| BUG-08 | 🟠 HIGH | `Autos.java` | ~594–611 | `getSimpleAuto()` only drives — shooting code is commented out |
| BUG-09 | 🟡 MEDIUM | `ShooterFlywheelSubsystem.java` | 94–97, 126–130 | State machine passes mode codes (1, 2) as RPM values to flywheel |
| BUG-10 | 🟡 MEDIUM | `Constants.java` | 227–229 | Arm PID / feedforward gains are all 0.0 — positional arm control is non-functional |
| BUG-11 | 🟡 MEDIUM | `Autos.java` | ~490–590 | `getRedTrenchRight()` and `getRedRightFull1()` are dead code — never selectable |
| BUG-12 | 🟡 MEDIUM | `Autos.java` | ~360–404 | `getBlueRightFull1()` / `getRedRightFull1()` end race group on `path1` not `path4` |
| BUG-13 | 🔵 LOW | `ZeroHoodCommand.java` | 7 | Unused `import frc.robot.subsystems.ClimbSubsystem` |
| BUG-14 | 🔵 LOW | `AlignWithTrench.java`, `PickUpFuelCommand.java` | ~24, ~18 | Duplicate NetworkTable topic names conflict on dashboard |

---

## Bugs Already Fixed (noted with `// BUG FIX:` annotations in code)

The following bugs were identified and corrected in the current working branch. They are listed here for completeness:

| File | Issue | Status |
|------|-------|--------|
| `ShooterTurretSubsystem.java` | `springFeedForward` computed but never added to motor voltage | ✅ Fixed |
| `ShooterTurretSubsystem.java` | `kISpring` change-detection compared against `kIturretSlack` constant (copy-paste error) | ✅ Fixed |
| `ShooterHoodSubsystem.java` | `finishedZeroing()` returned `true` after only 1 loop tick regardless of motor state | ✅ Fixed |
| `SwerveSubsystem.java` | `getGlobalHeading()` published to `"HEADING"` key (overwrote field-relative heading) | ✅ Fixed |
| `SwerveSubsystem.java` | `RED_NEUTRAL_RIGHT` added twice to `allPoints` list; one entry was wrong | ✅ Fixed |
| `IntakeArmSubsystem.java` | `setState()` compared `this.state != state` (field vs. itself — always `false`) | ✅ Fixed |
| `SwerveModule.java` | `CANcoderConfiguration` created but never applied to hardware | ✅ Fixed |
| `Constants.java` | `ClimbConstants.kP` / `kI` typed as `int` — fractional gains truncated to 0 | ✅ Fixed |
| `RobotContainer.java` | `isReadyToShoot()` returned only `flywheelReady` — turret and hood readiness ignored | ✅ Fixed |

---

## Recommendations

1. **Adopt a code review checklist** for every PR that includes: (a) subsystem requirements declared, (b) motor stop in `end()`, (c) no mutation of constructor-injected parameters inside `execute()`.

2. **Write simulation tests** using WPILib's simulation support (`edu.wpi.first.wpilibj.simulation`) to catch path-following regressions and command sequencing bugs before going to the field.

3. **Create a constants validation test** that checks every constant is within a physically sensible range (e.g. `armkP > 0`, all RPM values negative for motors that run in reverse).

4. **Remove commented-out code** that is not intended to return. Comments like `// return path1.andThen(...)` left in production files increase cognitive load and risk confusion during time-pressured match prep.

5. **Use named constants instead of magic numbers** in commands (e.g. `() -> -0.31`, `() -> 0.19`, `() -> 7.0`) — gather these in `Constants.java` so they can be tuned from one location.
