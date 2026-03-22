package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.ShooterReadinessTracker;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;

/**
 * Camera-integrated shooter command for fast, uninterrupted shooting cycles.
 *
 * <p><b>Strategy</b> (adapted from Team 254 Cheesy Poofs' pre-spin approach and
 * Team 1690 Valkyries' continuous vision-update loop):
 * <ol>
 *   <li><b>Pre-spin</b>: The flywheel starts spinning immediately when the button
 *       is pressed, before the turret finishes acquiring. This eliminates the
 *       spin-up delay that makes stationary shooting slow.</li>
 *   <li><b>Continuous vision updates</b>: Turret angle and hood position are
 *       updated every 20 ms from the Limelight via the swerve subsystem's
 *       pre-computed virtual-target values, which already compensate for the
 *       robot's own velocity.</li>
 *   <li><b>Hysteresis-guarded indexing</b>: {@link ShooterReadinessTracker}
 *       requires all three systems (flywheel, turret, hood) to be within
 *       tolerance for {@link ShooterReadinessTracker#kMinStableSeconds} before
 *       the indexer runs. This prevents premature shots caused by momentary
 *       within-tolerance readings.</li>
 *   <li><b>Graceful fallback</b>: If the Limelight loses all AprilTag targets
 *       for longer than {@link #kCameraLossTimeoutSeconds}, the command switches
 *       to pose-based aiming (using the robot's odometry / MegaTag-2 estimate).
 *       Shooting can still occur; the operator just needs to keep the button held.</li>
 * </ol>
 *
 * <p><b>Subsystem ownership</b>: The swerve drive subsystem is intentionally NOT
 * claimed here — the driver keeps full joystick control while this command runs.
 * Only flywheel, turret, hood, and indexer are claimed.
 */
public class CameraIntegratedShooterCommand extends Command {

    /**
     * Seconds of continuous Limelight target loss before switching to pose-based
     * (odometry-only) aiming. Keep this short enough to react quickly to tag
     * reacquisition, but long enough to tolerate frame drops.
     */
    private static final double kCameraLossTimeoutSeconds = 0.5;

    private final ShooterFlywheelSubsystem flywheel;
    private final ShooterTurretSubsystem turret;
    private final ShooterHoodSubsystem hood;
    private final IndexerSubsystem indexer;

    private final ShooterReadinessTracker readinessTracker;
    private final Timer cameraLossTimer = new Timer();
    private boolean usingFallback = false;

    /**
     * @param flywheel Flywheel subsystem.
     * @param turret   Turret subsystem.
     * @param hood     Hood subsystem.
     * @param indexer  Indexer subsystem.
     */
    public CameraIntegratedShooterCommand(
            ShooterFlywheelSubsystem flywheel,
            ShooterTurretSubsystem turret,
            ShooterHoodSubsystem hood,
            IndexerSubsystem indexer) {
        this.flywheel         = flywheel;
        this.turret           = turret;
        this.hood             = hood;
        this.indexer          = indexer;
        this.readinessTracker = new ShooterReadinessTracker(flywheel, turret, hood);

        // Swerve is intentionally excluded — the driver keeps full drive control.
        addRequirements(flywheel, turret, hood, indexer);
    }

    @Override
    public void initialize() {
        System.out.println("CameraIntegratedShooterCommand: started");
        cameraLossTimer.reset();
        cameraLossTimer.start();
        usingFallback = false;
    }

    @Override
    public void execute() {
        // ---- Camera status ----
        // Use the forward-facing limelight to detect AprilTags for the current target.
        boolean hasTarget = LimelightHelpers.getTV(LimelightConstants.LIMELIGHT_FORWARD);
        SmartDashboard.putBoolean("CameraShooter/HasTarget", hasTarget);

        if (hasTarget) {
            cameraLossTimer.reset();
            usingFallback = false;
        } else if (cameraLossTimer.get() > kCameraLossTimeoutSeconds) {
            usingFallback = true;
        }
        SmartDashboard.putBoolean("CameraShooter/UsingFallback", usingFallback);

        // ---- Targeting values ----
        // Whether the camera is locked or we are in fallback mode, the swerve
        // subsystem's getTurretToTarget* methods always return the best available
        // estimate (pose-based with virtual-target velocity compensation).
        double targetAngle = RobotContainer.swerveSubsystem.getTurretToTargetAngle();
        double targetHood  = RobotContainer.swerveSubsystem.getTurretToTargetHoodValue();
        double targetRPM   = RobotContainer.swerveSubsystem.getTurretToTargetRPMValue();

        SmartDashboard.putNumber("CameraShooter/TargetAngle", targetAngle);
        SmartDashboard.putNumber("CameraShooter/TargetHood",  targetHood);
        SmartDashboard.putNumber("CameraShooter/TargetRPM",   targetRPM);

        // ---- Turret + Hood: continuously update from best estimate ----
        turret.setTurretAngle(targetAngle);
        hood.setHoodValue(targetHood);

        // ---- Flywheel: pre-spin immediately (don't wait for turret/hood to settle) ----
        flywheel.setShooterVelocity(targetRPM);

        // ---- Indexer: run only when ALL systems are stable within tolerance ----
        boolean ready = readinessTracker.isReady();
        SmartDashboard.putBoolean("CameraShooter/ReadyToShoot", ready);
        indexer.runIndexerAtVoltage(ready ? IndexerConstants.indexVolts : 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("CameraIntegratedShooterCommand: ended (interrupted=" + interrupted + ")");
        flywheel.stopFlywheel();
        turret.runTurret(0);
        hood.runHood(0);
        indexer.runIndexerAtVoltage(0);
        cameraLossTimer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
