package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.ShooterReadinessTracker;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Enables the driver to drive the robot normally while the shooter auto-aims and
 * automatically indexes as soon as all systems are ready.
 *
 * <p>This command claims all five subsystems simultaneously (swerve, flywheel,
 * turret, hood, indexer) so that each subsystem's default command is safely
 * interrupted for the duration of the shot. Releasing the trigger button restores
 * all subsystems to their default commands.
 *
 * <p>Driving strategy: driver joystick suppliers are passed in directly — the
 * driver maintains full field-oriented control exactly as in {@link SwerveJoystick}.
 *
 * <p>Aiming strategy: turret angle, hood position, and flywheel RPM are all
 * supplied by {@link SwerveSubsystem}'s pre-computed targeting values, which
 * already apply virtual-target compensation for the robot's own velocity
 * (shoot-while-moving correction). See
 * {@code SwerveSubsystem.updateTurretAngleDistanceToTarget()} for details.
 *
 * <p>Indexing strategy: {@link ShooterReadinessTracker} requires all three systems
 * to be within tolerance for {@link ShooterReadinessTracker#kMinStableSeconds}
 * before the indexer runs, preventing premature shots caused by transient
 * within-tolerance readings (hysteresis).
 *
 * <p>Inspired by Team 3476's parallel command composition and Team 254's flywheel
 * pre-spin approach.
 */
public class ShootOnTheMoveCommand extends Command {

    private final SwerveSubsystem swerve;
    private final ShooterFlywheelSubsystem flywheel;
    private final ShooterTurretSubsystem turret;
    private final ShooterHoodSubsystem hood;
    private final IndexerSubsystem indexer;

    private final Supplier<Double> xSpeedSupplier;
    private final Supplier<Double> ySpeedSupplier;
    private final Supplier<Double> turningSpeedSupplier;
    private final Supplier<Double> rpmSupplier;

    private final ShooterReadinessTracker readinessTracker;

    // Rate limiters mirror SwerveJoystick so driving feels the same during shooting.
    private final SlewRateLimiter xLimiter =
            new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter yLimiter =
            new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter turningLimiter =
            new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    /**
     * @param swerve              Swerve drive subsystem — driver joystick inputs are
     *                            forwarded here so the driver keeps full control.
     * @param flywheel            Flywheel subsystem.
     * @param turret              Turret subsystem.
     * @param hood                Hood subsystem.
     * @param indexer             Indexer subsystem.
     * @param xSpeedSupplier      Forward drive speed supplier (driver left-Y axis).
     * @param ySpeedSupplier      Strafe drive speed supplier (driver left-X axis).
     * @param turningSpeedSupplier Rotation speed supplier (driver right-X axis).
     * @param rpmSupplier         Target flywheel RPM supplier
     *                            (e.g. {@code swerveSubsystem::getTurretToTargetRPMValue}).
     */
    public ShootOnTheMoveCommand(
            SwerveSubsystem swerve,
            ShooterFlywheelSubsystem flywheel,
            ShooterTurretSubsystem turret,
            ShooterHoodSubsystem hood,
            IndexerSubsystem indexer,
            Supplier<Double> xSpeedSupplier,
            Supplier<Double> ySpeedSupplier,
            Supplier<Double> turningSpeedSupplier,
            Supplier<Double> rpmSupplier) {
        this.swerve              = swerve;
        this.flywheel            = flywheel;
        this.turret              = turret;
        this.hood                = hood;
        this.indexer             = indexer;
        this.xSpeedSupplier      = xSpeedSupplier;
        this.ySpeedSupplier      = ySpeedSupplier;
        this.turningSpeedSupplier = turningSpeedSupplier;
        this.rpmSupplier         = rpmSupplier;
        this.readinessTracker    = new ShooterReadinessTracker(flywheel, turret, hood);

        addRequirements(swerve, flywheel, turret, hood, indexer);
    }

    @Override
    public void initialize() {
        System.out.println("ShootOnTheMoveCommand: started");
    }

    @Override
    public void execute() {
        // ---- Drive (field-oriented, same feel as SwerveJoystick) ----
        double xSpeed      = xSpeedSupplier.get();
        double ySpeed      = ySpeedSupplier.get();
        double turningSpeed = turningSpeedSupplier.get();

        xSpeed       = xLimiter.calculate(xSpeed)
                * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.teleSpeed;
        ySpeed       = yLimiter.calculate(ySpeed)
                * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.teleSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.teleTurnSpeed;

        // Alliance detection: defaults to Blue when the DS has not yet set an alliance.
        // "!equals(Red)" is intentionally used so that an unknown alliance behaves the
        // same as Blue (conservative default — field orientation is correct for Blue side).
        boolean isBlue = !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
        int flip = isBlue ? 1 : -1;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                flip * xSpeed, flip * ySpeed, turningSpeed, swerve.getRotation2d());
        SwerveModuleState[] moduleStates =
                DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(moduleStates);

        // ---- Flywheel: spin up to the distance-based target RPM ----
        flywheel.setShooterVelocity(rpmSupplier.get());

        // ---- Turret + Hood: auto-aim using virtual-target corrected values ----
        turret.setTurretAngle(RobotContainer.swerveSubsystem.getTurretToTargetAngle());
        hood.setHoodValue(RobotContainer.swerveSubsystem.getTurretToTargetHoodValue());

        // ---- Indexer: run only when all systems are stable and ready ----
        boolean ready = readinessTracker.isReady();
        SmartDashboard.putBoolean("ShootOnTheMove/Ready", ready);
        indexer.runIndexerAtVoltage(ready ? IndexerConstants.indexVolts : 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ShootOnTheMoveCommand: ended (interrupted=" + interrupted + ")");
        swerve.stopModules();
        flywheel.stopFlywheel();
        turret.runTurret(0);
        hood.runHood(0);
        indexer.runIndexerAtVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
