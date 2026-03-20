// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * ShootOnTheMoveCommand – Allows the robot to shoot accurately while driving.
 *
 * <p>The driver retains full control of the swerve drivetrain. At the same time:
 * <ul>
 *   <li>The turret aligns to the vision target.</li>
 *   <li>The hood moves to the vision-calculated position.</li>
 *   <li>The flywheel spins up to the vision-calculated RPM.</li>
 *   <li>The indexer feeds the ball automatically once all systems report ready.</li>
 * </ul>
 */
public class ShootOnTheMoveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterFlywheelSubsystem flywheel;
    private final ShooterTurretSubsystem turret;
    private final ShooterHoodSubsystem hood;
    private final IndexerSubsystem indexer;

    private final Supplier<Double> xSpeedSupplier;
    private final Supplier<Double> ySpeedSupplier;
    private final Supplier<Double> turnSpeedSupplier;
    private final Supplier<Double> flywheelRPMSupplier;
    private final Supplier<Double> turretAngleSupplier;
    private final Supplier<Double> hoodValueSupplier;

    // Indexer voltage used when all shooter systems are ready
    private static final double INDEX_VOLTS = 7.0;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;

    /**
     * Creates a new ShootOnTheMoveCommand.
     *
     * @param swerveSubsystem   The robot's swerve drive subsystem.
     * @param flywheel          The shooter flywheel subsystem.
     * @param turret            The shooter turret subsystem.
     * @param hood              The shooter hood subsystem.
     * @param indexer           The indexer subsystem.
     * @param xSpeedSupplier    Supplier for forward/backward drive speed (field-relative).
     * @param ySpeedSupplier    Supplier for left/right drive speed (field-relative).
     * @param turnSpeedSupplier Supplier for rotational drive speed.
     * @param flywheelRPMSupplier Supplier for the target flywheel RPM (from vision).
     * @param turretAngleSupplier Supplier for the target turret angle in degrees (from vision).
     * @param hoodValueSupplier   Supplier for the target hood position (from vision).
     */
    public ShootOnTheMoveCommand(
            SwerveSubsystem swerveSubsystem,
            ShooterFlywheelSubsystem flywheel,
            ShooterTurretSubsystem turret,
            ShooterHoodSubsystem hood,
            IndexerSubsystem indexer,
            Supplier<Double> xSpeedSupplier,
            Supplier<Double> ySpeedSupplier,
            Supplier<Double> turnSpeedSupplier,
            Supplier<Double> flywheelRPMSupplier,
            Supplier<Double> turretAngleSupplier,
            Supplier<Double> hoodValueSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.flywheel = flywheel;
        this.turret = turret;
        this.hood = hood;
        this.indexer = indexer;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.turnSpeedSupplier = turnSpeedSupplier;
        this.flywheelRPMSupplier = flywheelRPMSupplier;
        this.turretAngleSupplier = turretAngleSupplier;
        this.hoodValueSupplier = hoodValueSupplier;

        xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem, flywheel, turret, hood, indexer);
    }

    @Override
    public void initialize() {
        System.out.println("ShootOnTheMoveCommand initialized");
        SmartDashboard.putBoolean("Shoot On The Move Active", true);
    }

    @Override
    public void execute() {
        // --- Drive (field-relative, same modifiers as normal teleop) ---
        double xSpeed = xLimiter.calculate(xSpeedSupplier.get())
                * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
                * DriveConstants.teleSpeed;
        double ySpeed = yLimiter.calculate(ySpeedSupplier.get())
                * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
                * DriveConstants.teleSpeed;
        double turningSpeed = turningLimiter.calculate(turnSpeedSupplier.get())
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
                * DriveConstants.teleTurnSpeed;

        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
        int flip = isBlue ? 1 : -1;
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                flip * xSpeed, flip * ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);

        // --- Shooter targeting (vision-based) ---
        turret.setTurretAngle(turretAngleSupplier.get());
        hood.setHoodValue(hoodValueSupplier.get());
        flywheel.setShooterVelocity(flywheelRPMSupplier.get());

        // --- Auto-index when flywheel, turret, and hood are all on-target ---
        boolean readyToShoot = RobotContainer.isReadyToShoot();
        indexer.runIndexerAtVoltage(readyToShoot ? INDEX_VOLTS : 0.0);

        // --- Telemetry ---
        SmartDashboard.putBoolean("SOTM Ready To Shoot", readyToShoot);
        SmartDashboard.putNumber("SOTM Flywheel RPM Target", flywheelRPMSupplier.get());
        SmartDashboard.putNumber("SOTM Turret Angle Target", turretAngleSupplier.get());
        SmartDashboard.putNumber("SOTM Hood Value Target", hoodValueSupplier.get());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        flywheel.stopFlywheel();
        turret.runTurret(0);
        hood.runHood(0);
        indexer.runIndexerAtVoltage(0);
        SmartDashboard.putBoolean("Shoot On The Move Active", false);
        System.out.println("ShootOnTheMoveCommand ended. Interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
