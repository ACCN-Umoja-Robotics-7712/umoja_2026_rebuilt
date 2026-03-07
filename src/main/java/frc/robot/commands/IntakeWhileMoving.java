package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class IntakeWhileMoving extends Command{
    IntakeRollerSubsystem intakeRoller;
    SwerveSubsystem swerveSubsystem;
    Supplier<Double> rollerSpeed, forwardSpeed, leftXSupplier, leftYSupplier;

    private final PIDController driftController = new PIDController(DriveConstants.kPDrift, DriveConstants.kIDrift, 0);

    private final SlewRateLimiter xLimiter, turningLimiter;

    public IntakeWhileMoving(IntakeRollerSubsystem intakeRoller, SwerveSubsystem swerveSubsystem, Supplier<Double> rollerSpeed, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier, Supplier<Double> rightYSupplier) {
        this.intakeRoller = intakeRoller;
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSpeed = rollerSpeed;
        this.forwardSpeed = rightYSupplier;
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
  
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        
        addRequirements(intakeRoller, swerveSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("Intake while moving Initialized");
    }

    @Override
    public void execute(){
        // run rollers
        intakeRoller.runIntake(rollerSpeed.get());

        // drive at set speed forward, can control forward speed and rotation
        // set speed of 0.35 m/s forward
        double xSpeed = -0.30;
        double speedOffset = -0.30*forwardSpeed.get();
        xSpeed = xSpeed + speedOffset;

        double wantedAngle = swerveSubsystem.getHeading();
        wantedAngle = Units.degreesToRadians(Math.atan2(leftYSupplier.get(), leftXSupplier.get()) + 3*Math.PI/4);
        
        boolean isBlue = !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
        if (!isBlue) {
            wantedAngle += 180;
        }
        double diff = RobotContainer.diffFromWantedAngle(wantedAngle);
        double turningSpeed = driftController.calculate(diff, 0);
        
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond  * DriveConstants.teleSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.teleTurnSpeed; 
        
        int flip = isBlue ? 1 : -1;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(flip*xSpeed, 0, turningSpeed);
    
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Intake while moving end interrupted:" + isInterrupted);
        intakeRoller.runIntake(0);
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
