package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class IntakeWhileMoving extends Command{
    IntakeRollerSubsystem intakeRoller;
    SwerveSubsystem swerveSubsystem;
    Supplier<Double> rollerSpeed, forwardSpeed, rotationSpeed;
    private final SlewRateLimiter xLimiter, turningLimiter;

    public IntakeWhileMoving(IntakeRollerSubsystem intakeRoller, SwerveSubsystem swerveSubsystem, Supplier<Double> rollerSpeed, Supplier<Double> forwardSpeed, Supplier<Double> rotationSpeed) {
        this.intakeRoller = intakeRoller;
        this.swerveSubsystem = swerveSubsystem;
        this.rollerSpeed = rollerSpeed;
        this.forwardSpeed = forwardSpeed;
        this.rotationSpeed = rotationSpeed;
  
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
        double xSpeed = -0.35;
        double speedOffset = -0.35*forwardSpeed.get();
        xSpeed = xSpeed + speedOffset;
        double turningSpeed = rotationSpeed.get();
        
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond  * DriveConstants.teleSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.teleTurnSpeed; 
        
        boolean isBlue = !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
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
