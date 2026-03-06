package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AlignRobotBackWithHubFieldCommand extends Command {
    SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction;
    private final SlewRateLimiter xLimiter, yLimiter;

    DoublePublisher xSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("x speed").publish();
    DoublePublisher ySpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("y speed").publish();
  
    private final PIDController turnController = new PIDController(DriveConstants.kPDrift, DriveConstants.kIDrift, 0);

    public AlignRobotBackWithHubFieldCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction){
        this.swerveSubsystem = swerveSubsystem;

        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
    
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("Align with Hub Field Initialized");
    }

    @Override
    public void execute(){
        // boolean canSeeTarget = LimelightHelpers.getTV(Constants.LimelightConstants.tagName);
        // 1. Get joystic inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        // 2. Apply deadband
          // xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
          // ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
          if (Math.abs(xSpeed) + Math.abs(ySpeed) < OIConstants.kDeadband) {
            xSpeed = 0.0;
            ySpeed = 0.0;
          }

          // 3. Make the driving smoother
          if (!(RobotContainer.driverController.rightBumper().getAsBoolean())){
            xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
            ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
          } else if (RobotContainer.driverController.leftBumper().getAsBoolean()){
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
          } else{
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond  * DriveConstants.teleSpeed;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.teleSpeed;
          }

        // boolean canSeeTarget = LimelightHelpers.getTV(Constants.LimelightConstants.gamePieceName);
        // if (canSeeTarget){
        //     double target_x = LimelightHelpers.getTX(Constants.LimelightConstants.gamePieceName);
        //     swerveSubsystem.alignWithTag(target_x, ySpeed, turnController.calculate(RobotContainer.diffFromWantedAngle(wantedAngle), 0));
        // } else {
          double wantedAngle = RobotContainer.swerveSubsystem.getTurretToTargetAngle();
          ChassisSpeeds chassisSpeeds;
          boolean isBlue = !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
          int flipAlliance = isBlue ? 1 : -1;
          
          // xSpeedPublisher.accept(hoodMotor.getAbsoluteEncoder().getPosition());

          SmartDashboard.putNumber("WANTED TARGET ANGLE USED BY BUTTON", wantedAngle);
          SmartDashboard.putNumber("CURRENT ROBOT ANGLE USED BY BUTTON", RobotContainer.swerveSubsystem.getHeading());
          SmartDashboard.putNumber("CURRENT GLOBAL ROBOT ANGLE USED BY BUTTON", RobotContainer.swerveSubsystem.getGlobalHeading());
          double turningSpeed = turnController.calculate(RobotContainer.diffFromWantedAngle(wantedAngle));
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(flipAlliance*xSpeed, flipAlliance*ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
          swerveSubsystem.setModuleStates(moduleStates);
        // }
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Align with Hub Field end is interrupted:" + isInterrupted);
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
