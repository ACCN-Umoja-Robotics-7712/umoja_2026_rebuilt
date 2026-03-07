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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class AlginRobotBackWithHubCameraTxCommand extends Command{
    SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter;
  private double wantedAngle;

    DoublePublisher xSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("x speed").publish();
    DoublePublisher ySpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("y speed").publish();
  
  private final PIDController turnController = new PIDController(DriveConstants.kPAlignTrench, DriveConstants.kIAlignTrench, 0);

    public AlginRobotBackWithHubCameraTxCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction){
        this.swerveSubsystem = swerveSubsystem;

        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
    
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

        this.wantedAngle = (RobotContainer.swerveSubsystem.getTurretToTargetAngle() + 180) % 360;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("Align with Hub Tx Initialized");
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
          xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond  * DriveConstants.kSlowButtonDriveModifier;
          ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.teleSpeed;

          ChassisSpeeds chassisSpeeds;
          boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue);
          String limelightName = Constants.LimelightConstants.limelight4;
          if (isBlue) {
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, Constants.LimelightConstants.BLUE_HUB_CENTER_APRIL_TAG_IDS);
            LimelightHelpers.setPriorityTagID(limelightName, Constants.LimelightConstants.BLUE_HUB_FRONT_CENTER_APRIL_TAG_ID);
          } else {
            LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, Constants.LimelightConstants.RED_HUB_CENTER_APRIL_TAG_IDS);
            LimelightHelpers.setPriorityTagID(limelightName, Constants.LimelightConstants.RED_HUB_FRONT_CENTER_APRIL_TAG_ID);
          }
          boolean canSeeTarget = LimelightHelpers.getTV(limelightName);
          if (canSeeTarget){
            double target_x = LimelightHelpers.getTX(limelightName);
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnController.calculate(target_x, 0));
          } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Align with Hub Tx end is interrupted:" + isInterrupted);
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
