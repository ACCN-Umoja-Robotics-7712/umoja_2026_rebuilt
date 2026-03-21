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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class AlignWithOutpost extends Command{
    SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter;
  private double wantedAngle;

  private final PIDController turnController = new PIDController(DriveConstants.kPAlignTrench, DriveConstants.kIAlignTrench, 0);

    public AlignWithOutpost(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction){
        this.swerveSubsystem = swerveSubsystem;

        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
    
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

        boolean isBlue = !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
        wantedAngle = isBlue ? 180 : 0;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("Align with outpost Initialized");
    }

    @Override
    public void execute(){
      double xSpeed = xSpdFunction.get();
      double ySpeed = ySpdFunction.get();
      boolean canSeeTarget = LimelightHelpers.getTV(Constants.LimelightConstants.LIMELIGHT_FORWARD);
      if (canSeeTarget){
          double x_offset= LimelightHelpers.getTX(Constants.LimelightConstants.LIMELIGHT_FORWARD);
          double size_tag = LimelightHelpers.getTA(Constants.LimelightConstants.LIMELIGHT_FORWARD);
          double forwardError = Constants.AutoConstants.wantedOutpostArea - size_tag;
          double sideError = 0 - x_offset;
          swerveSubsystem.alignWithPID(forwardError, sideError, turnController.calculate(RobotContainer.diffFromWantedAngle(wantedAngle), 0));
      } else {
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnController.calculate(RobotContainer.diffFromWantedAngle(wantedAngle), 0));
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
      }
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Align with outpost is interrupted:" + isInterrupted);
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
