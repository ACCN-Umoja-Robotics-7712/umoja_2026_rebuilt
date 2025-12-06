// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.XBoxConstants;
import frc.robot.Constants.USB;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class SwerveJoystick extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private double autoTimer = 0; 

  Joystick j = new Joystick(USB.DRIVER_CONTROLLER);

  private final PIDController driftController = new PIDController(DriveConstants.kPDrift, DriveConstants.kIDrift, 0);
  
  
  /** Creates a new SwerveJoystick. */
  public SwerveJoystick(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFuntion) {

      this.swerveSubsystem = swerveSubsystem;
      this.xSpdFunction = xSpdFunction;
      this.ySpdFunction = ySpdFunction;
      this.turningSpdFunction = turningSpdFuntion;
  
      this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (DriverStation.getMatchType() != MatchType.None && DriverStation.getMatchTime() <= 30) {
          j.setRumble(RumbleType.kBothRumble, 0.2);
      } else {
          j.setRumble(RumbleType.kBothRumble, 0);
      }

      Boolean aButtonPressed = j.getRawButton(XBoxConstants.A);
      Boolean bButtonPressed = j.getRawButton(XBoxConstants.B);
      Boolean xButtonPressed = j.getRawButton(XBoxConstants.X);
      Boolean yButtonPressed = j.getRawButton(XBoxConstants.Y);
        RobotContainer.currentTrajectory = null;
        RobotContainer.goalPose = null;
      
      
          // 1. Get joystic inputs
          double xSpeed = xSpdFunction.get();
          double ySpeed = ySpdFunction.get();
          double turningSpeed = turningSpdFunction.get();
      
          // 2. Apply deadband
          // xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
          // ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
          if (Math.abs(xSpeed) + Math.abs(ySpeed) < OIConstants.kDeadband) {
            xSpeed = 0.0;
            ySpeed = 0.0;
          }
          turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
          
          SmartDashboard.putNumber("Wanted angle", RobotContainer.wantedAngle);
      
          boolean isRobotOrientatedDrive = RobotContainer.driverController.getRawAxis(XBoxConstants.RT) >= 0.5;
          // 3. Make the driving smoother
          if (!(RobotContainer.driverController.rightBumper().getAsBoolean()) || isRobotOrientatedDrive){
            xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
            ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.kSlowButtonDriveModifier);
            turningSpeed = turningLimiter.calculate(turningSpeed) * (DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.kSlowButtonTurnModifier);
          } else if (RobotContainer.driverController.leftBumper().getAsBoolean()){
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond; 
          }
          else{
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond  * DriveConstants.teleSpeed;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * DriveConstants.teleSpeed;
            turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * DriveConstants.teleTurnSpeed; 
          }
      
          // set current angle
          if (RobotContainer.wantedAngle == -1 || RobotContainer.shouldAutoFixDrift == 2 || yButtonPressed) {
            // auto fix drift
            if (RobotContainer.shouldAutoFixDrift == 1) {
              RobotContainer.wantedAngle = swerveSubsystem.getHeading();
              // RobotContainer.wantedAngle = 0; 

              // align
            } else {
              RobotContainer.wantedAngle = -1;
            }
          }

          boolean drift = RobotContainer.shouldAutoFixDrift == 1 && turningSpeed == 0;
          boolean align = RobotContainer.shouldAutoFixDrift == 2 && turningSpeed == 0;
          if (drift || align) {
            // Fixes negative angles from Pose2d
            SmartDashboard.putNumber("JoystickTurn", turningSpeed);
            if (RobotContainer.wantedAngle < 0) {
              RobotContainer.wantedAngle += 360;
            }
            // fix drift
            // if drifting
            if (RobotContainer.wantedAngle != swerveSubsystem.getHeading()) {
              // > 180 we want to go towards 0 but from negative to account for closest angle
              double currentAngle = swerveSubsystem.getHeading();
              double diff = Math.abs(currentAngle - RobotContainer.wantedAngle);
              if (diff > 180) {
                diff = 360 - diff;
                // since diff is > 180, it passes the 360-0 range
                // if we have 359 and want 1, we go positive, so no change
                // if we have 1 and want 359, we go negative, so multiple -1
                if (RobotContainer.wantedAngle > 180) {
                  diff = diff * -1;
                }
              } else {
                // if going from 5 to 10, do nothing
                // if going from 10 to 5, we go negative so multiple -1
                if (currentAngle > RobotContainer.wantedAngle) {
                  diff = diff * -1;
                }
              }

              turningSpeed = driftController.calculate(-diff, 0);
              
              // only fix drift when moving
              // if (joystickX == 0 && joystickY == 0) {
              //   turningSpeed = 0;
              // }
            }
          } else {
            // set new angle
            RobotContainer.wantedAngle = -1;
          }

          // 4. Construct desired chassis speeds
          ChassisSpeeds chassisSpeeds;
          if (!isRobotOrientatedDrive) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
          } else {
            boolean isBlue = !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
            int flip = isBlue ? -1 : 1;
            chassisSpeeds = new ChassisSpeeds(flip*xSpeed, flip*ySpeed, turningSpeed);
          }

          // 5. Convert chassis speeds to individual module states
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
          swerveSubsystem.setModuleStates(moduleStates); // TODO: add simple auto time thing

          if(j.getRawButtonPressed(XBoxConstants.PAGE)){
            swerveSubsystem.zeroHeading();
          }
          if(j.getRawButtonPressed(XBoxConstants.MENU)){
            RobotContainer.shouldAutoFixDrift += 1;
            if (RobotContainer.shouldAutoFixDrift > 2) {
              RobotContainer.shouldAutoFixDrift = 0;
            }
            System.out.println("AUTO FIX DRIFT TURNED " + Integer.toString(RobotContainer.shouldAutoFixDrift));
          }
          SmartDashboard.putBoolean("Auto Fix Align", RobotContainer.shouldAutoFixDrift == 2);
          SmartDashboard.putBoolean("Auto Fix Drift", RobotContainer.shouldAutoFixDrift == 1);
        }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
