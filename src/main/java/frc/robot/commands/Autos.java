package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
    private final SwerveSubsystem swerveSubsystem = RobotContainer.swerveSubsystem;

    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(DriveConstants.kDriveKinematics);
    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, AutoConstants.kIThetaController, 0, AutoConstants.kThetaControllerConstraints);

    PathPlannerPath depot_trench_to_neutral;
    
    public enum AUTO {
        BLUE_DRIVER_LEFT, BLUE_CENTER, BLUE_DRIVER_RIGHT,
        RED_DRIVER_LEFT, RED_CENTER, RED_DRIVER_RIGHT,
        PRACTICE_FIELD, SIMPLE_AUTO, TUNE_AUTO
    }
    private SendableChooser<AUTO> chooser;
    private SendableChooser<Command> ppChooser;
    
    public Autos(){

    try {
        // simple_auto = PathPlannerPath/

    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }

        // Create the auto chooser
        chooser = new SendableChooser<AUTO>();
        System.out.println("AUTO SETUP ~~~~~~~~~~~~~~~~~~~");
        // Add options to the chooser
        chooser.addOption("Blue Driver Left", AUTO.BLUE_DRIVER_LEFT);
        chooser.addOption("Blue Center", AUTO.BLUE_CENTER);
        chooser.addOption("Blue Driver Right", AUTO.BLUE_DRIVER_RIGHT);
        chooser.addOption("Red Driver Left", AUTO.RED_DRIVER_LEFT);
        chooser.addOption("Red Center", AUTO.RED_CENTER);
        chooser.addOption("Red Driver Right", AUTO.RED_DRIVER_RIGHT);
        // chooser.addOption("Blue Center Double", AUTO.BLUE_CENTER_DOUBLE);
        chooser.addOption("Practice field center", AUTO.PRACTICE_FIELD);
        chooser.addOption("simple", AUTO.SIMPLE_AUTO);
        chooser.addOption("tune", AUTO.TUNE_AUTO);
        chooser.setDefaultOption("Default", null);
        
        ppChooser = AutoBuilder.buildAutoChooser();
        // // Put the auto chooser on the dashboard
        SmartDashboard.putData("AUTOS", chooser);
        SmartDashboard.putData("PP AUTOS", ppChooser);
        
        // 1. Create trajectory settings
        this.trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        thetaController.enableContinuousInput(0, 360);
    }

    public Command getAuto() {
        // return ppChooser.getSelected();
        AUTO auto = chooser.getSelected();
        if (auto == null) {
            System.out.println("auto is null");
            return new InstantCommand();
        }

        return switch (auto) {
            // case BLUE_DRIVER_LEFT -> getBlueDriverLeft();
            // case BLUE_CENTER -> getBlueCenter();
            // case BLUE_DRIVER_RIGHT -> getBlueDriverRight();
            // case RED_DRIVER_LEFT -> getRedDriverLeft();
            // case RED_CENTER -> getRedCenter();
            // case RED_DRIVER_RIGHT -> getRedDriverRight();
            // case BLUE_CENTER_DOUBLE -> getBlueCenterDouble();
            // case PRACTICE_FIELD -> getPracticeField();
            case SIMPLE_AUTO -> getSimpleAuto();
            case TUNE_AUTO -> getTuneAuto();
            default -> new InstantCommand();
        };
    }

    public Command getTuneAuto() {
        // drive forward/side 1 meter, turn 60 degrees
        return AutoBuilder.pathfindToPose(swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 1, 1, 60), Constants.pathConstraints);
    }

    // public Command getSimpleAuto() {
    //     return getStationCommmand(simple_auto);
    // }

  public Command getSimpleAuto() {
    // stationPosePublisher.set(endPose);

    edu.wpi.first.math.trajectory.Trajectory traj = TrajectoryGenerator.generateTrajectory(
      swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
      List.of(),
      swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 1, 0),
      trajectoryConfig);

    // 4. Construct command to follow trajectory 
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        traj,
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);
        return swerveControllerCommand;
  }
}
