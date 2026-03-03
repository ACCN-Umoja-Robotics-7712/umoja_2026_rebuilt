package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SHOOTING_POSES;
import frc.robot.RobotContainer;
import frc.robot.commands.ManualCommands.ManualIntakeRoller;
import frc.robot.commands.ManualCommands.ManualShooterFlywheelCommand;
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
        BLUE_TRENCH_LEFT_NEUTRAL, BLUE_CENTER_TOWER, BLUE_TRENCH_RIGHT_NEUTRAL,
        RED_TRENCH_LEFT_NEUTRAL, RED_CENTER_TOWER, RED_TRENCH_RIGHT_NEUTRAL,
        PRACTICE_FIELD, SIMPLE_AUTO, TUNE_AUTO
    }
    private SendableChooser<AUTO> chooser;
    private SendableChooser<Command> ppChooser;
    
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Auto end pose", Pose2d.struct).publish();
    
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
        chooser.addOption("Blue Left Trench to Neutral", AUTO.BLUE_TRENCH_LEFT_NEUTRAL);
        chooser.addOption("Blue Center to Tower", AUTO.BLUE_CENTER_TOWER);
        chooser.addOption("Blue Right Trench to Neutral", AUTO.BLUE_TRENCH_RIGHT_NEUTRAL);
        chooser.addOption("Red Trench Left to Neutral", AUTO.RED_TRENCH_LEFT_NEUTRAL);
        chooser.addOption("Red Center to Tower", AUTO.RED_CENTER_TOWER);
        chooser.addOption("Red Right Trench to Neutral", AUTO.RED_TRENCH_RIGHT_NEUTRAL);
        chooser.addOption("Practice field center", AUTO.PRACTICE_FIELD);
        chooser.addOption("simple", AUTO.SIMPLE_AUTO);
        chooser.addOption("tune", AUTO.TUNE_AUTO);
        chooser.setDefaultOption("Default", null);
        
        // ppChooser = AutoBuilder.buildAutoChooser();
        // // Put the auto chooser on the dashboard
        SmartDashboard.putData("AUTOS", chooser);
        // SmartDashboard.putData("PP AUTOS", ppChooser);
        
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
            case BLUE_TRENCH_LEFT_NEUTRAL -> getBlueTrenchLeftNeutral();
            // case BLUE_CENTER_TOWER -> getBlueCenter();
            case BLUE_TRENCH_RIGHT_NEUTRAL -> getBlueTrenchRightNeutral();
            case RED_TRENCH_LEFT_NEUTRAL -> getRedTrenchLeftNeutral();
            case RED_CENTER_TOWER -> getRedTower();
            case RED_TRENCH_RIGHT_NEUTRAL -> getRedTrenchRightNeutral();
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

    public Command getRedTrenchRightNeutral() {
        Pose2d centerPose2d = SHOOTING_POSES.RED_NEUTRAL_RIGHT;
        Pose2d pickUpPose = SHOOTING_POSES.RED_NEUTRAL_RIGHT_PICKUP;
        Pose2d beforePickUpPose = SHOOTING_POSES.RED_TRENCH_RIGHT;
        posePublisher.set(centerPose2d);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(
        swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
        List.of(),
        centerPose2d,
        trajectoryConfig);
        

        Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        centerPose2d,
        List.of(),
        pickUpPose,
        trajectoryConfig);

        Command path1 = AutoBuilder.pathfindToPose(beforePickUpPose, Constants.pathConstraints);
        Command path2 = AutoBuilder.pathfindToPose(centerPose2d, Constants.pathConstraints);
        Command path3 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
        
        return path1.andThen(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.40),
                    path2
                ),
                new ParallelCommandGroup(
                    new ManualShooterFlywheelCommand(RobotContainer.shooterFlywheelSubsystem, () -> 0.45),
                    path3
                )
            )
        );

    }

    public Command getRedTrenchLeftNeutral() {
        Pose2d endPose2d = SHOOTING_POSES.RED_NEUTRAL_LEFT;
        Pose2d pickUpPose = SHOOTING_POSES.RED_TRENCH_LEFT;
        posePublisher.set(endPose2d);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(
        swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
        List.of(),
        endPose2d,
        trajectoryConfig);
        

        Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        endPose2d,
        List.of(),
        pickUpPose,
        trajectoryConfig);

        Command path1 = AutoBuilder.pathfindToPose(endPose2d, Constants.pathConstraints);
        Command path2 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
        
        return path1.andThen(
            new ParallelCommandGroup(
                new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.40),
                path2
                )
        );

    }
    
    public Command getBlueTrenchLeftNeutral() {
        Pose2d endPose2d = SHOOTING_POSES.RED_NEUTRAL_LEFT;
        Pose2d pickUpPose = SHOOTING_POSES.RED_TRENCH_LEFT;
        posePublisher.set(endPose2d);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(
        swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
        List.of(),
        endPose2d,
        trajectoryConfig);
        

        Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        endPose2d,
        List.of(),
        pickUpPose,
        trajectoryConfig);

        Command path1 = AutoBuilder.pathfindToPose(endPose2d, Constants.pathConstraints);
        Command path2 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
        
        return path1.andThen(
            new ParallelCommandGroup(
                new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.40),
                path2
                )
        );

    }

    public Command getRedTower() {
        Pose2d endPose2d = SHOOTING_POSES.RED_TOWER_CENTER;
        Pose2d pickUpPose = SHOOTING_POSES.RED_HUB_CENTER;
        posePublisher.set(endPose2d);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(
        swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
        List.of(),
        endPose2d,
        trajectoryConfig);
        

        Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        endPose2d,
        List.of(),
        pickUpPose,
        trajectoryConfig);

        Command path1 = AutoBuilder.pathfindToPose(endPose2d, Constants.pathConstraints);
        Command path2 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
        
        return path1.andThen(
            new ParallelCommandGroup(
                new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.40),
                path2
                )
        );

    }

    public Command getBlueTrenchRightNeutral() {
        Pose2d endPose2d = SHOOTING_POSES.BLUE_NEUTRAL_RIGHT;
        Pose2d pickUpPose = SHOOTING_POSES.BLUE_TRENCH_RIGHT;
        posePublisher.set(endPose2d);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(
        swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
        List.of(),
        endPose2d,
        trajectoryConfig);
        

        Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        endPose2d,
        List.of(),
        pickUpPose,
        trajectoryConfig);

        Command path1 = AutoBuilder.pathfindToPose(endPose2d, Constants.pathConstraints);
        Command path2 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
        
        return path1.andThen(
            new ParallelCommandGroup(
                new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.40),
                path2
                )
        );

    }

  public Command getSimpleAuto() {

    Pose2d endPose = swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), -0.25, 5.5, -90);
    Pose2d pickUpPose = swerveSubsystem.offsetPoint(endPose, 0, 4,  0);
    posePublisher.set(endPose);

    Trajectory traj = TrajectoryGenerator.generateTrajectory(
      swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
      List.of(),
      endPose,
      trajectoryConfig);
      

    Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
      endPose,
      List.of(),
      pickUpPose,
      trajectoryConfig);

    // 4. Construct command to follow trajectory 
    // SwerveControllerCommand path1 = new SwerveControllerCommand(
    //     traj,
    //     swerveSubsystem::getPose, 
    //     DriveConstants.kDriveKinematics,
    //     xController,
    //     yController,
    //     thetaController,
    //     swerveSubsystem::setModuleStates,
    //     swerveSubsystem);

    // SwerveControllerCommand path2 = new SwerveControllerCommand(
    //     traj2,
    //     swerveSubsystem::getPose, 
    //     DriveConstants.kDriveKinematics,
    //     xController,
    //     yController,
    //     thetaController,
    //     swerveSubsystem::setModuleStates,
    //     swerveSubsystem);
    
    Command path1 = AutoBuilder.pathfindToPose(endPose, Constants.pathConstraints);
    Command path2 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
    
    return path1.andThen(
        new ParallelCommandGroup(
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.30),
            path2
            )
    );
  }
}
