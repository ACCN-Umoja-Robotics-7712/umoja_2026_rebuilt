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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SHOOTING_POSES;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.ManualCommands.ManualClimbCommand;
import frc.robot.commands.ManualCommands.ManualIndexerCommand;
import frc.robot.commands.ManualCommands.ManualIntakeArmCommand;
import frc.robot.commands.ManualCommands.ManualIntakeRoller;
import frc.robot.commands.ManualCommands.ManualShooterFlywheelCommand;
import frc.robot.commands.ZeroCommands.ZeroHoodCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// ---------------------------------------------------------- AUTO COMMANDS -------------------------------------------------- //

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
        BLUE_TRENCH_LEFT_NEUTRAL, BLUE_CENTER_TOWER, BLUE_TRENCH_RIGHT_OUTPOST, BLUE_TRENCH_RIGHT_NEUTRAL,
        RED_TRENCH_LEFT_NEUTRAL, RED_CENTER_TOWER, RED_TRENCH_RIGHT_NEUTRAL, RED_TRENCH_RIGHT_OUTPOST, RED_CENTER_TOWER_R,
        PRACTICE_FIELD, SIMPLE_AUTO, TUNE_AUTO, BLUE_RIGHT_AUTO_FULL_1, BLUE_RIGHT_AUTO_FULL_2
    }
    private SendableChooser<AUTO> chooser;
    private SendableChooser<Command> ppChooser;
    
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Auto end pose", Pose2d.struct).publish();
    
    public Autos(){ // ALWAYS ENSURE ARM LOWERS THE MOMENT GAME STARTS PARALLEL TO SHOOTING (~2 balls must be shot by the time the arm is down)

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
        chooser.addOption("Red Trech to Neutral then Tower from Left", AUTO.RED_CENTER_TOWER);
        chooser.addOption("Red Tower to Neutral then Tower from Right", AUTO.RED_CENTER_TOWER_R);
        chooser.addOption("Red Right Trench to Neutral", AUTO.RED_TRENCH_RIGHT_NEUTRAL);
        chooser.addOption("Blue Right Trench to Outpost", AUTO.BLUE_TRENCH_RIGHT_OUTPOST);
        // chooser.addOption("Red Right Trench to Outpost", AUTO.BLUE_TRENCH_RIGHT_OUTPOST);
        chooser.addOption("Blue Right Auto Full 1", AUTO.BLUE_RIGHT_AUTO_FULL_1); // Go to neutral, intake, go to trench, shoot, go back to neutral, intake, go to outpost, shoot
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

// -------------------------------------------------------- AUTO SELECTOR --------------------------------------- ----------------- //

        return switch (auto) {
            case BLUE_TRENCH_LEFT_NEUTRAL -> getBlueTrenchLeftNeutral();
            // case BLUE_CENTER_TOWER -> getBlueCenter();
            case BLUE_TRENCH_RIGHT_NEUTRAL -> getBlueTrenchRightNeutral();
            case RED_TRENCH_LEFT_NEUTRAL -> getRedTrenchLeftNeutral();
            case RED_CENTER_TOWER -> getRedTowerFromLeft();
            case RED_CENTER_TOWER_R -> getRedTowerFromRight();
            case RED_TRENCH_RIGHT_NEUTRAL -> getRedTrenchRightNeutral();
            case RED_TRENCH_RIGHT_OUTPOST -> getRedTrenchRightOutpost();
            case BLUE_TRENCH_RIGHT_OUTPOST -> getBlueTrenchRightOutpost();
            case BLUE_RIGHT_AUTO_FULL_1 -> getBlueRightFull1();
            case SIMPLE_AUTO -> getSimpleAuto();
            case TUNE_AUTO -> getTuneAuto();
            default -> new InstantCommand();
        };
    }

// ------------------------------------------------------------------------ // ----------------------------- AUTOS ---------------------------------- //


    public Command getTuneAuto() {
        // drive forward/side 1 meter, turn 60 degrees
        return getPathToPose(swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 1, 1, 60));
    }

    public Command getRedTrenchRightNeutral() {
        Pose2d endPose = SHOOTING_POSES.RED_NEUTRAL_RIGHT;
        Pose2d pickUpPose = SHOOTING_POSES.RED_HALF_RIGHT;
        Pose2d returnPose = SHOOTING_POSES.RED_TRENCH_OUTPOST_AUTO_RETURN;
        Pose2d trenchPose = SHOOTING_POSES.RED_TRENCH_RIGHT;
        
        Command runIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0);
        Command stopIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0);
        Command runIndexer1 = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0);
        Command stopIndexer2 = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0);
        Command zeroHood = new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem);

        Command lowerArm = Commands.parallel(
            new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5),
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
        ).withTimeout(0.5);
        Command shoot = Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ConditionalCommand(runIndexer, stopIndexer, RobotContainer::isReadyToShoot)
        ).withTimeout(0.75).andThen(Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0)
            )
        );
        Command shoot2 = Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ConditionalCommand(runIndexer1, stopIndexer2, RobotContainer::isReadyToShoot)
        ).withTimeout(0.75).andThen(Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0)
            )
        );
        
        Command alignTurret = 
        Commands.parallel(
            new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
            new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
        );
        return Commands.parallel(
            zeroHood.withTimeout(1).alongWith(lowerArm).andThen(alignTurret),
            new WaitCommand(2)
                .andThen(shoot).withTimeout(5)
                .andThen(
                    Commands.deadline(
                        getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose)),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
                    )
                ).andThen(shoot2).withTimeout(5)
                .andThen(
                    Commands.deadline(
                        getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose)),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
                    )
                )
        );
    }

    public Command getRedTrenchLeftNeutral() {
        Pose2d endPose = SHOOTING_POSES.RED_NEUTRAL_LEFT;
        Pose2d pickUpPose = SHOOTING_POSES.RED_HALF_LEFT;
        Pose2d returnPose = SHOOTING_POSES.RED_TRENCH_DEPOT_AUTO_RETURN;
        Pose2d trenchPose = SHOOTING_POSES.RED_TRENCH_LEFT;
       
        Command runIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0);
        Command stopIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0);
        Command runIndexer1 = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0);
        Command stopIndexer2 = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0);
        Command zeroHood = new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem);

        Command lowerArm = Commands.parallel(
            new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5),
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
        ).withTimeout(0.5);
        Command shoot = Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ConditionalCommand(runIndexer, stopIndexer, RobotContainer::isReadyToShoot)
        ).withTimeout(0.75).andThen(Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0)
            )
        );
        Command shoot2 = Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ConditionalCommand(runIndexer1, stopIndexer2, RobotContainer::isReadyToShoot)
        ).withTimeout(0.75).andThen(Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0)
            )
        );
        
        Command alignTurret = 
        Commands.parallel(
            new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
            new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
        );
        return Commands.parallel(
            zeroHood.withTimeout(1).alongWith(lowerArm).andThen(alignTurret),
            new WaitCommand(2)
                .andThen(shoot).withTimeout(5)
                .andThen(
                    Commands.deadline(
                        getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose)),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
                    )
                ).andThen(shoot2).withTimeout(5)
                .andThen(
                    Commands.deadline(
                        getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose)),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
                    )
                )
        );
    }
    
    public Command getBlueTrenchLeftNeutral() {
        Pose2d endPose = SHOOTING_POSES.BLUE_NEUTRAL_LEFT;
        Pose2d pickUpPose = SHOOTING_POSES.BLUE_HALF_LEFT;
        Pose2d returnPose = SHOOTING_POSES.BLUE_TRENCH_DEPOT_AUTO_RETURN;
        Pose2d trenchPose = SHOOTING_POSES.BLUE_TRENCH_LEFT;
       
        Command runIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0);
        Command stopIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0);
        Command runIndexer1 = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0);
        Command stopIndexer2 = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0);
        Command zeroHood = new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem);

        Command lowerArm = Commands.parallel(
            new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5),
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
        ).withTimeout(0.5);
        Command shoot = Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ConditionalCommand(runIndexer, stopIndexer, RobotContainer::isReadyToShoot)
        ).withTimeout(0.75).andThen(Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0)
            )
        );
        Command shoot2 = Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ConditionalCommand(runIndexer1, stopIndexer2, RobotContainer::isReadyToShoot)
        ).withTimeout(0.75).andThen(Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0)
            )
        );
        
        Command alignTurret = 
        Commands.parallel(
            new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
            new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
        );
        return Commands.parallel(
            zeroHood.withTimeout(1).alongWith(lowerArm).andThen(alignTurret),
            new WaitCommand(2)
                .andThen(shoot).withTimeout(5)
                .andThen(
                    Commands.deadline(
                        getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose)),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
                    )
                ).andThen(shoot2).withTimeout(5)
                .andThen(
                    Commands.deadline(
                        getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose)),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
                    )
                )
        );
    }

    public Command getRedTowerFromLeft() {
        Pose2d endPose2d = SHOOTING_POSES.RED_TOWER_CENTER;
        Pose2d pickUpPose = SHOOTING_POSES.RED_NEUTRAL_LEFT;
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
        Pose2d endPose = SHOOTING_POSES.BLUE_NEUTRAL_RIGHT;
        Pose2d pickUpPose = SHOOTING_POSES.BLUE_HALF_RIGHT;
        Pose2d returnPose = SHOOTING_POSES.BLUE_TRENCH_OUTPOST_AUTO_RETURN;
        Pose2d trenchPose = SHOOTING_POSES.BLUE_TRENCH_RIGHT;
        
        Command runIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0);
        Command stopIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0);
        Command runIndexer1 = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0);
        Command stopIndexer2 = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0);
        Command zeroHood = new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem);

        Command lowerArm = Commands.parallel(
            new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5),
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
        ).withTimeout(0.5);
        Command shoot = Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ConditionalCommand(runIndexer, stopIndexer, RobotContainer::isReadyToShoot)
        ).withTimeout(0.75).andThen(Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0)
            )
        );
        Command shoot2 = Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ConditionalCommand(runIndexer1, stopIndexer2, RobotContainer::isReadyToShoot)
        ).withTimeout(0.75).andThen(Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0)
            )
        );
        
        Command alignTurret = 
        Commands.parallel(
            new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
            new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
        );
        return Commands.parallel(
            zeroHood.withTimeout(1).alongWith(lowerArm).andThen(alignTurret),
            new WaitCommand(2)
                .andThen(shoot).withTimeout(5)
                .andThen(
                    Commands.deadline(
                        getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose)),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
                    )
                ).andThen(shoot2).withTimeout(5)
                .andThen(
                    Commands.deadline(
                        getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose)),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
                    )
                )
        );
    }

    
    public Command getBlueTrenchRightOutpost() {
        Pose2d endPose2d = SHOOTING_POSES.BLUE_NEUTRAL_RIGHT;
        Pose2d pickUpPose = SHOOTING_POSES.BLUE_HALF_RIGHT;
        Pose2d returnPose = SHOOTING_POSES.BLUE_TRENCH_OUTPOST_AUTO_RETURN;
        Pose2d finalPose = SHOOTING_POSES.BLUE_TRENCH_RIGHT;
        posePublisher.set(endPose2d);

        // Trajectory traj = TrajectoryGenerator.generateTrajectory(
        // swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
        // List.of(),
        // endPose2d,
        // trajectoryConfig);
        

        // Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        // endPose2d,
        // List.of(),
        // pickUpPose,
        // trajectoryConfig);

        // Trajectory traj3 = TrajectoryGenerator.generateTrajectory(
        // pickUpPose,
        // List.of(),
        // returnPose,
        // trajectoryConfig);

        // Trajectory traj4 = TrajectoryGenerator.generateTrajectory(
        // returnPose,
        // List.of(),
        // finalPose,
        // trajectoryConfig);

        Command path1 = AutoBuilder.pathfindToPose(endPose2d, Constants.pathConstraints);
        Command path2 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
        Command path3 = AutoBuilder.pathfindToPose(returnPose, Constants.pathConstraints);
        Command path4 = AutoBuilder.pathfindToPose(finalPose, Constants.pathConstraints);
        
        return new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.0),
                    new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.20)
                ),
                path1.andThen(path2.andThen(path3.andThen(path4)))
            ).andThen(
                new ParallelCommandGroup(
                    new AlignRobotBackWithHubFieldCommand(swerveSubsystem, () -> 0.0, () -> 0.0),
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, () -> 0.99)
                )
            );
    }
    
    public Command getBlueRightFull1() {
        Pose2d endPose2d = SHOOTING_POSES.BLUE_NEUTRAL_RIGHT;
        Pose2d pickUpPose = SHOOTING_POSES.BLUE_HALF_RIGHT;
        Pose2d returnPose = SHOOTING_POSES.BLUE_TRENCH_OUTPOST_AUTO_RETURN;
        Pose2d finalPose = SHOOTING_POSES.BLUE_OUTPOST_CENTER;
        posePublisher.set(endPose2d);

        // Trajectory traj = TrajectoryGenerator.generateTrajectory(
        // swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
        // List.of(),
        // endPose2d,
        // trajectoryConfig);
        

        // Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        // endPose2d,
        // List.of(),
        // pickUpPose,
        // trajectoryConfig);

        // Trajectory traj3 = TrajectoryGenerator.generateTrajectory(
        // pickUpPose,
        // List.of(),
        // returnPose,
        // trajectoryConfig);

        // Trajectory traj4 = TrajectoryGenerator.generateTrajectory(
        // returnPose,
        // List.of(),
        // finalPose,
        // trajectoryConfig);

        Command path1 = AutoBuilder.pathfindToPose(endPose2d, Constants.pathConstraints);
        Command path2 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
        Command path3 = AutoBuilder.pathfindToPose(returnPose, Constants.pathConstraints);
        Command path4 = AutoBuilder.pathfindToPose(finalPose, Constants.pathConstraints);
        
        return new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.0),
                    new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.20)
                ),
                path1.andThen(path2.andThen(path3.andThen(path1)))
            ).andThen(
                new ParallelCommandGroup(
                    path4,
                    new AlignRobotBackWithHubFieldCommand(swerveSubsystem, () -> 0.0, () -> 0.0),
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, () -> 0.99)
                )
            );
    }


// ------------------------------------------------------------------------------ // ---------- RED AUTOS ----------- // ------------------------------------------------------------------------------ //
    public Command getRedTowerFromRight() {
        Pose2d endPose2d = SHOOTING_POSES.RED_NEUTRAL_RIGHT;
        Pose2d pickupPose2d = SHOOTING_POSES.RED_HALF_RIGHT;
        Pose2d returnPose2d = SHOOTING_POSES.RED_TRENCH_OUTPOST_AUTO_RETURN;
        Pose2d climbPose2d = SHOOTING_POSES.RED_TOWER_CENTER; // Change to the right position for us to actually climb

        posePublisher.set(endPose2d);

        Command path1 = AutoBuilder.pathfindToPose(endPose2d, Constants.pathConstraints, 0);
        Command path2 = AutoBuilder.pathfindToPose(pickupPose2d, Constants.pathConstraints, 0);
        Command path3 = AutoBuilder.pathfindToPose(returnPose2d, Constants.pathConstraints, 0);
        Command path4 = AutoBuilder.pathfindToPose(climbPose2d, Constants.pathConstraints, 0);
        
        return new ParallelRaceGroup(
            new ParallelCommandGroup(
                new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.2),
                new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.99)
                ),
                path1.andThen(path2)
        ).andThen(
            path3,
            new AlignRobotBackWithHubFieldCommand(swerveSubsystem, () -> 0.0, () -> 0.0),
            path1,
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, () -> 0.99)
        ).andThen(
            path4,
            new ManualClimbCommand(RobotContainer.climbSubsystem, () -> 0.3),
            new WaitCommand(0.3),
            new ManualClimbCommand(RobotContainer.climbSubsystem, () -> -1.0)
        );
    }
    public Command getRedTrenchRightOutpost() {
        Pose2d endPose2d = SHOOTING_POSES.RED_NEUTRAL_RIGHT;
        Pose2d pickUpPose = SHOOTING_POSES.RED_HALF_RIGHT;
        Pose2d returnPose = SHOOTING_POSES.RED_TRENCH_OUTPOST_AUTO_RETURN;
        Pose2d finalPose = SHOOTING_POSES.RED_OUTPOST_CENTER;
        posePublisher.set(endPose2d);

        // Trajectory traj = TrajectoryGenerator.generateTrajectory(
        // swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
        // List.of(),
        // endPose2d,
        // trajectoryConfig);
        

        // Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        // endPose2d,
        // List.of(),
        // pickUpPose,
        // trajectoryConfig);

        // Trajectory traj3 = TrajectoryGenerator.generateTrajectory(
        // pickUpPose,
        // List.of(),
        // returnPose,
        // trajectoryConfig);

        // Trajectory traj4 = TrajectoryGenerator.generateTrajectory(
        // returnPose,
        // List.of(),
        // finalPose,
        // trajectoryConfig);

        Command path1 = AutoBuilder.pathfindToPose(endPose2d, Constants.pathConstraints);
        Command path2 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
        Command path3 = AutoBuilder.pathfindToPose(returnPose, Constants.pathConstraints);
        Command path4 = AutoBuilder.pathfindToPose(finalPose, Constants.pathConstraints);
        
        return new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.0),
                    new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.20)
                ),
                path1.andThen(path2.andThen(path3.andThen(path4)))
            ).andThen(
                new ParallelCommandGroup(
                    new AlignRobotBackWithHubFieldCommand(swerveSubsystem, () -> 0.0, () -> 0.0),
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, () -> 0.99)
                )
            );
    }

    public Command getRedTrenchRight() {
        Pose2d endPose2d = SHOOTING_POSES.RED_NEUTRAL_RIGHT;
        Pose2d pickUpPose = SHOOTING_POSES.RED_HALF_RIGHT;
        Pose2d returnPose = SHOOTING_POSES.RED_TRENCH_OUTPOST_AUTO_RETURN;
        Pose2d finalPose = SHOOTING_POSES.RED_TRENCH_RIGHT;
        posePublisher.set(endPose2d);

        // Trajectory traj = TrajectoryGenerator.generateTrajectory(
        // swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
        // List.of(),
        // endPose2d,
        // trajectoryConfig);
        

        // Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        // endPose2d,
        // List.of(),
        // pickUpPose,
        // trajectoryConfig);

        // Trajectory traj3 = TrajectoryGenerator.generateTrajectory(
        // pickUpPose,
        // List.of(),
        // returnPose,
        // trajectoryConfig);

        // Trajectory traj4 = TrajectoryGenerator.generateTrajectory(
        // returnPose,
        // List.of(),
        // finalPose,
        // trajectoryConfig);

        Command path1 = AutoBuilder.pathfindToPose(endPose2d, Constants.pathConstraints);
        Command path2 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
        Command path3 = AutoBuilder.pathfindToPose(returnPose, Constants.pathConstraints);
        Command path4 = AutoBuilder.pathfindToPose(finalPose, Constants.pathConstraints);
        
        return new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.0),
                    new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.20)
                ),
                path1.andThen(path2.andThen(path3.andThen(path4)))
            ).andThen(
                new ParallelCommandGroup(
                    new AlignRobotBackWithHubFieldCommand(swerveSubsystem, () -> 0.0, () -> 0.0),
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, () -> 0.99)
                )
            );
    }
    
    public Command getRedRightFull1() {
        Pose2d endPose2d = SHOOTING_POSES.RED_NEUTRAL_RIGHT;
        Pose2d pickUpPose = SHOOTING_POSES.RED_HALF_RIGHT;
        Pose2d returnPose = SHOOTING_POSES.RED_TRENCH_OUTPOST_AUTO_RETURN;
        Pose2d finalPose = SHOOTING_POSES.RED_OUTPOST_CENTER;
        posePublisher.set(endPose2d);

        // Trajectory traj = TrajectoryGenerator.generateTrajectory(
        // swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 0, 0),
        // List.of(),
        // endPose2d,
        // trajectoryConfig);
        

        // Trajectory traj2 = TrajectoryGenerator.generateTrajectory(
        // endPose2d,
        // List.of(),
        // pickUpPose,
        // trajectoryConfig);

        // Trajectory traj3 = TrajectoryGenerator.generateTrajectory(
        // pickUpPose,
        // List.of(),
        // returnPose,
        // trajectoryConfig);

        // Trajectory traj4 = TrajectoryGenerator.generateTrajectory(
        // returnPose,
        // List.of(),
        // finalPose,
        // trajectoryConfig);

        Command path1 = AutoBuilder.pathfindToPose(endPose2d, Constants.pathConstraints);
        Command path2 = AutoBuilder.pathfindToPose(pickUpPose, Constants.pathConstraints);
        Command path3 = AutoBuilder.pathfindToPose(returnPose, Constants.pathConstraints);
        Command path4 = AutoBuilder.pathfindToPose(finalPose, Constants.pathConstraints);
        
        return new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.0),
                    new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.20)
                ),
                path1.andThen(path2.andThen(path3.andThen(path1)))
            ).andThen(
                new ParallelCommandGroup(
                    path4,
                    new AlignRobotBackWithHubFieldCommand(swerveSubsystem, () -> 0.0, () -> 0.0),
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, () -> 0.99)
                )
            );
    }



    public Command getSimpleAuto() {
        Command runIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0);
        Command stopIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0);
        Command zeroHood = new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem);

        Command lowerArm = Commands.parallel(
            new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.3),
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> 0.31)
        ).withTimeout(0.5);
        Command shoot = Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ConditionalCommand(runIndexer, stopIndexer, RobotContainer::isReadyToShoot)
        ).withTimeout(0.75).andThen(Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 7.0)
            )
        );
        
        Command alignTurret = 
        Commands.parallel(
            new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
            new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
        );
        return Commands.parallel(
            zeroHood.withTimeout(1).alongWith(lowerArm).andThen(alignTurret),
            new WaitCommand(2)
                .andThen(shoot)
        );
    }

  public Command getPathToPose(Pose2d endPose) {
    posePublisher.set(endPose);
    Trajectory traj = TrajectoryGenerator.generateTrajectory(
      swerveSubsystem.getPose(),
      List.of(),
      endPose,
      trajectoryConfig);
      

    // 4. Construct command to follow trajectory 
    SwerveControllerCommand trajectoryPath = new SwerveControllerCommand(
        traj,
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    Command pathPlannerPath = AutoBuilder.pathfindToPose(endPose, Constants.pathConstraints);
    return trajectoryPath;
  }
}
