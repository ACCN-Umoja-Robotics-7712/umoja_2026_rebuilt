package frc.robot.commands;

import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    //   AutoConstants.kMaxSpeedMetersPerSecond,
    //   AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //           .setKinematics(DriveConstants.kDriveKinematics);
    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, AutoConstants.kIThetaController, 0, AutoConstants.kThetaControllerConstraints);

    PathPlannerPath depot_trench_to_neutral;
    
    public enum AUTO {
        BLUE_TRENCH_LEFT_NEUTRAL, FAST_BLUE_TRENCH_LEFT_NEUTRAL, BLUE_CENTER_TOWER, BLUE_RIGHT_BUMP_OUTPOST, BLUE_RIGHT_TRENCH_OUTPOST, BLUE_TRENCH_RIGHT_NEUTRAL, FAST_BLUE_TRENCH_RIGHT_NEUTRAL,
        RED_TRENCH_LEFT_NEUTRAL, FAST_RED_TRENCH_LEFT_NEUTRAL, RED_CENTER_TOWER, RED_TRENCH_RIGHT_NEUTRAL, FAST_RED_TRENCH_RIGHT_NEUTRAL, RED_TRENCH_RIGHT_OUTPOST, RED_CENTER_TOWER_R,
        PRACTICE_FIELD, SIMPLE_AUTO, TUNE_AUTO, BLUE_LEFT_BUMP_DEPOT, BLUE_RIGHT_AUTO_FULL_2
    }
    private SendableChooser<AUTO> chooser;
    private SendableChooser<Command> ppChooser;
    private PathPlannerPath blueLeftTrenchPathForward, blueLeftTrenchPathBackward, blueRightTrenchPathForward, blueRightTrenchPathBackward, redLeftTrenchPathForward, redLeftTrenchPathBackward, redRightTrenchPathForward, redRightTrenchPathBackward;
    
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Auto end pose", Pose2d.struct).publish();
    // StructPublisher<PathPlannerPath> pathPublisher = NetworkTableInstance.getDefault().getStructTopic("path", PathPlannerPath.struct).publish();
    
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
        chooser.addOption("FAST Blue Left Trench to Neutral", AUTO.FAST_BLUE_TRENCH_LEFT_NEUTRAL);
        chooser.addOption("Blue Center to Tower", AUTO.BLUE_CENTER_TOWER);
        chooser.addOption("Blue Right Trench to Neutral", AUTO.BLUE_TRENCH_RIGHT_NEUTRAL);
        chooser.addOption("FAST Blue Right Trench to Neutral", AUTO.FAST_BLUE_TRENCH_RIGHT_NEUTRAL);
        chooser.addOption("Red Trench Left to Neutral", AUTO.RED_TRENCH_LEFT_NEUTRAL);
        chooser.addOption("FAST Red Trench Left to Neutral", AUTO.FAST_RED_TRENCH_LEFT_NEUTRAL);
        chooser.addOption("Red Trech to Neutral then Tower from Left", AUTO.RED_CENTER_TOWER);
        chooser.addOption("Red Tower to Neutral then Tower from Right", AUTO.RED_CENTER_TOWER_R);
        chooser.addOption("Red Right Trench to Neutral", AUTO.RED_TRENCH_RIGHT_NEUTRAL);
        chooser.addOption("FAST Red Right Trench to Neutral", AUTO.FAST_RED_TRENCH_RIGHT_NEUTRAL);
        chooser.addOption("Blue Right Outpost from Bump", AUTO.BLUE_RIGHT_BUMP_OUTPOST);
        chooser.addOption("Blue Right Outpost from Trench", AUTO.BLUE_RIGHT_TRENCH_OUTPOST);
        // chooser.addOption("Red Right Trench to Outpost", AUTO.BLUE_TRENCH_RIGHT_OUTPOST);
        chooser.addOption("Blue Left Bump to Neutral", AUTO.BLUE_LEFT_BUMP_DEPOT); // Go to neutral, intake, go to trench, shoot, go back to neutral, intake, go to outpost, shoot
        chooser.addOption("simple", AUTO.SIMPLE_AUTO);
        chooser.addOption("tune", AUTO.TUNE_AUTO);
        chooser.setDefaultOption("Default", null);
        
        // ppChooser = AutoBuilder.buildAutoChooser();
        // // Put the auto chooser on the dashboard
        SmartDashboard.putData("AUTOS", chooser);
        // SmartDashboard.putData("PP AUTOS", ppChooser);
        
        try {
            blueLeftTrenchPathForward = PathPlannerPath.fromPathFile("Blue Left Trench to Neutral Forward");
            blueLeftTrenchPathBackward = PathPlannerPath.fromPathFile("Blue Left Trench to Neutral Backward");
            blueRightTrenchPathForward = blueLeftTrenchPathBackward.mirrorPath();
            blueRightTrenchPathBackward = blueLeftTrenchPathBackward.mirrorPath();
            redLeftTrenchPathForward = blueLeftTrenchPathForward.flipPath();
            redLeftTrenchPathBackward = blueLeftTrenchPathBackward.flipPath();
            redRightTrenchPathForward = redLeftTrenchPathForward.mirrorPath();
            redRightTrenchPathBackward = redLeftTrenchPathBackward.mirrorPath();
        } catch (Exception e) {
            System.err.println("UNABLE TO FIND PATH");
        }

        // 1. Create trajectory settings
        // this.trajectoryConfig = new TrajectoryConfig(
        //         AutoConstants.kMaxSpeedMetersPerSecond,
        //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 .setKinematics(DriveConstants.kDriveKinematics);

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
            case FAST_BLUE_TRENCH_LEFT_NEUTRAL -> getFastBlueTrenchLeftNeutral();
            case BLUE_TRENCH_RIGHT_NEUTRAL -> getBlueTrenchRightNeutral();
            case FAST_BLUE_TRENCH_RIGHT_NEUTRAL -> getFastBlueTrenchRightNeutral();
            // case BLUE_CENTER_TOWER -> getBlueCenter();
            case RED_TRENCH_LEFT_NEUTRAL -> getRedTrenchLeftNeutral();
            case FAST_RED_TRENCH_LEFT_NEUTRAL -> getFastRedTrenchLeftNeutral();
            case RED_CENTER_TOWER -> getRedTowerFromLeft();
            case RED_CENTER_TOWER_R -> getRedTowerFromRight();
            case RED_TRENCH_RIGHT_NEUTRAL -> getRedTrenchRightNeutral();
            case FAST_RED_TRENCH_RIGHT_NEUTRAL -> getFastRedTrenchRightNeutral();
            case RED_TRENCH_RIGHT_OUTPOST -> getRedTrenchRightOutpost();
            case BLUE_RIGHT_BUMP_OUTPOST -> getBlueRightBumpOutpost();
            case BLUE_LEFT_BUMP_DEPOT -> getBlueBumpDepot();
            case SIMPLE_AUTO -> getSimpleAuto();
            case TUNE_AUTO -> getTuneAuto();
            default -> new InstantCommand();
        };
    }

// ------------------------------------------------------------------------ // ----------------------------- AUTOS ---------------------------------- //


    public Command getTuneAuto() {
        // drive forward/side 1 meter, turn 60 degrees
        // return getPathToPose(swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 1, 1, 60));
        
        PathPlannerPath path1;
        try {
            path1 = PathPlannerPath.fromPathFile("1").flipPath().mirrorPath();
            return AutoBuilder.followPath(path1).andThen(
                new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed)
            );
        } catch (Exception e) {
            System.out.println("PATH NOT FOUND, " + e);
            return new InstantCommand();
        }
        // PathPlannerPath path;
        // try {
        //     path = PathPlannerPath.fromPathFile("2");
        // } catch (Exception e) {
        //     return new InstantCommand();
        // }
        // Command path1 = getPathToPose(swerveSubsystem.offsetPoint(swerveSubsystem.getPose(), 0, 2));
        // RED_TRENCH_DEPOT_AUTO_RETURN
        // return Commands.sequence(
        //     getPathToPose(endPose),
        //     getPathToPose(pickUpPose),
        //     getPathToPose(returnPose),
        //     getPathToPose(trenchPose)
        // );
        // return path1;
    }

    public Command getRedTrenchRightNeutral() {
        Pose2d endPose = SHOOTING_POSES.RED_NEUTRAL_RIGHT;
        Pose2d pickUpPose = SHOOTING_POSES.RED_HALF_RIGHT;
        Pose2d returnPose = SHOOTING_POSES.RED_TRENCH_OUTPOST_AUTO_RETURN;
        Pose2d trenchPose = SHOOTING_POSES.RED_TRENCH_RIGHT;
        
        return Commands.parallel(
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed),
            // Zero Hood
            new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1).alongWith(
                // Lower Arm
                new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5)
            ).andThen(
                // Align Turret
                Commands.parallel(
                new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                )
            ),
            new WaitCommand(2)
                .andThen(
                    // SHOOT
                    Commands.parallel(
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ConditionalCommand(
                            // Run Indexer
                            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                            // Stop Indexer
                            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                            RobotContainer::isReadyToShoot)
                    ).withTimeout(0.9).andThen(Commands.parallel(
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5)
            .andThen(
                Commands.parallel(
                    new InstantCommand(() -> RobotContainer.shooterFlywheelSubsystem.stopFlywheel(), RobotContainer.shooterFlywheelSubsystem),
                    new InstantCommand(() -> RobotContainer.indexerSubsystem.stopIndexer(), RobotContainer.indexerSubsystem)
                )
            )
            .andThen(
                getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose))
            ).andThen(
                // Shoot again
                Commands.parallel(
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        // Run Indexer
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        // Stop Indexer
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot)
                ).withTimeout(0.9).andThen(Commands.parallel(
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts))
                )
            ).withTimeout(5)
            .andThen(
                Commands.parallel(
                    new InstantCommand(() -> RobotContainer.shooterFlywheelSubsystem.stopFlywheel(), RobotContainer.shooterFlywheelSubsystem),
                    new InstantCommand(() -> RobotContainer.indexerSubsystem.stopIndexer(), RobotContainer.indexerSubsystem)
                )
            )
            .andThen(
                getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose))
            )
        );
    }

    public Command getRedTrenchLeftNeutral() {
        Pose2d endPose = SHOOTING_POSES.RED_NEUTRAL_LEFT;
        Pose2d pickUpPose = SHOOTING_POSES.RED_HALF_LEFT;
        Pose2d returnPose = SHOOTING_POSES.RED_TRENCH_DEPOT_AUTO_RETURN;
        Pose2d trenchPose = SHOOTING_POSES.RED_TRENCH_LEFT;
 
        return Commands.parallel(
            // Zero hood and lower arm then aim turret
            new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1)
                .alongWith(
                    // Run Inake
                    Commands.parallel(
                        new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed)
                    ).withTimeout(0.5)
                ).andThen(
                    // Turret auto aim
                    Commands.parallel(
                        new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                        new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                    )
                ),
            // wait a bit then shoot
            new WaitCommand(2).andThen(
                Commands.parallel(
                    // Shoot
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5)
                .andThen(
                    Commands.deadline(
                        // Pick Up from CENTER and return
                        getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose)),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed)
                    )
                )
            .andThen(
                // shoot
                Commands.parallel(
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
            .andThen(
                Commands.parallel(
                    // Force shoot 
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                )
            )).withTimeout(5.5)
            .andThen(
                Commands.deadline(
                    // Drive back to center
                    getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose)),
                    new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed)
                )
            )
        );
    }
    
    public Command getBlueTrenchLeftNeutral() {
        return Commands.parallel(
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed),
            // Zero hood and lower arm then aim turret
            new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1)
                .alongWith(
                    // Run Inake
                    Commands.parallel(
                        new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5)
                    ).withTimeout(0.5)
                ).andThen(
                    // Turret auto aim
                    Commands.parallel(
                        new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                        new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                    )
                ),
            // wait a bit then shoot
            new WaitCommand(2).andThen(
                Commands.parallel(
                    // Shoot
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5)
            .andThen(
                    // Pick Up from CENTER and return
                    AutoBuilder.followPath(blueLeftTrenchPathBackward)
            )
            .andThen(
                // shoot
                Commands.parallel(
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot 
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            )
        );
    }

    public Command getFastBlueTrenchLeftNeutral() {
        return Commands.parallel(
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed),
            // Zero hood and lower arm then aim turret
            new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1)
                .alongWith(
                    // Run Intake
                    new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5)
                ).andThen(
                    // Turret auto aim
                    Commands.parallel(
                        new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                        new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                    )
                ),
            // Pick Up from CENTER and return
            AutoBuilder.followPath(blueLeftTrenchPathBackward)
            .andThen(
                // shoot
                Commands.parallel(
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot 
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5.5)
            .andThen(
                Commands.parallel(
                    new InstantCommand(() -> RobotContainer.shooterFlywheelSubsystem.stopFlywheel(), RobotContainer.shooterFlywheelSubsystem),
                    new InstantCommand(() -> RobotContainer.indexerSubsystem.stopIndexer(), RobotContainer.indexerSubsystem)
                )
            )
            .andThen(
                AutoBuilder.followPath(blueLeftTrenchPathBackward)
            )
        );
    }
    
    public Command getFastBlueTrenchRightNeutral() {
        return Commands.parallel(
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed),
            // Zero hood and lower arm then aim turret
            new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1)
                .alongWith(
                    // Run Intake
                    new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5)
                ).andThen(
                    // Turret auto aim
                    Commands.parallel(
                        new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                        new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                    )
                ),
            // Pick Up from CENTER and return
            AutoBuilder.followPath(blueRightTrenchPathBackward)
            .andThen(
                // shoot
                Commands.parallel(
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot 
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5.5)
            .andThen(
                Commands.parallel(
                    new InstantCommand(() -> RobotContainer.shooterFlywheelSubsystem.stopFlywheel(), RobotContainer.shooterFlywheelSubsystem),
                    new InstantCommand(() -> RobotContainer.indexerSubsystem.stopIndexer(), RobotContainer.indexerSubsystem)
                )
            )
            .andThen(
                AutoBuilder.followPath(blueRightTrenchPathBackward)
            )
        );
    }
    public Command getFastRedTrenchLeftNeutral() {
        return Commands.parallel(
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed),
            // Zero hood and lower arm then aim turret
            new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1)
                .alongWith(
                    // Run Intake
                    new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5)
                ).andThen(
                    // Turret auto aim
                    Commands.parallel(
                        new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                        new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                    )
                ),
            // Pick Up from CENTER and return
            AutoBuilder.followPath(redLeftTrenchPathBackward)
            .andThen(
                // shoot
                Commands.parallel(
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot 
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5.5)
            .andThen(
                Commands.parallel(
                    new InstantCommand(() -> RobotContainer.shooterFlywheelSubsystem.stopFlywheel(), RobotContainer.shooterFlywheelSubsystem),
                    new InstantCommand(() -> RobotContainer.indexerSubsystem.stopIndexer(), RobotContainer.indexerSubsystem)
                )
            )
            .andThen(
                AutoBuilder.followPath(redLeftTrenchPathBackward)
            )
        );
    }
    public Command getFastRedTrenchRightNeutral() {
        return Commands.parallel(
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed),
            // Zero hood and lower arm then aim turret
            Commands.sequence(
                new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1),
                // Lower arm
                new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5),
                Commands.parallel(
                    // Turret auto aim
                    Commands.parallel(
                        new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                        new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                    ),
                            // Pick Up from CENTER and return
                    AutoBuilder.followPath(redRightTrenchPathBackward)
                    .andThen(
                        // shoot
                        Commands.parallel(
                            new InstantCommand(() -> RobotContainer.swerveSubsystem.stopModules(), RobotContainer.swerveSubsystem),
                            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                            new ConditionalCommand(
                                new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                                new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                                RobotContainer::isReadyToShoot
                            ),
                            Commands.repeatingSequence(
                                new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.1).withTimeout(0.2),
                                new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem,  () -> 0.0).withTimeout(0.2)
                            )
                        ).withTimeout(0.9)
                        .andThen(
                            Commands.parallel(
                                // Force shoot 
                                new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                                new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                                Commands.repeatingSequence(
                                    new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.1).withTimeout(0.1),
                                    new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem,  () -> 0.0).withTimeout(0.1)
                                )
                            )
                        ).withTimeout(7)
                    )
                    .andThen(
                        Commands.parallel(
                            new InstantCommand(() -> RobotContainer.shooterFlywheelSubsystem.stopFlywheel(), RobotContainer.shooterFlywheelSubsystem),
                            new InstantCommand(() -> RobotContainer.indexerSubsystem.stopIndexer(), RobotContainer.indexerSubsystem)
                        )
                    )
                    .andThen(
                        AutoBuilder.followPath(redRightTrenchPathBackward)
                    )
                )
            )
        );
    }

    public Command getRedTowerFromLeft() {
        Pose2d endPose2d = SHOOTING_POSES.RED_TOWER_CENTER;
        Pose2d pickUpPose = SHOOTING_POSES.RED_NEUTRAL_LEFT;
        posePublisher.set(endPose2d);

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
       
        return Commands.parallel(
            // Zero hood and lower arm then aim turret
            new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1)
                .alongWith(
                    // Run Inake
                    Commands.parallel(
                        new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5)
                    ).withTimeout(0.5)
                ).andThen(
                    // Turret auto aim
                    Commands.parallel(
                        new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                        new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                    )
                ),
            // wait a bit then shoot
            new WaitCommand(2).andThen(
                Commands.parallel(
                    // Shoot
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5)
            .andThen(
                // Pick Up from CENTER and return
                AutoBuilder.followPath(blueLeftTrenchPathBackward)
            )
            .andThen(
                // shoot
                Commands.parallel(
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot 
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5.5)
            .andThen(
                Commands.parallel(
                    new InstantCommand(() -> RobotContainer.shooterFlywheelSubsystem.stopFlywheel(), RobotContainer.shooterFlywheelSubsystem),
                    new InstantCommand(() -> RobotContainer.indexerSubsystem.stopIndexer(), RobotContainer.indexerSubsystem)
                )
            )
            .andThen(
                getPathToPose(endPose).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(trenchPose))
            )
        );
}

    
    public Command getBlueRightBumpOutpost() {
        Pose2d endPose = SHOOTING_POSES.RED_OUTPOST_CENTER;
        
        return Commands.parallel(
                    // Zero hood and lower arm then aim turret
                    new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1)
                        .alongWith(
                            // Run Inake
                            Commands.parallel(
                                new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5)
                            ).withTimeout(0.5)
                        ).andThen(
                            // Turret auto aim
                            Commands.parallel(
                                new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                                new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                            )
                        ),
                    // wait a bit then shoot
                    new WaitCommand(2).andThen(
                        Commands.parallel(
                            // Shoot
                            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                            new ConditionalCommand(
                                new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                                new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                                RobotContainer::isReadyToShoot
                            )
                        ).withTimeout(0.9)
                        .andThen(
                            Commands.parallel(
                                // Force shoot
                                new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                                new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                            )
                        )
                    ).withTimeout(5)
                    .andThen(
                            // Pick Up from CENTER and return
                        Commands.sequence(
                            getPathToPose(endPose)
                        )
                    )
                    .andThen(
                        // shoot
                        Commands.parallel(
                            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                            new ConditionalCommand(
                                new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                                new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                                RobotContainer::isReadyToShoot
                            )
                        ).withTimeout(0.9)
                        .andThen(
                            Commands.parallel(
                                // Force shoot 
                                new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                                new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                            )
                        )
                    ).withTimeout(5.5)
                    .andThen(
                        Commands.parallel(
                            new InstantCommand(() -> RobotContainer.shooterFlywheelSubsystem.stopFlywheel(), RobotContainer.shooterFlywheelSubsystem),
                            new InstantCommand(() -> RobotContainer.indexerSubsystem.stopIndexer(), RobotContainer.indexerSubsystem)
                        )
                    )
                    .andThen(
                        getPathToPose(endPose)
                    )
                );
    }
    
    public Command getBlueBumpDepot() {
        Pose2d endPose = SHOOTING_POSES.BLUE_DEPOT_CENTER;

       
        return Commands.parallel(
            // Zero hood and lower arm then aim turret
            new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1)
                .alongWith(
                    // Run Inake
                    Commands.parallel(
                        new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5)
                    ).withTimeout(0.5)
                ).andThen(
                    // Turret auto aim
                    Commands.parallel(
                        new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                        new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                    )
                ),
            // wait a bit then shoot
            new WaitCommand(2).andThen(
                Commands.parallel(
                    // Shoot
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5)
            .andThen(
                    // Pick Up from CENTER and return
                Commands.sequence(
                    getPathToPose(endPose)
                )
            )
            .andThen(
                // shoot
                Commands.parallel(
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot 
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5.5)
            .andThen(
                Commands.parallel(
                    new InstantCommand(() -> RobotContainer.shooterFlywheelSubsystem.stopFlywheel(), RobotContainer.shooterFlywheelSubsystem),
                    new InstantCommand(() -> RobotContainer.indexerSubsystem.stopIndexer(), RobotContainer.indexerSubsystem)
                )
            )
            .andThen(
                getPathToPose(endPose)
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

        
        return Commands.parallel(
            // Zero hood and lower arm then aim turret
            new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem).withTimeout(1)
                .alongWith(
                    // Run Inake
                    Commands.parallel(
                        new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.5),
                        new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed)
                    ).withTimeout(0.5)
                ).andThen(
                    // Turret auto aim
                    Commands.parallel(
                        new ShooterTurretAngleCommand(RobotContainer.shooterTurretSubsystem, swerveSubsystem::getTurretToTargetAngle),
                        new ShooterHoodValueCommand(RobotContainer.shooterHoodSubsystem, swerveSubsystem::getTurretToTargetHoodValue)
                    )
                ),
            // wait a bit then shoot
            new WaitCommand(2).andThen(
                Commands.parallel(
                    // Shoot
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
                .andThen(
                    Commands.parallel(
                        // Force shoot
                        new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                    )
                )
            ).withTimeout(5)
            .andThen(
                Commands.deadline(
                    // Pick Up from CENTER and return
                    getPathToPose(endPose2d).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(finalPose)),
                    new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed)
                )
            )
            .andThen(
                // shoot
                Commands.parallel(
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ConditionalCommand(
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts),
                        new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0),
                        RobotContainer::isReadyToShoot
                    )
                ).withTimeout(0.9)
            .andThen(
                Commands.parallel(
                    // Force shoot 
                    new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
                    new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
                )
            )).withTimeout(5.5)
            .andThen(
                Commands.deadline(
                    // Drive back to center
                    getPathToPose(endPose2d).andThen(getPathToPose(pickUpPose)).andThen(getPathToPose(returnPose)).andThen(getPathToPose(finalPose)),
                    new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed)
                )
            )
        );
    }
    
    public Command getSimpleAuto() {
        Command runIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts);
        Command stopIndexer = new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> 0.0);
        Command zeroHood = new ZeroHoodCommand(RobotContainer.shooterHoodSubsystem);

        Command lowerArm = Commands.parallel(
            new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem, () -> 0.19).withTimeout(0.6),
            new ManualIntakeRoller(RobotContainer.intakeRollerSubsystem, () -> Constants.IntakeConstants.intakeSpeed)
        ).withTimeout(0.5);
        Command shoot = Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ConditionalCommand(runIndexer, stopIndexer, RobotContainer::isReadyToShoot)
        ).withTimeout(0.9).andThen(Commands.parallel(
            new ShooterFlywheelVelocityCommand(RobotContainer.shooterFlywheelSubsystem, swerveSubsystem::getTurretToTargetRPMValue),
            new ManualIndexerCommand(RobotContainer.indexerSubsystem, () -> Constants.IndexerConstants.indexVolts)
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
    // posePublisher.set(endPose);

    // Trajectory traj = TrajectoryGenerator.generateTrajectory(
    //   swerveSubsystem.getPose(),
    //   List.of(),
    //   endPose,
    //   trajectoryConfig);
      

    // // 4. Construct command to follow trajectory 
    // SwerveControllerCommand trajectoryPath = new SwerveControllerCommand(
    //     traj,
    //     swerveSubsystem::getPose, 
    //     DriveConstants.kDriveKinematics,
    //     xController,
    //     yController,
    //     thetaController,
    //     swerveSubsystem::setModuleStates,
    //     swerveSubsystem);

    Command pathPlannerPath = AutoBuilder.pathfindToPose(endPose, Constants.pathConstraints);
    return pathPlannerPath;
  }
  
  public Command moveRobot(double forward, double side, double rotation, double duration) {
    return new SwerveJoystick(
        swerveSubsystem, 
        () -> forward, 
        () -> side, 
        () -> rotation
    ).withTimeout(duration);
  }
}
