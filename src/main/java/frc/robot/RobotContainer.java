// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterStates;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterStates;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.IntakeArmStates;
import frc.robot.Constants.USB;
import frc.robot.commands.AlignWithTrench;
import frc.robot.commands.ManualShooterFlywheelCommand;
import frc.robot.commands.ManualCommands.ManualIntakeArmCommand;
import frc.robot.commands.ManualCommands.ManualIntakeRoller;
import frc.robot.commands.ManualCommands.ManualShooterHoodCommand;
import frc.robot.commands.ManualCommands.ManualTurretCommand;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  public final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final static IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
  public final static IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();
  public final static ShooterFlywheelSubsystem shooterFlywheelSubsystem = new ShooterFlywheelSubsystem();
  // public final static ShooterHoodSubsystem shooterHoodSubsystem = new ShooterHoodSubsystem();
  public final static IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  public final static ShooterTurretSubsystem ShooterTurretSubsystem = new ShooterTurretSubsystem();




  public final static CommandXboxController driverController = new CommandXboxController(USB.DRIVER_CONTROLLER);
  public final static CommandXboxController operatorController = new CommandXboxController(USB.OPERATOR_CONTROLLER);

  public static double wantedAngle = -1;
  public static int shouldAutoFixDrift = 0; // 1 = auto drift, 0 = none
  public static int gameState = GameConstants.Robot;
  public static Trajectory currentTrajectory = null;
  public static Pose2d goalPose = null;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    // Button Commands
    RobotContainer.driverController.y().whileTrue(
      new AlignWithTrench(
        RobotContainer.swerveSubsystem,
        () -> RobotContainer.driverController.getLeftY(),
        () -> RobotContainer.driverController.getLeftX(),
        0
      )
    );
    RobotContainer.driverController.a().whileTrue(
      new AlignWithTrench(
        RobotContainer.swerveSubsystem,
        () -> RobotContainer.driverController.getLeftY(),
        () -> RobotContainer.driverController.getLeftX(),
        180
      )
    );

    //Manual Commands (Just for Now)
    // Intake Roller
    RobotContainer.operatorController.leftBumper().whileTrue(
      new ManualIntakeRoller(intakeRollerSubsystem,
        () -> operatorController.getRightY() * 0.65
      )
    );

    //Flywheel Motor
    // RobotContainer.operatorController.rightStick().whileTrue(
    //   new ManualShooterFlywheelCommand(shooterFlywheelSubsystem,
    //     () -> operatorController.getRightY() * 0.5
    //   )
    // );

    // Hood Motor
    // RobotContainer.operatorController.rightStick().whileTrue(
    //   new ManualShooterHoodCommand(shooterHoodSubsystem,
    //     () -> operatorController.getRightY() * 0.5
    //   )
    // ); 

    // Turret Motor
    // RobotContainer.operatorController.rightStick().whileTrue(
    //   new ManualTurretCommand(ShooterTurretSubsystem,
    //     () -> operatorController.getRightY()
    //   )
    // );

    // Indexer Motor
    // RobotContainer.operatorController.rightStick().whileTrue(
    //   new ManualIndexerCommand(IndexerSubsystem,
    //     () -> operatorController.getRightY() * 0.5
    //   )
    // );

    // Intake Arm Motor
    RobotContainer.operatorController.rightBumper().whileTrue(
      new ManualIntakeArmCommand(intakeArmSubsystem,
        () -> operatorController.getRightY() * 0.6
      )
    );
  }

  public static double diffFromWantedAngle(double wantedAngle) {
    if (wantedAngle < 0) {
      wantedAngle += 360;
    }
    // fix drift
    // if drifting
    if (wantedAngle != swerveSubsystem.getHeading()) {
      // > 180 we want to go towards 0 but from negative to account for closest angle
      double currentAngle = swerveSubsystem.getHeading();
      double diff = Math.abs(currentAngle - wantedAngle);
      if (diff > 180) {
        diff = 360 - diff;
        // since diff is > 180, it passes the 360-0 range
        // if we have 359 and want 1, we go positive, so no change
        // if we have 1 and want 359, we go negative, so multiple -1
        if (wantedAngle > 180) {
          diff = diff * -1;
        }
      } else {
        // if going from 5 to 10, do nothing
        // if going from 10 to 5, we go negative so multiple -1
        if (currentAngle > wantedAngle) {
          diff = diff * -1;
        }
      }
      return -diff;
    } else {
      return 0;
    }
  }
}
