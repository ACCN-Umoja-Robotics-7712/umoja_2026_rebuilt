// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterTurretSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.USB;
import frc.robot.commands.AlignWithTrench;
import frc.robot.commands.ShooterFlywheelCommand;
import frc.robot.commands.ManualCommands.ManualClimbCommand;
import frc.robot.commands.ManualCommands.ManualIndexerCommand;
import frc.robot.commands.ManualCommands.ManualIntakeArmCommand;
import frc.robot.commands.ManualCommands.ManualIntakeRoller;
import frc.robot.commands.ManualCommands.ManualShooterFlywheelCommand;
import frc.robot.commands.ManualCommands.ManualShooterHoodCommand;
import frc.robot.commands.ManualCommands.ManualTurretCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
  public final static ShooterHoodSubsystem shooterHoodSubsystem = new ShooterHoodSubsystem();
  public final static IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  public final static ShooterTurretSubsystem ShooterTurretSubsystem = new ShooterTurretSubsystem();
  public final static ClimbSubsystem climbSubsystem = new ClimbSubsystem();




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
    
    // Align with trench
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
    RobotContainer.driverController.leftBumper().whileTrue(
      new ManualIntakeRoller(intakeRollerSubsystem,
        () -> 0.30
      )
    );

    //Flywheel Motor
    RobotContainer.operatorController.rightBumper().whileTrue(
      new ShooterFlywheelCommand(shooterFlywheelSubsystem,
        () -> -0.2
      )
    );

    // Hood Motor
    // RobotContainer.operatorController.leftBumper().whileTrue(
    //   new ManualShooterHoodCommand(shooterHoodSubsystem,
    //     () -> operatorController.getLeftY() * 0.1
    //   )
    // ); 

    // Turret Motor
    // RobotContainer.operatorController.leftBumper().whileTrue(
    //   new ManualTurretCommand(ShooterTurretSubsystem,
    //     () -> operatorController.getLeftX() * 0.3
    //   )
    // );

    // Indexer Motor
    RobotContainer.operatorController.y().whileTrue(
      new ManualIndexerCommand(indexerSubsystem,
        () -> 0.5
      )
    );

    //Intake Arm Motor
    RobotContainer.operatorController.rightBumper().whileTrue(
      new ManualIntakeArmCommand(RobotContainer.intakeArmSubsystem,
        () -> operatorController.getRightY() * 0.5
      )
    );

  //  // Climber
    // RobotContainer.operatorController.b().whileTrue(
    //   new ManualClimbCommand(climbSubsystem,
    //     () -> 0.30
    //   )
    // );
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
