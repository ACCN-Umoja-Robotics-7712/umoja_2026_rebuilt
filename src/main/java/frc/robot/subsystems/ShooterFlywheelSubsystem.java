// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.google.flatbuffers.Constants;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig.Presets;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterStates;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DriverStation;

public class ShooterFlywheelSubsystem extends SubsystemBase {
    private final SparkFlex flywheelMotorLeader;
    private final SparkFlex flywheelMotorFollower;
    private SparkFlex kickerMotor;

    // private final PIDController flywheelPidController;
    // private final SimpleMotorFeedforward flywheelFFController;
    private final SparkClosedLoopController flywheelController, kickerController;
    // private final PIDController kickerPidController;
    // private final SimpleMotorFeedforward kickerFFController;

    private double state = ShooterStates.NONE;
    private double wantedShooterRPM = 0;
    private double wantedKickerRPM = 0;

    // NetworkTables
    private NetworkTable shooterTable;
    private NetworkTable telemetryTable;
    private NetworkTable pidTable;
    private NetworkTable flywheelPIDTable;
    private NetworkTable kickerPIDTable;
    private NetworkTable controlTable;

    // Telemetry publishers
    private DoublePublisher flywheelVelocityPub;
    private DoublePublisher flywheelTemperaturePub;
    private DoublePublisher kickerVelocityPub;
    private DoublePublisher wantedKickerVelocityPub;

    // Flywheel PID entries
    private DoubleEntry flywheelKPEntry;
    private DoubleEntry flywheelKIEntry;
    private DoubleEntry flywheelKDEntry;
    private DoubleEntry flywheelKVEntry;

    // Kicker PID entries
    private DoubleEntry kickerKPEntry;
    private DoubleEntry kickerKIEntry;
    private DoubleEntry kickerKDEntry;
    private DoubleEntry kickerKVEntry;

    // Control entries
    private DoubleEntry customFlywheelVelocityEntry;

    // Last applied PID values
    private double lastFlywheelKP = Double.NaN;
    private double lastFlywheelKI = Double.NaN;
    private double lastFlywheelKD = Double.NaN;
    private double lastFlywheelKV = Double.NaN;

    private double lastKickerKP = Double.NaN;
    private double lastKickerKI = Double.NaN;
    private double lastKickerKD = Double.NaN;
    private double lastKickerKV = Double.NaN;

    public ShooterFlywheelSubsystem() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        shooterTable = nt.getTable("Shooter");

        telemetryTable = shooterTable.getSubTable("Telemetry");
        pidTable = shooterTable.getSubTable("PID");
        flywheelPIDTable = pidTable.getSubTable("Flywheel");
        kickerPIDTable = pidTable.getSubTable("Kicker");
        controlTable = shooterTable.getSubTable("Control");

        // Telemetry
        flywheelVelocityPub = telemetryTable.getDoubleTopic("Flywheel Velocity").publish();
        flywheelTemperaturePub = telemetryTable.getDoubleTopic("Flywheel Temperature").publish();
        kickerVelocityPub = telemetryTable.getDoubleTopic("Kicker Velocity").publish();
        wantedKickerVelocityPub = telemetryTable.getDoubleTopic("Wanted Kicker Velocity").publish();

        // Flywheel PID
        flywheelKPEntry = flywheelPIDTable.getDoubleTopic("kP").getEntry(TurretConstants.kPfly);
        flywheelKIEntry = flywheelPIDTable.getDoubleTopic("kI").getEntry(TurretConstants.kIfly);
        flywheelKDEntry = flywheelPIDTable.getDoubleTopic("kD").getEntry(TurretConstants.kDfly);
        flywheelKVEntry = flywheelPIDTable.getDoubleTopic("kV").getEntry(TurretConstants.kVfly);

        // Kicker PID
        kickerKPEntry = kickerPIDTable.getDoubleTopic("kP").getEntry(TurretConstants.kPkicker);
        kickerKIEntry = kickerPIDTable.getDoubleTopic("kI").getEntry(TurretConstants.kIkicker);
        kickerKDEntry = kickerPIDTable.getDoubleTopic("kD").getEntry(TurretConstants.kDkicker);
        kickerKVEntry = kickerPIDTable.getDoubleTopic("kV").getEntry(TurretConstants.kVkicker);

        // Control
        customFlywheelVelocityEntry =
            controlTable.getDoubleTopic("Custom Flywheel Velocity").getEntry(3000);

        flywheelMotorLeader = new SparkFlex(TurretConstants.flywheelMotorLeaderID, MotorType.kBrushless);
        flywheelController = flywheelMotorLeader.getClosedLoopController();
        flywheelMotorFollower = new SparkFlex(TurretConstants.flywheelMotorFollowerID, MotorType.kBrushless);

        SparkBaseConfig leaderConfig = new SparkFlexConfig().smartCurrentLimit(60, 20);
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(TurretConstants.kPfly)
            .i(TurretConstants.kIfly)
            .d(TurretConstants.kDfly)
            .outputRange(0, 1)
            .feedForward.kV(TurretConstants.kVfly);

        leaderConfig.closedLoop.maxMotion
            .maxAcceleration(TurretConstants.flyMaxAccel);
        
        flywheelMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkBaseConfig followerConfig = new SparkFlexConfig().smartCurrentLimit(60, 20);
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.follow(TurretConstants.flywheelMotorLeaderID, true); // follower is opposite of leader
        flywheelMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // flywheelPidController = new PIDController(TurretConstants.kPfly, TurretConstants.kIfly, TurretConstants.kDfly);
        // flywheelFFController = new SimpleMotorFeedforward(TurretConstants.kSfly, TurretConstants.kVfly,0);
        // kickerPidController = new PIDController(TurretConstants.kPkicker, TurretConstants.kIkicker, TurretConstants.kDkicker);
        // kickerFFController = new SimpleMotorFeedforward(TurretConstants.kSkicker, TurretConstants.kVkicker,0);
        
        kickerMotor = new SparkFlex(IndexerConstants.kickerMotorID, MotorType.kBrushless);
        kickerController = kickerMotor.getClosedLoopController();
        SparkBaseConfig kickerConfig = new SparkFlexConfig().smartCurrentLimit(40, 20); // NEO_Vortex (Current Limit is 80) 
        kickerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(TurretConstants.kPkicker)
            .i(TurretConstants.kIkicker)
            .d(TurretConstants.kDkicker)
            .outputRange(-1, 1)
            .feedForward.kV(TurretConstants.kVkicker);

        kickerConfig.closedLoop.maxMotion
            .maxAcceleration(TurretConstants.kickerMaxAccel);
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // flywheelPidController.setTolerance(100, 50); // 50 RPM tolerance and 50 RPM/s velocity tolerance
        // kickerPidController.setTolerance(100, 50); // 50 RPM tolerance and 50 RPM/s velocity tolerance
    }
    
    public void runShooter(double speed) {
        flywheelMotorLeader.set(speed);
        kickerMotor.set(Math.min(speed*1.5, 0.75));
    }

    public void setShooterVelocity(double wantedVelocity) {
        // double feedforwardFly = flywheelFFController.calculate(wantedVelocity);
        // double pidFly = flywheelPidController.calculate(flywheelMotorLeader.getEncoder().getVelocity(), wantedVelocity);
        // flywheelMotorLeader.setVoltage(feedforwardFly + pidFly);
        flywheelController.setSetpoint(wantedVelocity, ControlType.kMAXMotionVelocityControl);

        // kicker
        double wantedKickerVelocity = wantedVelocity*1.5;
        wantedKickerVelocity = Math.min(wantedKickerVelocity, 4000); // limit to 4000 RPM to prevent the motor from burning out

        wantedKickerVelocityPub.set(wantedKickerVelocity);

        // double feedforwardKicker = kickerFFController.calculate(wantedKickerVelocity);
        // double pidKicker = kickerPidController.calculate(kickerMotor.getEncoder().getVelocity(), wantedKickerVelocity);
        // kickerMotor.setVoltage(feedforwardKicker + pidKicker);
        kickerController.setSetpoint(wantedKickerVelocity, ControlType.kMAXMotionVelocityControl);
        
        wantedShooterRPM = wantedVelocity;
        wantedKickerRPM = wantedKickerVelocity;
    }
    
    public void setShooterSpeed(double wantedRPM) {
        flywheelController.setSetpoint(wantedRPM, ControlType.kMAXMotionVelocityControl);
        // flywheelMotorLeader.set(flywheelPidController.calculate(flywheelMotorLeader.getEncoder().getVelocity(), wantedRPM));
    }

    public void setState(double shooterState) {
        if (this.state != shooterState) {
            this.state = shooterState;
        }
    }

    public boolean didReachVelocity() {
        // return flywheelPidController.atSetpoint() && kickerPidController.atSetpoint();
        boolean kickerAtRPM = Math.abs(kickerMotor.getEncoder().getVelocity() - wantedKickerRPM) <= 50;
        boolean shooterAtRPM = Math.abs(flywheelMotorLeader.getEncoder().getVelocity() - wantedShooterRPM) <= 50;
        // boolean shooterAtRPM = flywheelController.isAtSetpoint();
        // boolean kickerAtRPM = kickerController.isAtSetpoint();
        // boolean kickerRPMChange = ;
        // boolean shooterRPMChange = flywheelController.isAtSetpoint();
        System.out.println("Kicker" + kickerAtRPM + "shooter" + shooterAtRPM);
        System.out.println(kickerMotor.getEncoder().getVelocity() - wantedKickerRPM);
        return kickerAtRPM && shooterAtRPM;
    }

    public double getDashboardVelocity() {
        return customFlywheelVelocityEntry.get();
    }

    public boolean isShooting() {
        return Math.abs(flywheelMotorLeader.getEncoder().getVelocity()) > 100; // if the flywheel is spinning faster than 100 RPM, we consider it shooting
    }
    
    public void stopFlywheel() {
        flywheelMotorLeader.stopMotor();
    }
    
    @Override
    public void periodic() {
        if (state == ShooterStates.NONE) {
        } else {
            setShooterVelocity(state);
        }

        flywheelVelocityPub.set(flywheelMotorLeader.getEncoder().getVelocity());
        flywheelTemperaturePub.set(flywheelMotorLeader.getMotorTemperature());
        kickerVelocityPub.set(kickerMotor.getEncoder().getVelocity());

        if (!DriverStation.isFMSAttached()) {
            // Read flywheel PID values
            double flyKP = flywheelKPEntry.get();
            double flyKI = flywheelKIEntry.get();
            double flyKD = flywheelKDEntry.get();
            double flyKV = flywheelKVEntry.get();

            // Read kicker PID values
            double kickKP = kickerKPEntry.get();
            double kickKI = kickerKIEntry.get();
            double kickKD = kickerKDEntry.get();
            double kickKV = kickerKVEntry.get();

            // Update flywheel configuration
            if (flyKP != lastFlywheelKP ||
                flyKI != lastFlywheelKI ||
                flyKD != lastFlywheelKD ||
                flyKV != lastFlywheelKV) {

                SparkBaseConfig leaderConfig =
                    new SparkFlexConfig().smartCurrentLimit(60, 20);

                leaderConfig.idleMode(IdleMode.kCoast);

                leaderConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .p(flyKP)
                    .i(flyKI)
                    .d(flyKD)
                    .outputRange(0, 1)
                    .feedForward.kV(flyKV);

                leaderConfig.closedLoop.maxMotion
                    .maxAcceleration(TurretConstants.flyMaxAccel);

                flywheelMotorLeader.configure(
                    leaderConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
                );

                lastFlywheelKP = flyKP;
                lastFlywheelKI = flyKI;
                lastFlywheelKD = flyKD;
                lastFlywheelKV = flyKV;
            }

            // Update kicker configuration
            if (kickKP != lastKickerKP ||
                kickKI != lastKickerKI ||
                kickKD != lastKickerKD ||
                kickKV != lastKickerKV) {

                SparkBaseConfig kickerConfig =
                    new SparkFlexConfig().smartCurrentLimit(40, 20);

                kickerConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .p(kickKP)
                    .i(kickKI)
                    .d(kickKD)
                    .outputRange(-1, 1)
                    .feedForward.kV(kickKV);

                kickerConfig.closedLoop.maxMotion
                    .maxAcceleration(TurretConstants.kickerMaxAccel);

                kickerMotor.configure(
                    kickerConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
                );

                lastKickerKP = kickKP;
                lastKickerKI = kickKI;
                lastKickerKD = kickKD;
                lastKickerKV = kickKV;
            }
        }
    }
}
