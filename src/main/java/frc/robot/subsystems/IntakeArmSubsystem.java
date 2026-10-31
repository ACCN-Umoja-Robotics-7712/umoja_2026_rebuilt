// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeArmStates;
import frc.robot.Constants.IntakeConstants;

public class IntakeArmSubsystem extends SubsystemBase {
    private final TalonFX intakeArmMotorLeader;
    private final TalonFX intakeArmMotorFollower;
    private final ArmFeedforward intakeArmFeedforward;
    private final PIDController intakeArmPidController;

    private NetworkTable intakeArmTable;
    private NetworkTable telemetryTable;

    private DoublePublisher armSpeedPub;
    private DoublePublisher armPositionPub;
    private DoublePublisher armCurrentPub;

    private double state = IntakeArmStates.NONE;

    public IntakeArmSubsystem() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        intakeArmTable = nt.getTable("IntakeArm");
        telemetryTable = intakeArmTable.getSubTable("Telemetry");

        armSpeedPub = telemetryTable.getDoubleTopic("Arm speed").publish();
        armPositionPub = telemetryTable.getDoubleTopic("Arm position").publish();
        armCurrentPub = telemetryTable.getDoubleTopic("Arm current stall").publish();
    
        CANBus rio = CANBus.roboRIO(); // TODO: Default Coast when disabled, and Brake when enabled
        intakeArmMotorLeader = new TalonFX(IntakeConstants.leftMotorID, rio);
        intakeArmMotorFollower = new TalonFX(IntakeConstants.rightMotorID, rio);
        CurrentLimitsConfigs armCurrentLimits = new CurrentLimitsConfigs();
        armCurrentLimits.StatorCurrentLimit = 60;
        armCurrentLimits.StatorCurrentLimitEnable = true;
        armCurrentLimits.SupplyCurrentLimit = 30;
        armCurrentLimits.SupplyCurrentLimitEnable = true;
        intakeArmMotorLeader.getConfigurator().apply(armCurrentLimits);
        intakeArmMotorFollower.getConfigurator().apply(armCurrentLimits);
        
        // TalonFXConfiguration armConfig = new TalonFXConfiguration();
        // armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // intakeArmMotorLeader.getConfigurator().apply(armConfig);
        intakeArmMotorFollower.setControl(new Follower(intakeArmMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        // intakeArmZeroLimitSwitch = new DigitalInput(IntakeConstants.intakeArmZeroLimitSwitchID);
        intakeArmPidController = new PIDController(IntakeConstants.armkP, 0, 0);
        intakeArmFeedforward = new ArmFeedforward(0, IntakeConstants.armkG, IntakeConstants.armkV, 0);

        // intakeArmTrigger = new Trigger(intakeArmZeroLimitSwitch::get);
        setBrakeMode(NeutralModeValue.Coast);
    }

    public void zeroIntakeArm() {
        intakeArmMotorLeader.setPosition(0);
    }

    public boolean isIntakeLowered() {
        return intakeArmMotorLeader.getPosition().getValueAsDouble() >= 1.5;
    }

    public void driveMotorDown() {
        if (isIntakeLowered() && intakeArmMotorLeader.getVelocity().getValueAsDouble() < -0.1) {
            runIntakeArm(0.05);
        } else {
            runIntakeArm(0);
        }
    }

    public void setBrakeMode(NeutralModeValue mode) {
        intakeArmMotorLeader.setNeutralMode(mode);
    }

    public void runIntakeArm(double speed) {
        
        // if limit switch is pressed, and going same direction as limit switch, STOP
        // if (intakeArmZeroLimitSwitch.get() && speed < 0) {
        //     speed = 0;
        // }
        armSpeedPub.set(speed);
        intakeArmMotorLeader.set(speed);
    }
    
    public void setIntakeArmAngle(double angle) {
        runIntakeArm(intakeArmPidController.calculate(intakeArmMotorLeader.getPosition().getValueAsDouble(), angle));
    }

    public void setState(double armState) {
        if (this.state != armState) {
            intakeArmPidController.reset();
            this.state = armState;
        }
    }
    
    public boolean didReachState() {
        return intakeArmPidController.atSetpoint();
    }

    @Override
    public void periodic() {
        armPositionPub.set(intakeArmMotorLeader.getPosition().getValueAsDouble());
        armCurrentPub.set(intakeArmMotorLeader.getStatorCurrent().getValueAsDouble());

        if (state == IntakeArmStates.NONE) {
        } else {
            setIntakeArmAngle(state);
        }
    }
}
