// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeArmStates;
import frc.robot.Constants.IntakeConstants;

public class IntakeArmSubsystem extends SubsystemBase {
    private final TalonFX intakeArmMotorLeader;
    private final TalonFX intakeArmMotorFollower;
    private final DigitalInput intakeArmZeroLimitSwitch;
    private final PIDController intakeArmPidController;
    private final Trigger intakeArmTrigger;

    private double state = IntakeArmStates.NONE;

    public IntakeArmSubsystem() {
        CANBus rio = new CANBus("rio"); // TODO: Default Coast when disabled, and Brake when enabled
        intakeArmMotorLeader = new TalonFX(IntakeConstants.leftMotorID, rio);
        intakeArmMotorFollower = new TalonFX(IntakeConstants.rightMotorID, rio);

        intakeArmMotorFollower.setControl(new Follower(intakeArmMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        intakeArmZeroLimitSwitch = new DigitalInput(IntakeConstants.intakeArmZeroLimitSwitchID);
        intakeArmPidController = new PIDController(IntakeConstants.armkP, 0, 0);

        intakeArmTrigger = new Trigger(intakeArmZeroLimitSwitch::get);
    }

    public void runIntakeArm(double speed) {
        
        // if limit switch is pressed, and going same direction as limit switch, STOP
        // if (intakeArmZeroLimitSwitch.get() && speed < 0) {
        //     speed = 0;
        // }
        SmartDashboard.putNumber("Arm speed", speed);
        intakeArmMotorLeader.set(speed);
    }
    
    public void setIntakeArmAngle(double angle) {
        runIntakeArm(intakeArmPidController.calculate(intakeArmMotorLeader.getPosition().getValueAsDouble(), angle));
    }

    public void setState(double armState) {
        if (this.state != state) {
            intakeArmPidController.reset();
            this.state = armState;
        }
    }
    
    public boolean didReachState() {
        return intakeArmPidController.atSetpoint();
    }

    @Override
    public void periodic() {
        // Pulish values on Elastic, this goes to SmartDashboard - Lewi
        SmartDashboard.putNumber("Intake Arm position", intakeArmMotorLeader.getPosition().getValueAsDouble());
        
        if (state == IntakeArmStates.NONE) {
        } else {
            setIntakeArmAngle(state);
        }
    }
}
