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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmStates;

public class IntakeArmSubsystem extends SubsystemBase {
    private final TalonFX intakeArmMotorLeader;
    private final TalonFX intakeArmMotorFollower;
    private final DigitalInput intakeArmZeroLimitSwitch;
    private final PIDController intakeArmPidController;

    private double state = IntakeArmStates.NONE;

    public IntakeArmSubsystem() {
        CANBus CANivore = new CANBus("CANivore");
        intakeArmMotorLeader = new TalonFX(0, CANivore);
        intakeArmMotorFollower = new TalonFX(1, CANivore);

        intakeArmMotorFollower.setControl(new Follower(intakeArmMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        intakeArmZeroLimitSwitch = new DigitalInput(1);
        intakeArmPidController = new PIDController(0.01, 0, 0);
    }

    public void runIntakeArm(double speed) {
        
        // if limit switch is pressed, and going same direction as limit switch, STOP
        if (intakeArmZeroLimitSwitch.get() && speed < 0) {
            speed = 0;
        }
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
        if (state == IntakeArmStates.NONE) {
            runIntakeArm(0);
        } else {
            setIntakeArmAngle(state);
        }
    }
}
