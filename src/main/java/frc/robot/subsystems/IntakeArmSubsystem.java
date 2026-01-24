// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeArmSubsystem extends SubsystemBase {
    private final TalonFX IntakeArmMotor;
    private final DigitalInput IntakeArmZeroLimitSwitch;
    private final PIDController IntakeArmPidController;

    private final VoltageOut voltageReg;

    public IntakeArmSubsystem() {
        CANBus CANivore = new CANBus("CANivore");
        IntakeArmMotor = new TalonFX(0, CANivore);
        IntakeArmZeroLimitSwitch = new DigitalInput(1);
        IntakeArmPidController = new PIDController(0.01, 0, 0);
        
    }

    public void runIntakeArm(double speed) {
        
        // if limit switch is pressed, and going same direction as limit switch, STOP
        if (hoodZeroLimitSwitch.get() && speed < 0) {
            speed = 0;
        }
        IntakeArmMotor.set(speed);
    }
    
    public void setIntakeArmSpeed(double wantedSpeed) {
        IntakeArmMotor.set(IntakeArmPidController.calculate(IntakeArmMotor.getVelocity().getValueAsDouble(), wantedSpeed));
    }

    public void setIntakeArmAngle(double wantedIntakeArmRotation) {
        IntakeArmMotor.set(IntakeArmPidController.calculate(IntakeArmMotor.getPosition().getValueAsDouble(), wantedIntakeArmRotation));
    }

    @Override
    public void periodic() {
    }
}
