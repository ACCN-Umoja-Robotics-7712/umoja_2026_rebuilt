// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterStates;

public class ShooterFlywheelSubsystem extends SubsystemBase {

    private final TalonFX flywheelMotor;

    private final PIDController flywheelPidController;

    private double state = ShooterStates.NONE;

    public ShooterFlywheelSubsystem() {
        CANBus CANivore = new CANBus("CANivore");
        flywheelMotor = new TalonFX(0, CANivore);

        flywheelPidController = new PIDController(0.01, 0, 0);
    }
    public void runShooter(double speed) {
        flywheelMotor.set(speed);
    }

    public void setShooterSpeed(double wantedSpeed) {
        flywheelMotor.set(flywheelPidController.calculate(flywheelMotor.getVelocity().getValueAsDouble(), wantedSpeed));
    }

    public void setState(double ShooterState) {
        if (this.state != ShooterState) {
            flywheelPidController.reset();
            this.state = ShooterState;
        }
    }

      public boolean didReachState() {
        return flywheelPidController.atSetpoint();
    }
    
    @Override
    public void periodic() {
        if (state == ShooterStates.NONE) {
            runShooter(0);
        } else {
            setShooterSpeed(state);
        }
    }
}
