// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX flywheelMotor;
    private final TalonFX turretMotor;
    private final DigitalInput turretZeroLimitSwitch;
    private final TalonFX hoodMotor;
    private final DigitalInput hoodZeroLimitSwitch;

    private final PIDController flywheelPidController;
    private final PIDController turretPidController;
    private final PIDController hoodPidController;

    public ShooterSubsystem() {
        CANBus CANivore = new CANBus("CANivore");
        flywheelMotor = new TalonFX(0, CANivore);
        turretMotor = new TalonFX(1, CANivore);
        hoodMotor = new TalonFX(2, CANivore);
        turretZeroLimitSwitch = new DigitalInput(0);
        hoodZeroLimitSwitch = new DigitalInput(1);

        flywheelPidController = new PIDController(0.01, 0, 0);
        turretPidController = new PIDController(0.01, 0, 0);
        hoodPidController = new PIDController(0.01, 0, 0);
    }

    public void runShooter(double speed) {
        flywheelMotor.set(speed);
    }

    public void runTurret(double speed) {
        // if limit switch is pressed, and going same direction as limit switch, STOP
        if (turretZeroLimitSwitch.get() && speed < 0) {
            speed = 0;
        }
        
        turretMotor.set(speed);
    }

    public void runHood(double speed) {
        // if limit switch is pressed, and going same direction as limit switch, STOP
        if (hoodZeroLimitSwitch.get() && speed < 0) {
            speed = 0;
        }

        hoodMotor.set(speed);
    }

    public void setShooterSpeed(double wantedSpeed) {
        flywheelMotor.set(flywheelPidController.calculate(flywheelMotor.getVelocity().getValueAsDouble(), wantedSpeed));
    }

    public void setTurretAngle(double wantedTurretRotation) {
        turretMotor.set(turretPidController.calculate(turretMotor.getPosition().getValueAsDouble(), wantedTurretRotation));
    }
    
    public void setHoodAngle(double wantedHoodRotation) {
        hoodMotor.set(hoodPidController.calculate(hoodMotor.getPosition().getValueAsDouble(), wantedHoodRotation));
    }
    
    @Override
    public void periodic() {
        if (turretZeroLimitSwitch.get()) {
            turretMotor.setPosition(0);
        }
        if (hoodZeroLimitSwitch.get()) {
            hoodMotor.setPosition(0);
        }
    }
}
