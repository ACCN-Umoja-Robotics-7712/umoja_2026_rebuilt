// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterTurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final DigitalInput turretZeroLimitSwitch;

    private final PIDController turretPidController;


    public ShooterTurretSubsystem() {
        CANBus CANivore = new CANBus("CANivore");
        turretMotor = new TalonFX(1, CANivore);
        turretZeroLimitSwitch = new DigitalInput(0);
        
        turretPidController = new PIDController(0.01, 0, 0);
    }

    public void runTurret(double speed) {
        // if limit switch is pressed, and going same direction as limit switch, STOP
        if (turretZeroLimitSwitch.get() && speed < 0) {
            speed = 0;
        }
        
        turretMotor.set(speed);
    }

    public void setTurretAngle(double wantedTurretRotation) {
        turretMotor.set(turretPidController.calculate(turretMotor.getPosition().getValueAsDouble(), wantedTurretRotation));
    }

    @Override
    public void periodic() {
        if (turretZeroLimitSwitch.get()) {
            turretMotor.setPosition(0);
        }
    }
}
