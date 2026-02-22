// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig.Presets;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretStates;

public class ShooterTurretSubsystem extends SubsystemBase {
    private final SparkMax turretMotor;
    private final DigitalInput turretZeroLimitSwitch;

    private final PIDController turretPidController;

    private double state = TurretStates.NONE;

    public ShooterTurretSubsystem() {
        turretMotor = new SparkMax(TurretConstants.turretMotorID, MotorType.kBrushless);
        SparkBaseConfig turretConfig = new SparkMaxConfig().smartCurrentLimit(80);
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretZeroLimitSwitch = new DigitalInput(TurretConstants.turretLimitSwitchID);
        
        turretPidController = new PIDController(TurretConstants.kPturret, 0, 0);
    }

    public void runTurret(double speed) {
        // if limit switch is pressed, and going same direction as limit switch, STOP
        // if (turretZeroLimitSwitch.get() && speed < 0) {
        //     speed = 0;
        // }
        
        turretMotor.set(speed);
    }

    public void setTurretAngle(double wantedTurretRotation) {
        turretMotor.set(turretPidController.calculate(turretMotor.getEncoder().getPosition(), wantedTurretRotation));
    }

    public void setState(double TurretStates) {
        if (this.state != TurretStates) {
            turretPidController.reset();
            this.state = TurretStates;
        }

    }

    @Override
    public void periodic() {
        // if (turretZeroLimitSwitch.get()) {
        //     turretMotor.getEncoder().setPosition(0);
        // }
    }
}
