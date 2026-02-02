// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterFlywheelSubsystem extends SubsystemBase {
    private final SparkFlex flywheelMotor;
    private final SparkFlex flywheelMotorFollower;

    private final PIDController flywheelPidController;
    private final SimpleMotorFeedforward FFController;

    public ShooterFlywheelSubsystem() {
        // TODO: Update to constants file
        int leader = 1;
        flywheelMotor = new SparkFlex(leader, MotorType.kBrushless);
        flywheelMotorFollower = new SparkFlex(2, MotorType.kBrushless);

        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig.follow(leader, true);

        flywheelMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelPidController = new PIDController(0, 0, 0);
        FFController = new SimpleMotorFeedforward(0, 0,0);

    }
    public void runShooter(double speed) {
        flywheelMotor.set(speed);
    }

    public void setShooterVelocity(double wantedVelocity) {
        double feedforward = FFController.calculate(wantedVelocity);
        double pid = flywheelPidController.calculate(flywheelMotor.getEncoder().getVelocity(), wantedVelocity);
        flywheelMotor.setVoltage(feedforward + pid);
    }
    
    @Override
    public void periodic() {
    }
}
