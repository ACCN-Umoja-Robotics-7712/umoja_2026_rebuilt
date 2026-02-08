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
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig.Presets;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TurretConstants;

public class ShooterFlywheelSubsystem extends SubsystemBase {
    private final SparkFlex flywheelMotorLeader;
    private final SparkFlex flywheelMotorFollower;

    private final PIDController flywheelPidController;
    private final SimpleMotorFeedforward FFController;

    public ShooterFlywheelSubsystem() {
        flywheelMotorLeader = new SparkFlex(TurretConstants.flywheelMotorLeaderID, MotorType.kBrushless);
        flywheelMotorFollower = new SparkFlex(TurretConstants.flywheelMotorFollowerID, MotorType.kBrushless);

        SparkBaseConfig leaderConfig = Presets.REV_Vortex;
        flywheelMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkBaseConfig followerConfig = Presets.REV_Vortex;
        followerConfig.follow(TurretConstants.flywheelMotorLeaderID, true); // follower is opposite of leader
        flywheelMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelPidController = new PIDController(TurretConstants.kPfly, TurretConstants.kIfly, 0);
        FFController = new SimpleMotorFeedforward(TurretConstants.kSfly, 0,0);

    }
    public void runShooter(double speed) {
        flywheelMotorLeader.set(speed);
    }

    public void setShooterVelocity(double wantedVelocity) {
        double feedforward = FFController.calculate(wantedVelocity);
        double pid = flywheelPidController.calculate(flywheelMotorLeader.getEncoder().getVelocity(), wantedVelocity);
        flywheelMotorLeader.setVoltage(feedforward + pid);
    }
    
    @Override
    public void periodic() {
    }
}
