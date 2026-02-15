// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig.Presets;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ShooterStates;
import frc.robot.Constants.hoodStates;
import frc.robot.Constants;

public class ShooterHoodSubsystem extends SubsystemBase {
    private final SparkMax hoodMotor;

    private final PIDController hoodPidController;
    
    DoublePublisher absoluteEncodPublisher = NetworkTableInstance.getDefault().getDoubleTopic("hood absolute encoder").publish();

    private double state = hoodStates.NONE;

    public ShooterHoodSubsystem() {
        hoodMotor = new SparkMax(TurretConstants.hoodMotorID, MotorType.kBrushless);
        
        SparkBaseConfig config = Presets.REV_NEO_550;
        config.inverted(TurretConstants.hoodMotorReversed);
        hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodPidController = new PIDController(TurretConstants.kPhood, 0, 0);
    }

    public void setState(double hoodStates) {
        if (this.state != hoodStates) {
            hoodPidController.reset();
            this.state = hoodStates;
        }
    }

    public void runHood(double speed) {
        hoodMotor.set(speed);
    }
    
    public void setHoodAngle(double wantedHoodRotation) {
        hoodMotor.set(hoodPidController.calculate(hoodMotor.getAbsoluteEncoder().getPosition(), wantedHoodRotation));
    }

    @Override
    public void periodic() {
        absoluteEncodPublisher.accept(hoodMotor.getAbsoluteEncoder().getPosition());
    }
}
