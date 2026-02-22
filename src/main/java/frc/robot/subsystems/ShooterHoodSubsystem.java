// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig.Presets;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.hoodStates;

public class ShooterHoodSubsystem extends SubsystemBase {
    private final TalonFX hoodMotor;
    private final DutyCycleEncoder hoodAbsolDutyCycleEncoder;
    private final PIDController hoodPidController;
    
    DoublePublisher absoluteEncodPublisher = NetworkTableInstance.getDefault().getDoubleTopic("hood absolute encoder").publish();

    private double state = hoodStates.NONE;


    public ShooterHoodSubsystem() {
        CANBus rio = new CANBus("rio");
        hoodMotor = new TalonFX(TurretConstants.hoodMotorID, rio);
        hoodAbsolDutyCycleEncoder = new DutyCycleEncoder(TurretConstants.hoodAbsoluteEncoderID);
        hoodPidController = new PIDController(TurretConstants.kPhood, 0, 0);
    }

    public void setState(double hoodStates) {
        if (this.state != hoodStates) {
            hoodPidController.reset();
            this.state = hoodStates;
        }
    }

    public boolean didReachState() {
        return hoodPidController.atSetpoint();
    }

    public void runHood(double speed) {
        hoodMotor.set(speed);
    }
    
    public void setHoodAngle(double wantedHoodRotation) {
        hoodMotor.set(hoodPidController.calculate(hoodMotor.getPosition().getValueAsDouble(), wantedHoodRotation));
    }

    @Override
    public void periodic() {
        absoluteEncodPublisher.accept(hoodAbsolDutyCycleEncoder.get());
        SmartDashboard.putNumber("Wanted angle", hoodAbsolDutyCycleEncoder.get());
    }
}
