// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig.Presets;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ShooterStates;

public class ShooterHoodSubsystem extends SubsystemBase {
    private final TalonFX hoodMotor;
    private final DutyCycleEncoder hoodAbsoluteDutyCycleEncoder;
    private final PIDController hoodPidController;
    
    DoublePublisher absoluteEncodPublisher = NetworkTableInstance.getDefault().getDoubleTopic("hood absolute encoder network").publish();

    private double state = ShooterStates.NONE;

    // upperlimit is actually down and lowerlimit is up
    private double maxMovement = 0.955 - 0.61;
    private double lowerLimit = 0.61;


    public ShooterHoodSubsystem() {
        CANBus rio = new CANBus("rio");
        hoodMotor = new TalonFX(TurretConstants.hoodMotorID, rio);
        CurrentLimitsConfigs hoodCurrentLimits = new CurrentLimitsConfigs();
        hoodCurrentLimits.StatorCurrentLimit = 20;
        hoodCurrentLimits.StatorCurrentLimitEnable = true;
        hoodMotor.getConfigurator().apply(hoodCurrentLimits);

        hoodAbsoluteDutyCycleEncoder = new DutyCycleEncoder(TurretConstants.hoodAbsoluteEncoderID);
        hoodPidController = new PIDController(TurretConstants.kPhood, 0, 0);
    }

    public void setState(double state) {
        if (this.state != state) {
            this.state = state;
        } else {
            this.state = ShooterStates.NONE;
        }
    }

    public boolean didReachValue() {
        return hoodPidController.atSetpoint();
    }

    public void runHood(double speed) {
        // going down and passing encoder

        // boolean lowerLimitHit = hoodAbsoluteDutyCycleEncoder.get() <= lowerLimit;
        // boolean upperLimitHit = hoodAbsoluteDutyCycleEncoder.get() >= upperLimit;

        // if (lowerLimitHit && speed < 0) {
        //     System.out.println("Lower Limit Hit! hood hit top" + lowerLimitHit);
        //     speed = 0;
        // }
        // if (upperLimitHit && speed > 0) {
        //     System.out.println("Upper Limit Hit! hood hit bottom" + upperLimitHit);
        //     speed = 0;
        // }
        hoodMotor.set(speed);
    }
    
    public void setHoodValue(double wantedHoodValue) {
        runHood(hoodPidController.calculate(hoodMotor.getPosition().getValueAsDouble(), wantedHoodValue));
    }

    @Override
    public void periodic() {
        absoluteEncodPublisher.set(hoodAbsoluteDutyCycleEncoder.get());
        SmartDashboard.putNumber("hood absolute encoder smart dashboard", hoodAbsoluteDutyCycleEncoder.get());
        SmartDashboard.putNumber("hood encoder", hoodMotor.getPosition().getValueAsDouble());
        
        if (state != ShooterStates.NONE) {
            setHoodValue(RobotContainer.swerveSubsystem.getTurretToTargetHoodValue());
        }
    }
}
