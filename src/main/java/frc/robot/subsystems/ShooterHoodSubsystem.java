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
    private double zeroLimit = 0.61;
    private boolean finishedZeroing = true;


    public ShooterHoodSubsystem() {
        CANBus rio = new CANBus("rio");
        hoodMotor = new TalonFX(TurretConstants.hoodMotorID, rio);
        CurrentLimitsConfigs hoodCurrentLimits = new CurrentLimitsConfigs();
        hoodCurrentLimits.StatorCurrentLimit = 20;
        hoodCurrentLimits.StatorCurrentLimitEnable = true;
        hoodMotor.getConfigurator().apply(hoodCurrentLimits);

        hoodAbsoluteDutyCycleEncoder = new DutyCycleEncoder(TurretConstants.hoodAbsoluteEncoderID);
        hoodPidController = new PIDController(TurretConstants.kPhood, 0, 0);
        hoodPidController.enableContinuousInput(0, 1);
    }

    public void setState(double state) {
        if (this.state != state) {
            this.state = state;
        } else {
            this.state = ShooterStates.NONE;
        }
    }

    public void zeroHood() {
        finishedZeroing = false;
        hoodMotor.set(0.02);
    }

    public boolean finishedZeroing() {
        if (hoodMotor.getVelocity().getValueAsDouble()  <= 0.002) {
            zeroLimit = hoodAbsoluteDutyCycleEncoder.get();
            hoodMotor.set(0);
            return true;
        } else {
            return false;
        }
    }

    public double getHoodValueFromZero() {
        double actualHoodAngle = zeroLimit - hoodAbsoluteDutyCycleEncoder.get();
        if (actualHoodAngle <= 0) {
            actualHoodAngle += 1;
        }
        return actualHoodAngle;
    }

    public boolean didReachValue() {
        return hoodPidController.atSetpoint();
    }

    public void runHood(double speed) {
        // going down and passing encoder

        boolean lowerLimitHit = getHoodValueFromZero() <= 0;
        boolean upperLimitHit = getHoodValueFromZero() >= maxMovement;

        if (lowerLimitHit && speed > 0) {
            System.out.println("Lower Limit Hit! hood hit zero" + lowerLimitHit);
            speed = 0;
        }
        if (upperLimitHit && speed < 0) {
            System.out.println("Upper Limit Hit! hood hit max" + upperLimitHit);
            speed = 0;
        }
        hoodMotor.set(speed);
    }
    
    // wantedHoodValue should be encoder ticks above from 0
    public void setHoodValue(double wantedHoodValueFromZero) {
        // value is from how much increased from our zero value
        double actualHoodAngle = hoodAbsoluteDutyCycleEncoder.get() - wantedHoodValueFromZero;
        if (actualHoodAngle <= 0) {
            actualHoodAngle += 1;
        }
        runHood(hoodPidController.calculate(hoodAbsoluteDutyCycleEncoder.get(), actualHoodAngle));
    }

    @Override
    public void periodic() {
        absoluteEncodPublisher.set(hoodAbsoluteDutyCycleEncoder.get());
        SmartDashboard.putNumber("hood absolute encoder dash", hoodAbsoluteDutyCycleEncoder.get());
        SmartDashboard.putNumber("hood absolute encoder from zero", getHoodValueFromZero());
        
        if (state != ShooterStates.NONE) {
            setHoodValue(RobotContainer.swerveSubsystem.getTurretToTargetHoodValue());
        }

        // TODO: REMOVE FOR COMP
        double kPHood = SmartDashboard.getNumber("kP hood", TurretConstants.kPhood);

        SmartDashboard.putNumber("kP hood", kPHood);
        if (SmartDashboard.getNumber("kP hood", TurretConstants.kPhood) != hoodPidController.getP()) {
            hoodPidController.setP(kPHood);
            System.out.println("Updated hood PID and FF values: kP = " + kPHood);
        }
    }
}
