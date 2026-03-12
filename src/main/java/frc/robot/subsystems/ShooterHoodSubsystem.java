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
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private final SimpleMotorFeedforward feedforward;
    private double lastPositiveVelocity = Double.POSITIVE_INFINITY;
    
    DoublePublisher absoluteEncodPublisher = NetworkTableInstance.getDefault().getDoubleTopic("hood absolute encoder network").publish();

    private double state = ShooterStates.NONE;

    // upperlimit is actually down and lowerlimit is up
    // private double maxMovement = 0.955 - 0.61;
    // private double zeroLimit = 0.375;
    private double maxMovement = 5.0;

    public ShooterHoodSubsystem() {
        CANBus rio = new CANBus("rio");
        hoodMotor = new TalonFX(TurretConstants.hoodMotorID, rio);
        CurrentLimitsConfigs hoodCurrentLimits = new CurrentLimitsConfigs();
        hoodCurrentLimits.StatorCurrentLimit = 20;
        hoodCurrentLimits.StatorCurrentLimitEnable = true;
        hoodMotor.getConfigurator().apply(hoodCurrentLimits);

        hoodAbsoluteDutyCycleEncoder = new DutyCycleEncoder(TurretConstants.hoodAbsoluteEncoderID);
        hoodPidController = new PIDController(TurretConstants.kPhood, TurretConstants.kIhood, TurretConstants.kDhood);
        hoodPidController.setTolerance(0.1);
        feedforward = new SimpleMotorFeedforward(TurretConstants.kShood, 0, 0);
    }

    public void setState(double state) {
        if (this.state != state) {
            this.state = state;
        } else {
            this.state = ShooterStates.NONE;
        }
    }

    public double getDashboardHoodValue() {
        double hoodValue = SmartDashboard.getNumber("Custom hoodValue", 0);
        SmartDashboard.putNumber("Custom hoodValue", hoodValue);
        return hoodValue;
    }


    public void zeroHood() {
        hoodMotor.set(-0.1);
    }

    public void setCurrentToZero() {
        hoodMotor.setPosition(-0.2);
    }

    public boolean finishedZeroing() {
        System.out.println(lastPositiveVelocity);
        
        if (lastPositiveVelocity  <= 0.05) {
            hoodMotor.setPosition(-0.2);
            hoodMotor.set(0);
            return true;
        } else {
            double currVel = Math.abs(hoodMotor.getVelocity().getValueAsDouble());
            if (currVel > 0) {
                lastPositiveVelocity = Math.abs(hoodMotor.getVelocity().getValueAsDouble());
            }
            return false;
        }
    }

    // public double getHoodValueFromZero() {
    //     // num from 1-0 -> 1-0
    //     // num from 0.25 going down to 0.26... -> 0 down to 0.1 essentiall 1...0
    //     double normalEncoderValue = zeroLimit - hoodAbsoluteDutyCycleEncoder.get();
    //     // value now from 0 to 1
    //     double actualHoodAngle = 1 - normalEncoderValue;
    //     if (actualHoodAngle < 0) {
    //         actualHoodAngle += 1;
    //     }
    //     return actualHoodAngle;
    // }

    public boolean didReachValue() {
        return hoodPidController.atSetpoint();
    }

    public void runHood(double speed) {

        double position = hoodMotor.getPosition().getValueAsDouble();
        boolean lowerLimitHit = position <= 0.0;
        boolean upperLimitHit = position >= maxMovement;

        if (lowerLimitHit && speed < 0) {
            System.out.println("Lower Limit Hit! hood hit zero" + lowerLimitHit);
            speed = 0;
        }
        if (upperLimitHit && speed > 0) {
            System.out.println("Upper Limit Hit! hood hit max" + upperLimitHit);
            speed = 0;
        }
        hoodMotor.set(speed);
    }
    
    // wantedHoodValue should be encoder ticks above from 0
    public void setHoodValue(double wantedHoodValueFromZero) {
        // value is from how much increased from our zero value
        hoodMotor.setVoltage(hoodPidController.calculate(hoodMotor.getPosition().getValueAsDouble(), wantedHoodValueFromZero) + feedforward.calculate(hoodMotor.getVelocity().getValueAsDouble()));
    }

    @Override
    public void periodic() {
        // absoluteEncodPublisher.set(hoodAbsoluteDutyCycleEncoder.get());
        // SmartDashboard.putNumber("hood absolute encoder dash", hoodAbsoluteDutyCycleEncoder.get());
        // SmartDashboard.putNumber("hood absolute encoder from zero", getHoodValueFromZero());
        SmartDashboard.putNumber("hood motor position", hoodMotor.getPosition().getValueAsDouble());    
        SmartDashboard.putNumber("hood motor velocity", hoodMotor.getVelocity().getValueAsDouble());     
        SmartDashboard.putNumber("hood maxMovement", maxMovement);

        if (state != ShooterStates.NONE) {
            setHoodValue(RobotContainer.swerveSubsystem.getTurretToTargetHoodValue());
        }

        // TODO: REMOVE FOR COMP
        double kPHood = SmartDashboard.getNumber("kP hood", TurretConstants.kPhood);
        double kIHood = SmartDashboard.getNumber("kI hood", TurretConstants.kIhood);
        double kSHood = SmartDashboard.getNumber("kS hood", TurretConstants.kShood);
        double kDHood = SmartDashboard.getNumber("kD hood", TurretConstants.kDhood);

        SmartDashboard.putNumber("kP hood", kPHood);
        SmartDashboard.putNumber("kI hood", kIHood);
        SmartDashboard.putNumber("kS hood", kSHood);
        SmartDashboard.putNumber("kD hood", kDHood);

        if (SmartDashboard.getNumber("kP hood", kPHood) != hoodPidController.getP() || SmartDashboard.getNumber("kI hood", kIHood) != hoodPidController.getI() || SmartDashboard.getNumber("kS hood", kSHood) != feedforward.getKs() || SmartDashboard.getNumber("kD hood", kDHood) != hoodPidController.getD()) {
            hoodPidController.setP(kPHood);
            hoodPidController.setI(kIHood);
            hoodPidController.setD(kDHood);
            feedforward.setKs(kSHood);
            System.out.println("Updated hood PID and FF values: kP = " + kPHood + "kI = " + kSHood + "kI = " + kSHood);
        }
    }
}
