// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ShooterStates;

public class ShooterHoodSubsystem extends SubsystemBase {
    private final TalonFX hoodMotor;
    private final DutyCycleEncoder hoodAbsoluteDutyCycleEncoder;
    // private final PIDController hoodPidController;
    private final SimpleMotorFeedforward feedforward;
    
    DoublePublisher absoluteEncodPublisher = NetworkTableInstance.getDefault().getDoubleTopic("hood absolute encoder network").publish();
    // NetworkTables
    private NetworkTable hoodTable;
    private NetworkTable telemetryTable;
    private NetworkTable pidTable;
    private NetworkTable controlTable;

    // Telemetry publishers
    private DoublePublisher hoodPositionPub;
    private DoublePublisher hoodVelocityPub;
    private DoublePublisher hoodCurrentPub;
    private DoublePublisher absoluteEncoderPub;

    // PID entries
    private DoubleEntry hoodKPEntry;
    private DoubleEntry hoodKIEntry;
    private DoubleEntry hoodKDEntry;
    private DoubleEntry hoodKSEntry;

    // Control entries
    private DoubleEntry customHoodValueEntry;

    private double state = ShooterStates.NONE;

    // upperlimit is actually down and lowerlimit is up
    // private double maxMovement = 0.955 - 0.61;
    // private double zeroLimit = 0.375;
    
    PositionVoltage positionVoltage = new PositionVoltage(0);
    private double maxMovement = 5.0;
    private double wantedHoodValue = 0;
    private double lastP = TurretConstants.kPhood;
    private double lastI = TurretConstants.kIhood;
    private double lastD = TurretConstants.kDhood;
    private double lastS = TurretConstants.kShood;

    public ShooterHoodSubsystem() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        hoodTable = nt.getTable("ShooterHood");

        telemetryTable = hoodTable.getSubTable("Telemetry");
        pidTable = hoodTable.getSubTable("PID");
        controlTable = hoodTable.getSubTable("Control");

        // Telemetry
        hoodPositionPub = telemetryTable.getDoubleTopic("Hood motor position").publish();
        hoodVelocityPub = telemetryTable.getDoubleTopic("Hood motor velocity").publish();
        hoodCurrentPub = telemetryTable.getDoubleTopic("Hood stall current").publish();
        absoluteEncoderPub = telemetryTable.getDoubleTopic("Absolute encoder").publish();

        // PID tuning
        hoodKPEntry = pidTable.getDoubleTopic("kP").getEntry(TurretConstants.kPhood);
        hoodKIEntry = pidTable.getDoubleTopic("kI").getEntry(TurretConstants.kIhood);
        hoodKDEntry = pidTable.getDoubleTopic("kD").getEntry(TurretConstants.kDhood);
        hoodKSEntry = pidTable.getDoubleTopic("kS").getEntry(TurretConstants.kShood);

        // Control
        customHoodValueEntry =
            controlTable.getDoubleTopic("Custom hoodValue").getEntry(0);
        CANBus rio = CANBus.roboRIO();
        hoodMotor = new TalonFX(TurretConstants.hoodMotorID, rio);

        TalonFXConfiguration config = new TalonFXConfiguration();

        CurrentLimitsConfigs hoodCurrentLimits = config.CurrentLimits;
        hoodCurrentLimits.StatorCurrentLimit = 20;
        hoodCurrentLimits.StatorCurrentLimitEnable = true;

        Slot0Configs pidConfigs = config.Slot0;
        pidConfigs.kP = TurretConstants.kPhood;
        pidConfigs.kI = TurretConstants.kIhood;
        pidConfigs.kD = TurretConstants.kDhood;
        pidConfigs.kS = TurretConstants.kShood;
        
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 1;

        hoodMotor.getConfigurator().apply(config);

        hoodAbsoluteDutyCycleEncoder = new DutyCycleEncoder(TurretConstants.hoodAbsoluteEncoderID);
        // hoodPidController = new PIDController(TurretConstants.kPhood, TurretConstants.kIhood, TurretConstants.kDhood);
        // hoodPidController.setTolerance(0.1);
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
        return customHoodValueEntry.get();
    }

    public void zeroHood() {
        hoodMotor.set(-0.05);
    }

    public void setCurrentToZero() {
        hoodMotor.setPosition(-0.3);
    }

    public boolean finishedZeroing() {
        // System.out.println(hoodMotor.getSupplyCurrent().getValueAsDouble());
        
        if (hoodMotor.getSupplyCurrent().getValueAsDouble() >= 0.6) {
            hoodMotor.setPosition(-0.3);
            hoodMotor.set(0);
            return true;
        } else {
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
        return Math.abs(hoodMotor.getPosition().getValueAsDouble() - wantedHoodValue) < 0.01;
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
        wantedHoodValue = wantedHoodValueFromZero;
        // value is from how much increased from our zero value
        hoodMotor.setControl(positionVoltage.withPosition(wantedHoodValueFromZero).withFeedForward(0));
        // hoodMotor.setVoltage(hoodPidController.calculate(hoodMotor.getPosition().getValueAsDouble(), wantedHoodValueFromZero) + feedforward.calculate(hoodMotor.getVelocity().getValueAsDouble()));
    }

    @Override
    public void periodic() {
        hoodPositionPub.set(hoodMotor.getPosition().getValueAsDouble());
        hoodVelocityPub.set(hoodMotor.getVelocity().getValueAsDouble());
        hoodCurrentPub.set(hoodMotor.getSupplyCurrent().getValueAsDouble());
        absoluteEncoderPub.set(hoodAbsoluteDutyCycleEncoder.get());

        if (state != ShooterStates.NONE) {
            setHoodValue(RobotContainer.swerveSubsystem.getTurretToTargetHoodValue());
        }

        // Only allow PID tuning when NOT connected to FMS
        if (!DriverStation.isFMSAttached()) {
            double kPHood = hoodKPEntry.get();
            double kIHood = hoodKIEntry.get();
            double kDHood = hoodKDEntry.get();
            double kSHood = hoodKSEntry.get();

            if (kPHood != lastP ||
                kIHood != lastI ||
                kDHood != lastD ||
                kSHood != lastS) {

                Slot0Configs pidConfigs = new Slot0Configs()
                    .withKP(kPHood)
                    .withKI(kIHood)
                    .withKD(kDHood)
                    .withKS(kSHood);

                hoodMotor.getConfigurator().apply(pidConfigs);

                lastP = kPHood;
                lastI = kIHood;
                lastD = kDHood;
                lastS = kSHood;
            }
        }
    }
}
