// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig.Presets;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterStates;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.BooleanPublisher;

public class ShooterTurretSubsystem extends SubsystemBase {
    private final SparkClosedLoopController turretController;
    private final SparkMax turretMotor;
    private final DigitalInput turretZeroLimitSwitch;

    // private final PIDController turretPidController;
    // private final SimpleMotorFeedforward turretFeedforward;

    private double state = ShooterStates.NONE;
    private boolean isZeroed = false;

    private double minAngle = 30;
    private double maxAngle = 280;
    private double wantedTurretAngle = 180;
    private double lastDirection = 0;

    // NetworkTables
    private NetworkTable turretTable;
    private NetworkTable telemetryTable;
    private NetworkTable controlTable;
    private NetworkTable pidTable;
    
    // PID entries
    private DoubleEntry turretKPEntry;
    private DoubleEntry turretKIEntry;
    private DoubleEntry turretKSEntry;

    // Telemetry publishers
    private DoublePublisher turretEncoderPub;
    private DoublePublisher wantedAnglePub;
    private BooleanPublisher limitSwitchPub;

    // Control entries
    private DoubleEntry customAngleEntry;

    public ShooterTurretSubsystem() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        turretTable = nt.getTable("ShooterTurret");

        telemetryTable = turretTable.getSubTable("Telemetry");
        controlTable = turretTable.getSubTable("Control");
        pidTable = turretTable.getSubTable("PID");

        turretKPEntry = pidTable.getDoubleTopic("kP").getEntry(TurretConstants.kPturret);
        turretKIEntry = pidTable.getDoubleTopic("kI").getEntry(TurretConstants.kIturret);
        turretKSEntry = pidTable.getDoubleTopic("kS").getEntry(TurretConstants.turretFakeFeedForward);

        // Telemetry
        turretEncoderPub = telemetryTable.getDoubleTopic("Turret encoder").publish();
        wantedAnglePub = telemetryTable.getDoubleTopic("New angle that we want").publish();
        limitSwitchPub = telemetryTable.getBooleanTopic("Turret limit switch hit").publish();

        // Control
        customAngleEntry = controlTable.getDoubleTopic("Custom turret angle").getEntry(180);

        turretMotor = new SparkMax(TurretConstants.turretMotorID, MotorType.kBrushless);
        turretController = turretMotor.getClosedLoopController();
        SparkBaseConfig turretConfig = new SparkMaxConfig().smartCurrentLimit(15, 15);

        turretConfig.idleMode(IdleMode.kCoast); 
        turretConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .allowedClosedLoopError(TurretConstants.turretAllowedClosedLoopError, ClosedLoopSlot.kSlot0)
            .p(TurretConstants.kPturret)
            .feedForward.kS(TurretConstants.kSturret);

        turretConfig.inverted(true);
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretZeroLimitSwitch = new DigitalInput(TurretConstants.turretLimitSwitchID);
        
        // turretPidController = new PIDController(TurretConstants.kPturretSlack, TurretConstants.kIturretSlack, 0);
        // turretFeedforward = new SimpleMotorFeedforward(TurretConstants.turretFakeFeedForward, 0);
    }

    public void runTurret(double speed) {
        // if limit switch is pressed, and going same direction as limit switch, STOP
        // if (turretZeroLimitSwitch.get() && speed < 0) {
        //     speed = 0;
        // }
        if (isZeroed) {
            if (speed > 0 && getAngleDegrees() > maxAngle) {
                speed = 0;
            } else if (speed < 0 && getAngleDegrees() < minAngle) {
                speed = 0;
            }
        }
        turretMotor.set(speed);
    }

    public double getAngleDegrees() {
        return Units.rotationsToDegrees(turretMotor.getEncoder().getPosition()*TurretConstants.motorPositionToTurretPositionRatio);
    }

    public double getVelocity() {
        return turretMotor.getEncoder().getVelocity();
    }
    
    public boolean didReachAngle() {
        return Math.abs(wantedTurretAngle - getAngleDegrees()) <= 0.1;
    }

    public void resetTurret() {
        turretMotor.getEncoder().setPosition(Units.degreesToRotations(180)/TurretConstants.motorPositionToTurretPositionRatio);  // Max angle is 335 and min is 55
    }

    public void setTurretAngle(double wantedTurretAngleInDegrees) {
        double limitToRange = wantedTurretAngleInDegrees % 360;
        if (wantedTurretAngleInDegrees <= minAngle) { // 25
            
            limitToRange = minAngle;
        }
        if (wantedTurretAngleInDegrees >= maxAngle) { // 280
            limitToRange = maxAngle;
        }
        // double fakeFeedForward = SmartDashboard.getNumber("Turret static friction", TurretConstants.turretFakeFeedForward);
        // double springResistance = SmartDashboard.getNumber("Turret spring resistance", TurretConstants.turretSpringResistance);
        // SmartDashboard.putNumber("Turret static friction", fakeFeedForward);
        // SmartDashboard.putNumber("Turret spring resistance", springResistance);

        double currentAngle = getAngleDegrees();
        // within slack range
        double maxSlack = 215;
        double minSlack = 110;
        boolean withinSlackRange = currentAngle <= maxSlack && currentAngle >= minSlack; 
        wantedAnglePub.set(limitToRange);
        wantedTurretAngle = limitToRange;
        // double pidVal = turretPidController.calculate(getAngleDegrees(), limitToRange);
        // double feedforward = turretFeedforward.calculate(turretMotor.getEncoder().getVelocity());
        // double diff = wantedTurretAngleInDegrees - currentAngle;
        // double direction = diff != 0 ? diff/Math.abs(diff) : 1;
        // if (direction != lastDirection) {
        //     // turretSlackPidController.reset();
        //     lastDirection = direction;
        // }
        // System.out.println(getAngleDegrees());
        // System.out.println(limitToRange);
        // System.out.println(pidVal + direction*fakeFeedForward);
        // double springFeedForward = 0;
        // if (!withinSlackRange) {
        //     // within lower spring area and moving into spring
        //     // if (currentAngle <= minSlack) {
        //     if (currentAngle <= minSlack && direction < 1) {
        //         springFeedForward = direction*springResistance;
        //     } else if (currentAngle >= maxSlack && direction > 1) {
        //     // } else if (currentAngle >= maxSlack) {
        //     // within upper spring area and moving into spring
        //         springFeedForward = direction*springResistance;
        //     }
        // }
        // System.out.println(" " + pidVal + isZeroed);
        // Math.
        // double voltage = pidVal + feedforward;
        // double voltage = feedforward;
        // if (voltage < 0) {
        //     voltage = Math.max(voltage, -3);
        // } else if (voltage > 0) {
        //     voltage = Math.min(voltage, 3);
        // }
        if (isZeroed) {
            turretController.setSetpoint(Units.degreesToRotations(wantedTurretAngle)/TurretConstants.motorPositionToTurretPositionRatio, ControlType.kPosition);
            // turretMotor.setVoltage(voltage);
        } else {
            turretMotor.set(0);
        }
    }

    public void setState(double state) {
        if (this.state != state) {
            this.state = state;
        } else {
            this.state = ShooterStates.NONE;
        }
    }
    
    public void setBrakeMode(IdleMode mode) {
        SparkBaseConfig turretConfig = new SparkMaxConfig().smartCurrentLimit(15, 15);
        turretConfig.idleMode(mode);
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void enableZero() {
        isZeroed = false;
    }

    public boolean isLimitSwitchHit() {
        return !turretZeroLimitSwitch.get();
    }
    public double getCustomAngle() {
        return customAngleEntry.get();
    }

    @Override
    public void periodic() {
        if (isLimitSwitchHit() && !isZeroed) {
            resetTurret();
            isZeroed = true;
        }
        // blink limelight if turret not zeroed to indicate to setup
        if (!isZeroed) {
            LimelightHelpers.setLEDMode_ForceOn(LimelightConstants.LIMELIGHT_FORWARD);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(LimelightConstants.LIMELIGHT_FORWARD);
        }
        if (state != ShooterStates.NONE) {
            setTurretAngle(RobotContainer.swerveSubsystem.getRobotToTargetAngle());
        }
        turretEncoderPub.set(getAngleDegrees());
        limitSwitchPub.set(isLimitSwitchHit());
    }
}
