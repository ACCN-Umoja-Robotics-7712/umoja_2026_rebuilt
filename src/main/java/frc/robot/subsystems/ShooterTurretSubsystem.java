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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig.Presets;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterStates;

public class ShooterTurretSubsystem extends SubsystemBase {
    private final SparkMax turretMotor;
    private final DigitalInput turretZeroLimitSwitch;

    private final PIDController turretPidController;
    private final SimpleMotorFeedforward turretFeedforward;

    private double state = ShooterStates.NONE;
    private boolean isZeroed = false;

    private double minAngle = 30;
    private double maxAngle = 280;
    private double wantedTurretAngle = 180;
    private double lastDirection = 0;

    public ShooterTurretSubsystem() {
        turretMotor = new SparkMax(TurretConstants.turretMotorID, MotorType.kBrushless);
        SparkBaseConfig turretConfig = new SparkMaxConfig().smartCurrentLimit(15);
        turretConfig.inverted(true);
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretZeroLimitSwitch = new DigitalInput(TurretConstants.turretLimitSwitchID);
        
        turretPidController = new PIDController(TurretConstants.kPturretSlack, TurretConstants.kIturretSlack, 0);
        turretFeedforward = new SimpleMotorFeedforward(TurretConstants.turretFakeFeedForward, 0);
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
        return Units.rotationsToDegrees(turretMotor.getEncoder().getPosition()*TurretConstants.motorToTurretRatio)+180;
    }

    public double getVelocity() {
        return turretMotor.getEncoder().getVelocity();
    }
    
    public boolean didReachAngle() {
        return Math.abs(wantedTurretAngle - getAngleDegrees()) <= 1;
    }

    public void resetTurret() {
        turretMotor.getEncoder().setPosition(0);  // Max angle is 335 and min is 55
    }

    public void setTurretAngle(double wantedTurretAngleInDegrees) {
        double limitToRange = wantedTurretAngleInDegrees % 360;
        if (wantedTurretAngleInDegrees <= minAngle) { // 25
            limitToRange = minAngle;
        }
        if (wantedTurretAngleInDegrees >= maxAngle) { // 280
            limitToRange = maxAngle;
        }
        double fakeFeedForward = SmartDashboard.getNumber("Turret static friction", TurretConstants.turretFakeFeedForward);
        double springResistance = SmartDashboard.getNumber("Turret spring resistance", TurretConstants.turretSpringResistance);
        SmartDashboard.putNumber("Turret static friction", fakeFeedForward);
        SmartDashboard.putNumber("Turret spring resistance", springResistance);

        double currentAngle = getAngleDegrees();
        // within slack range
        double maxSlack = 215;
        double minSlack = 110;
        boolean withinSlackRange = currentAngle <= maxSlack && currentAngle >= minSlack; 
        SmartDashboard.putNumber("new angle that we want", limitToRange);
        wantedTurretAngle = limitToRange;
        double pidVal = turretPidController.calculate(getAngleDegrees(), limitToRange);
        double feedforward = turretFeedforward.calculate(turretMotor.getEncoder().getVelocity());
        double diff = wantedTurretAngleInDegrees - currentAngle;
        double direction = diff != 0 ? diff/Math.abs(diff) : 1;
        if (direction != lastDirection) {
            // turretSlackPidController.reset();
            lastDirection = direction;
        }
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
        double voltage = feedforward;
        if (voltage < 0) {
            voltage = Math.max(voltage, -3);
        } else if (voltage > 0) {
            voltage = Math.min(voltage, 3);
        }
        if (isZeroed) {
            turretMotor.setVoltage(voltage);
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
        SparkBaseConfig turretConfig = new SparkMaxConfig().smartCurrentLimit(15);
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
        double angle = SmartDashboard.getNumber("Custom turret angle", 180);
        SmartDashboard.putNumber("Custom turret angle", angle);
        return angle;
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
        SmartDashboard.putNumber("turret encoder", getAngleDegrees());
        SmartDashboard.putBoolean("turret limit switch hit", isLimitSwitchHit());
        
        // TODO: REMOVE FOR COMP
        double kPturretSlack = SmartDashboard.getNumber("kP Turret slack", TurretConstants.kPturretSlack);
        double kIturretSlack = SmartDashboard.getNumber("kI Turret slack", TurretConstants.kIturretSlack);

        SmartDashboard.putNumber("kP Turret slack", kPturretSlack);

        SmartDashboard.putNumber("kI Turret slack", kIturretSlack);

        boolean kPSlack = SmartDashboard.getNumber("kP Turret slack", TurretConstants.kPturretSlack) != turretPidController.getP();
        boolean kISlack = SmartDashboard.getNumber("kI Turret slack", TurretConstants.kIturretSlack) != turretPidController.getI();

        if (kPSlack || kISlack) {
            turretPidController.setP(kPturretSlack);
            turretPidController.setI(kIturretSlack);
            System.out.println("Updated turret PID and FF values: kP slack = " + kPturretSlack + "kI slack " + kIturretSlack + kPSlack + kISlack + turretPidController.getP());
        }
    }
}
