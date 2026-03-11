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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    private final PIDController turretSlackPidController;
    private final PIDController turretSpringPidController;

    private double state = ShooterStates.NONE;
    private boolean isZeroed = false;

    private double minAngle = 55;
    private double maxAngle = 335;
    private double wantedTurretAngle = 180;
    private double lastDirection = 0;
    private PIDController lastPIController = null;

    public ShooterTurretSubsystem() {
        turretMotor = new SparkMax(TurretConstants.turretMotorID, MotorType.kBrushless);
        SparkBaseConfig turretConfig = new SparkMaxConfig().smartCurrentLimit(15);
        turretConfig.inverted(true);
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretZeroLimitSwitch = new DigitalInput(TurretConstants.turretLimitSwitchID);
        
        turretSlackPidController = new PIDController(TurretConstants.kPturretSlack, TurretConstants.kIturretSlack, 0);
        turretSpringPidController = new PIDController(TurretConstants.kPturretSpring, TurretConstants.kIturretSpring, 0);
    }

    public void runTurret(double speed) {
        // if limit switch is pressed, and going same direction as limit switch, STOP
        // if (turretZeroLimitSwitch.get() && speed < 0) {
        //     speed = 0;
        // }
        
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
        if (wantedTurretAngleInDegrees <= minAngle) { // 55
            limitToRange = minAngle;
        }
        if (wantedTurretAngleInDegrees >= maxAngle) { // 335
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
        double pidVal = turretSlackPidController.calculate(getAngleDegrees(), limitToRange);
        double diff = wantedTurretAngleInDegrees - currentAngle;
        double direction = diff != 0 ? diff/Math.abs(diff) : 1;
        if (direction != lastDirection) {
            turretSlackPidController.reset();
            lastDirection = direction;
        }
        // System.out.println(getAngleDegrees());
        // System.out.println(limitToRange);
        // System.out.println(pidVal + direction*fakeFeedForward);
        double springFeedForward = 0;
        if (!withinSlackRange) {
            // within lower spring area and moving into spring
            // if (currentAngle <= minSlack) {
            if (currentAngle <= minSlack && direction < 1) {
                springFeedForward = direction*springResistance;
            } else if (currentAngle >= maxSlack && direction > 1) {
            // } else if (currentAngle >= maxSlack) {
            // within upper spring area and moving into spring
                springFeedForward = direction*springResistance;
            }
        }
        // System.out.println(" " + pidVal + isZeroed);
        if (isZeroed) {
            turretMotor.setVoltage(pidVal + direction*fakeFeedForward);
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
            LimelightHelpers.setLEDMode_ForceBlink(LimelightConstants.LIMELIGHT_FORWARD);
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
        double kPturretSpring = SmartDashboard.getNumber("kP Turret spring", TurretConstants.kPturretSpring);
        double kIturretSlack = SmartDashboard.getNumber("kI Turret slack", TurretConstants.kIturretSlack);
        double kIturretSpring = SmartDashboard.getNumber("kI Turret spring", TurretConstants.kIturretSpring);

        SmartDashboard.putNumber("kP Turret slack", kPturretSlack);
        SmartDashboard.putNumber("kP Turret spring", kPturretSpring);

        SmartDashboard.putNumber("kI Turret slack", kIturretSlack);
        SmartDashboard.putNumber("kI Turret spring", kIturretSpring);

        boolean kPSlack = SmartDashboard.getNumber("kP Turret slack", TurretConstants.kPturretSlack) != turretSlackPidController.getP();
        boolean kPSpring = SmartDashboard.getNumber("kP Turret spring", TurretConstants.kPturretSpring) != turretSpringPidController.getP();
        boolean kISlack = SmartDashboard.getNumber("kI Turret slack", TurretConstants.kIturretSlack) != turretSlackPidController.getI();
        boolean kISpring = SmartDashboard.getNumber("kI Turret spring", TurretConstants.kIturretSlack) != turretSpringPidController.getI();

        if (kPSlack || kPSpring || kISlack || kISpring) {
            turretSlackPidController.setP(kPturretSlack);
            turretSpringPidController.setP(kPturretSpring);
            turretSlackPidController.setI(kIturretSlack);
            turretSpringPidController.setI(kIturretSpring);
            System.out.println("Updated turret PID and FF values: kP slack = " + kPturretSlack + "kP spring = " + kPturretSpring + "kI slack " + kIturretSlack + "kI spring = " + kIturretSpring + kPSlack + kPSpring + kISlack + kISpring + turretSpringPidController.getP());
        }
    }
}
