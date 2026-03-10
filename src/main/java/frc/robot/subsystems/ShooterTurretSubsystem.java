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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
        double currentAngle = getAngleDegrees();
        // within slack range
        boolean withinSlackRange = currentAngle <= 255 && currentAngle >= 155; 
        // PIDController pidToUse;
        // if (withinSlackRange) {
        //     pidToUse = turretSlackPidController;
        // } else {
        //     pidToUse = turretSpringPidController;
        // }
        // // if (lastPIController != pidToUse) {
        // //     pidToUse.reset();
        // // }
        // lastPIController = pidToUse;
        SmartDashboard.putNumber("new angle that we want", limitToRange);
        wantedTurretAngle = limitToRange;
        turretMotor.setVoltage(turretSlackPidController.calculate(getAngleDegrees(), limitToRange));
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
        // if (turretZeroLimitSwitch.get() && !isZeroed) {
        //     turretMotor.getEncoder().setPosition(0);
        //     isZeroed = true;
        // }
        if (state != ShooterStates.NONE) {
            setTurretAngle(RobotContainer.swerveSubsystem.getRobotToTargetAngle());
        }
        if (isLimitSwitchHit()) {
            resetTurret();
        }
        SmartDashboard.putNumber("turret encoder", getAngleDegrees());
        SmartDashboard.putBoolean("turret limit switch hit", turretZeroLimitSwitch.get());
        
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
