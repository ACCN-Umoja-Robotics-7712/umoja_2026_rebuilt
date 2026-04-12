// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfig.Presets;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeRollerStates;
import frc.robot.Constants.TurretConstants;



public class IntakeRollerSubsystem extends SubsystemBase {
    private final SparkFlex intakeRollerMotor;
    private final SparkClosedLoopController pidController;
    // private final PIDController intakeRollerPidController;

    private double state = IntakeRollerStates.NONE;

    private double lastKP = 0.0;
    private double lastKV = 0.0;
    
    public IntakeRollerSubsystem() {
        intakeRollerMotor = new SparkFlex(IntakeConstants.rollerMotorID, MotorType.kBrushless);
        pidController = intakeRollerMotor.getClosedLoopController();
        
        SparkBaseConfig intakeRollerConfig = new SparkFlexConfig().smartCurrentLimit(40, 20); // Was 40
        intakeRollerConfig.idleMode(IdleMode.kCoast);
        intakeRollerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(IntakeConstants.rollerkP)
            .outputRange(-1, 1)
            .feedForward.kV(TurretConstants.kVfly);

        intakeRollerMotor.configure(intakeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // intakeRollerPidController = new PIDController(IntakeConstants.rollerkP, 0, 0);
    }

   public void setState(double rollerState) {
        if (this.state != rollerState) {
            // intakeRollerPidController.reset();
            this.state = rollerState;
        }
    }
    public boolean didReachState() {
        return false;
        // return intakeRollerPidController.atSetpoint();
    }

    public void runIntake(double speed) {
        SmartDashboard.putNumber("intake speed", speed);
        intakeRollerMotor.set(speed);
    }

    public void setIntakeVoltage(double voltage) {
        intakeRollerMotor.setVoltage(voltage);
    }

    public void setIntakeSpeed(double wantedSpeed) {
        pidController.setSetpoint(wantedSpeed, ControlType.kVoltage);
        // intakeRollerMotor.set(intakeRollerPidController.calculate(intakeRollerMotor.getEncoder().getVelocity(), wantedSpeed));
    }
    
    @Override
    public void periodic() {
        if (state == IntakeRollerStates.NONE) {
        } else {
            setIntakeSpeed(state);
        }
        
        double kPIntake = SmartDashboard.getNumber("kP Intake", IntakeConstants.rollerkP);
        double kVIntake = SmartDashboard.getNumber("kV Intake", IntakeConstants.rollerkV);

        
        SmartDashboard.putNumber("kP Intake", kPIntake);
        SmartDashboard.putNumber("kV Intake", kVIntake);

        if (kPIntake != lastKP || kVIntake != lastKV) {
            SparkBaseConfig intakeRollerConfig = new SparkFlexConfig().smartCurrentLimit(40, 20); // Was 40
            intakeRollerConfig.idleMode(IdleMode.kCoast);
            intakeRollerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(kPIntake)
                .outputRange(-1, 1)
                .feedForward.kV(kVIntake);

            intakeRollerMotor.configure(intakeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        SmartDashboard.putNumber("Intake Velocity", intakeRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake Current", intakeRollerMotor.getOutputCurrent());
    }
}
