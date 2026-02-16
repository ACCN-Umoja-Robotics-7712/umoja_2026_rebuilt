// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig.Presets;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeRollerStates;



public class IntakeRollerSubsystem extends SubsystemBase {
    private final SparkFlex intakeRollerMotor;
    private final PIDController intakeRollerPidController;

    private double state = IntakeRollerStates.NONE;
    


    public IntakeRollerSubsystem() {
        intakeRollerMotor = new SparkFlex(IntakeConstants.rollerMotorID, MotorType.kBrushless);
        
        SparkBaseConfig intakeRollerConfig = Presets.REV_Vortex;
        intakeRollerMotor.configure(intakeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeRollerPidController = new PIDController(IntakeConstants.rollerkP, 0, 0);
    }

   public void setState(double rollerState) {
        if (this.state != rollerState) {
            intakeRollerPidController.reset();
            this.state = rollerState;
        }
    }
    public boolean didReachState() {
        return intakeRollerPidController.atSetpoint();
    }

    public void runIntake(double speed) {
        SmartDashboard.putNumber("intake speed", speed);
        intakeRollerMotor.set(speed);
    }

    public void setIntakeSpeed(double wantedSpeed) {
        intakeRollerMotor.set(intakeRollerPidController.calculate(intakeRollerMotor.getEncoder().getVelocity(), wantedSpeed));
    }
    
    @Override
    public void periodic() {
        if (state == IntakeRollerStates.NONE) {
        } else {
            setIntakeSpeed(state);
        }
    }
}
