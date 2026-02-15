// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmStates;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeRollerStates;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final TalonFX intakeRollerMotor;
    private final PIDController intakeRollerPidController;

    private double state = IntakeRollerStates.NONE;
    

    public IntakeRollerSubsystem() {
        CANBus CANivore = new CANBus("CANivore");
        intakeRollerMotor = new TalonFX(IntakeConstants.rollerMotorID, CANivore);

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
        intakeRollerMotor.set(speed);
    }

    public void setIntakeSpeed(double wantedSpeed) {
        intakeRollerMotor.set(intakeRollerPidController.calculate(intakeRollerMotor.getVelocity().getValueAsDouble(), wantedSpeed));
    }
    
    @Override
    public void periodic() {
        if (state == IntakeRollerStates.NONE) {
            runIntake(0);
        } else {
            setIntakeSpeed(state);
        }
    }
}
