// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterFlywheelSubsystem extends SubsystemBase {
    private final TalonFX flywheelMotor;

    private final PIDController flywheelPidController;
    private final SimpleMotorFeedforward FFController;

    private final VoltageOut voltageReg;

    private final SysIdRoutine sysIdRoutine;

    public ShooterFlywheelSubsystem() {
        CANBus CANivore = new CANBus("CANivore");
        flywheelMotor = new TalonFX(0, CANivore);

        flywheelPidController = new PIDController(0, 0, 0);
        FFController = new SimpleMotorFeedforward(0, 0,0);
        
        voltageReg = new VoltageOut(0.0);
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> flywheelMotor.setControl(voltageReg.withOutput(volts.in(Volts))),
                null,
                this
            )
        );
    }

    public void runShooter(double speed) {
        flywheelMotor.set(speed);
    }

    public void setShooterVelocity(double wantedVelocity) {
        double feedforward = FFController.calculate(wantedVelocity);
        double pid = flywheelPidController.calculate(flywheelMotor.getVelocity().getValueAsDouble(), wantedVelocity);
        flywheelMotor.setVoltage(feedforward + pid);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
    
    @Override
    public void periodic() {
    }
}
