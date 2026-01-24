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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterHoodSubsystem extends SubsystemBase {
    private final TalonFX hoodMotor;
    private final DigitalInput hoodZeroLimitSwitch;

    private final PIDController hoodPidController;

    private final VoltageOut voltageReg;

    private final SysIdRoutine sysIdRoutine;

    public ShooterHoodSubsystem() {
        CANBus CANivore = new CANBus("CANivore");
        hoodMotor = new TalonFX(2, CANivore);
        hoodZeroLimitSwitch = new DigitalInput(1);

        hoodPidController = new PIDController(0.01, 0, 0);
        
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
                (volts) -> hoodMotor.setControl(voltageReg.withOutput(volts.in(Volts))),
                null,
                this
            )
        );
    }

    public void runHood(double speed) {
        // if limit switch is pressed, and going same direction as limit switch, STOP
        if (hoodZeroLimitSwitch.get() && speed < 0) {
            speed = 0;
        }

        hoodMotor.set(speed);
    }
    
    public void setHoodAngle(double wantedHoodRotation) {
        hoodMotor.set(hoodPidController.calculate(hoodMotor.getPosition().getValueAsDouble(), wantedHoodRotation));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
    
    @Override
    public void periodic() {
        if (hoodZeroLimitSwitch.get()) {
            hoodMotor.setPosition(0);
        }
    }
}
