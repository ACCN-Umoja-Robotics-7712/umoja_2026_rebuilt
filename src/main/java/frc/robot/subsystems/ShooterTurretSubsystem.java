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

public class ShooterTurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final DigitalInput turretZeroLimitSwitch;

    private final PIDController turretPidController;

    private final VoltageOut voltageReg;

    private final SysIdRoutine sysIdRoutine;

    public ShooterTurretSubsystem() {
        CANBus CANivore = new CANBus("CANivore");
        turretMotor = new TalonFX(1, CANivore);
        turretZeroLimitSwitch = new DigitalInput(0);
        
        turretPidController = new PIDController(0.01, 0, 0);
        
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
                (volts) -> turretMotor.setControl(voltageReg.withOutput(volts.in(Volts))),
                null,
                this
            )
        );
    }

    public void runTurret(double speed) {
        // if limit switch is pressed, and going same direction as limit switch, STOP
        if (turretZeroLimitSwitch.get() && speed < 0) {
            speed = 0;
        }
        
        turretMotor.set(speed);
    }

    public void setTurretAngle(double wantedTurretRotation) {
        turretMotor.set(turretPidController.calculate(turretMotor.getPosition().getValueAsDouble(), wantedTurretRotation));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
    
    @Override
    public void periodic() {
        if (turretZeroLimitSwitch.get()) {
            turretMotor.setPosition(0);
        }
    }
}
