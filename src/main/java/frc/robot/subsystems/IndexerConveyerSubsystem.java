package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;



public class IndexerConveyerSubsystem extends SubsystemBase{
    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
    private final TalonFX IndexConveyerMotor;

    public IndexerConveyerSubsystem() {
        CANBus CANivore = new CANBus("CANivore");
        IndexConveyerMotor = new TalonFX(0, CANivore);
    }

    public void runShooter(double speed) {
        IndexConveyerMotor.set(speed);
    }
    
    @Override
    public void periodic() {
    }
}
