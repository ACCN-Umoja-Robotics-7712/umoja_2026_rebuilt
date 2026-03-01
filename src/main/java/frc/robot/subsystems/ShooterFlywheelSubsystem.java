// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkFlexConfig.Presets;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.ShooterTurretAngleCommand;
import frc.robot.Constants.ShooterStates;

public class ShooterFlywheelSubsystem extends SubsystemBase {
    private final SparkFlex flywheelMotorLeader;
    private final SparkFlex flywheelMotorFollower;

    private final PIDController flywheelPidController;
    private final SimpleMotorFeedforward FFController;

    private double state = ShooterStates.NONE;

    public ShooterFlywheelSubsystem() {
        flywheelMotorLeader = new SparkFlex(TurretConstants.flywheelMotorLeaderID, MotorType.kBrushless);
        flywheelMotorFollower = new SparkFlex(TurretConstants.flywheelMotorFollowerID, MotorType.kBrushless);

        SparkBaseConfig leaderConfig = new SparkFlexConfig().smartCurrentLimit(40);
        flywheelMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkBaseConfig followerConfig = new SparkFlexConfig().smartCurrentLimit(40);
        followerConfig.follow(TurretConstants.flywheelMotorLeaderID, true); // follower is opposite of leader
        flywheelMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelPidController = new PIDController(TurretConstants.kPfly, TurretConstants.kIfly, 0);
        FFController = new SimpleMotorFeedforward(TurretConstants.kSfly, TurretConstants.kVfly,0);
        
        flywheelPidController.setTolerance(5, 5); // 5 RPM tolerance and 5 RPM/s velocity tolerance
    }
    public void runShooter(double speed) {
        flywheelMotorLeader.set(speed);
    }

    public void setShooterVelocity(double wantedVelocity) {
        double feedforward = FFController.calculate(wantedVelocity);
        double pid = flywheelPidController.calculate(flywheelMotorLeader.getEncoder().getVelocity(), wantedVelocity);
        flywheelMotorLeader.setVoltage(feedforward + pid);
    }
    
    public void setShooterSpeed(double wantedRPM) {
        flywheelMotorLeader.set(flywheelPidController.calculate(flywheelMotorLeader.getEncoder().getVelocity(), wantedRPM));
    }

    public void setState(double shooterState) {
        if (this.state != shooterState) {
            this.state = shooterState;
        }
    }

      public boolean didReachVelocity() {
        return flywheelPidController.atSetpoint();
    }
    
    @Override
    public void periodic() {
        if (state == ShooterStates.NONE) {
        } else {
            setShooterVelocity(state);
        }

        SmartDashboard.putNumber("Flywheel Velocity", flywheelMotorLeader.getEncoder().getVelocity());
    }
}
