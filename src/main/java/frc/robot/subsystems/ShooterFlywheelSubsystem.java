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
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterStates;

public class ShooterFlywheelSubsystem extends SubsystemBase {
    private final SparkFlex flywheelMotorLeader;
    private final SparkFlex flywheelMotorFollower;
    private SparkFlex kickerMotor;

    private final PIDController flywheelPidController;
    private final SimpleMotorFeedforward flywheelFFController;
    private final PIDController kickerPidController;
    private final SimpleMotorFeedforward kickerFFController;

    private double state = ShooterStates.NONE;
    private double wantedShooterRPM = 0;
    private double wantedKickerRPM = 0;

    public ShooterFlywheelSubsystem() {
        flywheelMotorLeader = new SparkFlex(TurretConstants.flywheelMotorLeaderID, MotorType.kBrushless);
        flywheelMotorFollower = new SparkFlex(TurretConstants.flywheelMotorFollowerID, MotorType.kBrushless);

        SparkBaseConfig leaderConfig = new SparkFlexConfig().smartCurrentLimit(40);
        flywheelMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkBaseConfig followerConfig = new SparkFlexConfig().smartCurrentLimit(40);
        followerConfig.follow(TurretConstants.flywheelMotorLeaderID, true); // follower is opposite of leader
        flywheelMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelPidController = new PIDController(TurretConstants.kPfly, TurretConstants.kIfly, TurretConstants.kDfly);
        flywheelFFController = new SimpleMotorFeedforward(TurretConstants.kSfly, TurretConstants.kVfly,0);
        kickerPidController = new PIDController(TurretConstants.kPkicker, TurretConstants.kIkicker, 0);
        kickerFFController = new SimpleMotorFeedforward(TurretConstants.kSkicker, TurretConstants.kVkicker,0);
        
        kickerMotor = new SparkFlex(IndexerConstants.kickerMotorID, MotorType.kBrushless);
        SparkBaseConfig kickerConfig = new SparkFlexConfig().smartCurrentLimit(40); // NEO_Vortex (Current Limit is 80) 
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        flywheelPidController.setTolerance(50, 50); // 50 RPM tolerance and 50 RPM/s velocity tolerance
        kickerPidController.setTolerance(50, 50); // 50 RPM tolerance and 50 RPM/s velocity tolerance
    }
    
    public void runShooter(double speed) {
        flywheelMotorLeader.set(speed);
        kickerMotor.set(Math.min(speed*1.5, 0.75));
    }

    public void setShooterVelocity(double wantedVelocity) {
        double feedforwardFly = flywheelFFController.calculate(wantedVelocity);
        double pidFly = flywheelPidController.calculate(flywheelMotorLeader.getEncoder().getVelocity(), wantedVelocity);
        flywheelMotorLeader.setVoltage(feedforwardFly + pidFly);

        // kicker
        double wantedKickerVelocity = wantedVelocity*-1.5;
        wantedKickerVelocity = Math.min(wantedKickerVelocity, 5000); // limit to 5000 RPM to prevent the motor from burning out

        double feedforwardKicker = kickerFFController.calculate(wantedKickerVelocity);
        double pidKicker = kickerPidController.calculate(kickerMotor.getEncoder().getVelocity(), wantedKickerVelocity);
        kickerMotor.setVoltage(feedforwardKicker + pidKicker);
        
        wantedShooterRPM = wantedVelocity;
        wantedKickerRPM = wantedKickerVelocity;
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
        // return flywheelPidController.atSetpoint() && kickerPidController.atSetpoint();
        boolean kickerAtRPM = Math.abs(kickerMotor.getEncoder().getVelocity()) - wantedKickerRPM <= 50;
        boolean shooterAtRPM = Math.abs(flywheelMotorLeader.getEncoder().getVelocity()) - wantedShooterRPM <= 50;
        return kickerAtRPM && shooterAtRPM;
    }

    public double getDashboardVelocity() {
        double velocity = SmartDashboard.getNumber("Custom flywheel velocity", -3000);
        SmartDashboard.putNumber("Custom flywheel velocity", velocity);
        return velocity;
    }
    
    @Override
    public void periodic() {
        if (state == ShooterStates.NONE) {
        } else {
            setShooterVelocity(state);
        }

        SmartDashboard.putNumber("Flywheel Velocity", flywheelMotorLeader.getEncoder().getVelocity());
        SmartDashboard.putNumber("Kicker Velocity", kickerMotor.getEncoder().getVelocity());

        double kSkicker = SmartDashboard.getNumber("kS Kicker", TurretConstants.kSkicker);
        double kVkicker = SmartDashboard.getNumber("kV Kicker", TurretConstants.kVkicker);
        double kPkicker = SmartDashboard.getNumber("kP Kicker", TurretConstants.kPkicker);

        SmartDashboard.putNumber("kS Kicker", kSkicker);
        SmartDashboard.putNumber("kV Kicker", kVkicker);
        SmartDashboard.putNumber("kP Kicker", kPkicker);
        if (SmartDashboard.getNumber("kS Kicker", TurretConstants.kSkicker) != kickerFFController.getKs() || SmartDashboard.getNumber("kV Kicker", TurretConstants.kVkicker) != kickerFFController.getKv() || SmartDashboard.getNumber("kP Kicker", TurretConstants.kPkicker) != kickerPidController.getP()) {

            kickerFFController.setKs(kSkicker);
            kickerFFController.setKv(kVkicker);
            kickerPidController.setP(kPkicker);
            System.out.println("Updated Kicker PID and FF values: kS = " + kSkicker + ", kV = " + kVkicker + ", kP = " + kPkicker);
        }
    }
}
