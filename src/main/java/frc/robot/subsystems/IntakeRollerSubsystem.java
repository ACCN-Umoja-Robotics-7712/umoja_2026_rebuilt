// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DriverStation;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeRollerStates;
import frc.robot.Constants.TurretConstants;



public class IntakeRollerSubsystem extends SubsystemBase {
        // NetworkTables
    private NetworkTable intakeRollerTable;
    private NetworkTable telemetryTable;
    private NetworkTable pidTable;
    private NetworkTable controlTable;

    // Telemetry publishers
    private DoublePublisher intakeSpeedPub;
    private DoublePublisher intakeVelocityPub;
    private DoublePublisher intakeCurrentPub;

    // PID entries
    private DoubleEntry kPEntry;
    private DoubleEntry kDEntry;
    private DoubleEntry kVEntry;

    // Control entries
    private DoubleEntry wantedRPMEntry;

    private final SparkFlex intakeRollerMotor;
    private final SparkClosedLoopController pidController;
    // private final PIDController intakeRollerPidController;

    private double state = IntakeRollerStates.NONE;

    private double lastKP = 0.0;
    private double lastKV = 0.0;
    private double lastKD = 0.0;
    
    public IntakeRollerSubsystem() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        intakeRollerTable = nt.getTable("IntakeRoller");

        telemetryTable = intakeRollerTable.getSubTable("Telemetry");
        pidTable = intakeRollerTable.getSubTable("PID");
        controlTable = intakeRollerTable.getSubTable("Control");

        // Telemetry
        intakeSpeedPub = telemetryTable.getDoubleTopic("Intake speed").publish();
        intakeVelocityPub = telemetryTable.getDoubleTopic("Intake velocity").publish();
        intakeCurrentPub = telemetryTable.getDoubleTopic("Intake current").publish();

        // PID tuning
        kPEntry = pidTable.getDoubleTopic("kP").getEntry(IntakeConstants.rollerkP);
        kDEntry = pidTable.getDoubleTopic("kD").getEntry(IntakeConstants.rollerkD);
        kVEntry = pidTable.getDoubleTopic("kV").getEntry(IntakeConstants.rollerkV);

        // Control
        wantedRPMEntry = controlTable.getDoubleTopic("Wanted RPM").getEntry(5000);

        intakeRollerMotor = new SparkFlex(IntakeConstants.rollerMotorID, MotorType.kBrushless);
        pidController = intakeRollerMotor.getClosedLoopController();
        
        SparkBaseConfig intakeRollerConfig = new SparkFlexConfig().smartCurrentLimit(60, 20); // Was 40
        intakeRollerConfig.idleMode(IdleMode.kCoast);
        intakeRollerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(IntakeConstants.rollerkP)
            .d(IntakeConstants.rollerkD)
            .outputRange(-1, 1)
            .feedForward.kV(IntakeConstants.rollerkV);

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
        intakeSpeedPub.set(speed);
        intakeRollerMotor.set(speed);
    }

    public void setIntakeVoltage(double voltage) {
        intakeRollerMotor.setVoltage(voltage);
    }

    public void setIntakeVelocity(double wantedRPM) {
        pidController.setSetpoint(wantedRPM, ControlType.kVelocity);
    }
    
    @Override
    public void periodic() {
        if (state == IntakeRollerStates.NONE) {
        } else {
            setIntakeVelocity(state);
        }

        // Only allow PID tuning when NOT connected to FMS
        if (!DriverStation.isFMSAttached()) {

            double kPIntake = kPEntry.get();
            double kDIntake = kDEntry.get();
            double kVIntake = kVEntry.get();

            if (kPIntake != lastKP || kVIntake != lastKV || kDIntake != lastKD) {

                lastKP = kPIntake;
                lastKV = kVIntake;
                lastKD = kDIntake;

                SparkBaseConfig intakeRollerConfig =
                    new SparkFlexConfig().smartCurrentLimit(60, 20);

                intakeRollerConfig.idleMode(IdleMode.kCoast);

                intakeRollerConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .p(kPIntake)
                    .d(kDIntake)
                    .outputRange(-1, 1)
                    .feedForward.kV(kVIntake);

                intakeRollerMotor.configure(
                    intakeRollerConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
                );
            }
        }

        intakeVelocityPub.set(intakeRollerMotor.getEncoder().getVelocity());
        intakeCurrentPub.set(intakeRollerMotor.getOutputCurrent());
    }
}
