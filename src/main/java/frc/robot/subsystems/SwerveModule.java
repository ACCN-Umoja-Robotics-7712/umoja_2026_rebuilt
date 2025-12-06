package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    public final SparkMax driveMotor, turnMotor;
    // private final RelativeEncoder driveEncoder, turnEncoder;
    private final PIDController turnPIDController;
    // public final CANcoder absoluteEncoder;
    public double absoluteEncoderDegreeOffset;
    public final CANcoder absoluteEncoder;
    public final int absoluteEncoderID;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed){   
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        
        CANconfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        CANconfig.MagnetSensor.SensorDirection = isAbsoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        // config.MagnetSensor.AbsoluteSensorRangeValue = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
        this.absoluteEncoderID = absoluteEncoderId;
        this.absoluteEncoderDegreeOffset = absoluteEncoderOffset;

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kCoast);
        driveConfig.inverted(driveMotorReversed);
        driveConfig.smartCurrentLimit(30);
        
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.idleMode(IdleMode.kCoast);
        turnConfig.inverted(turnMotorReversed);
        turnConfig.smartCurrentLimit(20);

        driveMotor.configure(driveConfig, null, null);
        turnMotor.configure(turnConfig, null, null);        

        turnPIDController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition()*ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return Math.toRadians(getAbsoluteEncoderDegree());
    }

    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity()*ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningVelocity() {
        return turnMotor.getEncoder().getVelocity()*ModuleConstants.kTurnEncoderRot2Rad;
    }

    public double getAbsoluteEncoderDegree() {
        // subtract offset to zero wheels
        double correctZeroEncoder = absoluteEncoder.getPosition().getValueAsDouble()*360 - absoluteEncoderDegreeOffset;
        // reverse direction so counter clockwise positive
        double correctDirectionEncoder = correctZeroEncoder * -1;
        // add and mod to switch from negative to positive
        return ((correctDirectionEncoder + 360) % 360);   
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        resetTurn();
    }

    public void resetTurn(){
        double position = getAbsoluteEncoderDegree();
        turnMotor.getEncoder().setPosition(position);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state.optimize(getState().angle);
        SmartDashboard.putNumber("Swerve current speed" + absoluteEncoderID, getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve wanted speed" + absoluteEncoderID, state.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve current" + absoluteEncoderID, driveMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve velocity" + absoluteEncoderID, driveMotor.getEncoder().getVelocity());
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // driveMotor.set(drivePIDcontroller.calculate())
        // TODO: CHANGE THIS TO PID

        //fill canbus
        // SmartDashboard.putNumber("ID (DRIVE) "+absoluteEncoderID + " TEMP: ", driveMotor.getMotorTemperature());
        // SmartDashboard.putNumber("ID (TURN) "+absoluteEncoderID + " TEMP: ", turnMotor.getMotorTemperature());

        // SmartDashboard.putNumber("ID: " + absoluteEncoderID, Math.toDegrees(getTurningPosition()));
        // SmartDashboard.putNumber("GOAL: " + absoluteEncoderID, Math.toDegrees(state.angle.getRadians()));
        // SmartDashboard.putNumber("Set motor percent: " + absoluteEncoderID, turnPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        
        turnMotor.set(turnPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        //turnMotor.set(turnPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(),
            Rotation2d.fromRadians(getTurningPosition()));
      }
}