package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveModule {
    public final TalonFX driveMotor;
    public final TalonFX turnMotor;
    // private final RelativeEncoder driveEncoder, turnEncoder; 
    private final PIDController turnPIDController;
    private final SimpleMotorFeedforward turnFeedforward;
    // private final PIDController drivePIDController;
    // public final CANcoder absoluteEncoder;

    // NetworkTables
    private NetworkTable moduleTable;
    private NetworkTable telemetryTable;
    private NetworkTable pidTable;
    private NetworkTable controlTable;

    // Telemetry publishers
    private DoublePublisher wantedAnglePub;
    private DoublePublisher currentSpeedPub;
    private DoublePublisher wantedSpeedPub;
    private DoublePublisher driveCurrentPub;
    private DoublePublisher driveVelocityPub;
    private DoublePublisher turnErrorPub;

    // PID entries
    private DoubleEntry turnKPEntry;
    private DoubleEntry turnKSEntry;

    // Control entries
    private DoubleEntry encoderOffsetEntry;

    PositionVoltage turnPositionVoltage = new PositionVoltage(0);
    public double absoluteEncoderDegreeOffset, lastOffset, lastPTurn, lastkSTurn;
    public final CANcoder absoluteEncoder;
    public final int absoluteEncoderID;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed, double kP, double kS){   
        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        // Each module gets its own table based on encoder CAN ID
        NetworkTable modulesTable = nt.getTable("Swerve").getSubTable("Modules");

        moduleTable = modulesTable.getSubTable("Module_" + absoluteEncoderId);

        telemetryTable = moduleTable.getSubTable("Telemetry");
        pidTable = moduleTable.getSubTable("PID");
        controlTable = moduleTable.getSubTable("Control");

        // Telemetry
        wantedAnglePub = telemetryTable.getDoubleTopic("Wanted angle").publish();
        currentSpeedPub = telemetryTable.getDoubleTopic("Current speed").publish();
        wantedSpeedPub = telemetryTable.getDoubleTopic("Wanted speed").publish();
        driveCurrentPub = telemetryTable.getDoubleTopic("Drive current").publish();
        driveVelocityPub = telemetryTable.getDoubleTopic("Drive velocity").publish();
        turnErrorPub = telemetryTable.getDoubleTopic("Turn error").publish();

        // PID
        turnKPEntry = pidTable.getDoubleTopic("kP").getEntry(kP);
        turnKSEntry = pidTable.getDoubleTopic("kS").getEntry(kS);

        // Encoder offset
        encoderOffsetEntry = controlTable.getDoubleTopic("Encoder offset").getEntry(absoluteEncoderOffset);

        CANBus CANivoreBus = new CANBus("CANivore");

        absoluteEncoder = new CANcoder(absoluteEncoderId, CANivoreBus);

        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        
        CANconfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        CANconfig.MagnetSensor.SensorDirection = isAbsoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        
        absoluteEncoder.getConfigurator().apply(CANconfig);
    
        this.absoluteEncoderID = absoluteEncoderId;
        this.absoluteEncoderDegreeOffset = absoluteEncoderOffset;
        this.lastOffset = absoluteEncoderOffset;
        this.lastPTurn = kP;
        this.lastkSTurn = kS;

        driveMotor = new TalonFX(driveMotorId, CANivoreBus);

        CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs();
        driveCurrentLimits.StatorCurrentLimit = 100; // Original 60
        driveCurrentLimits.StatorCurrentLimitEnable = true;
        driveCurrentLimits.SupplyCurrentLimit = 50; // Original 50
        driveCurrentLimits.SupplyCurrentLimitEnable = true;
        driveCurrentLimits.SupplyCurrentLowerLimit = 40; // defaults drops to lower limit after 1s

        MotorOutputConfigs driveConfigs = new MotorOutputConfigs();
        driveConfigs.Inverted = driveMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        driveConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveConfigs);
        driveMotor.getConfigurator().apply(driveCurrentLimits);

        turnMotor = new TalonFX(turnMotorId, CANivoreBus);
        
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();

        CurrentLimitsConfigs turnCurrentLimit = turnConfig.CurrentLimits;
        turnCurrentLimit.StatorCurrentLimit = 60;
        turnCurrentLimit.StatorCurrentLimitEnable = true;

        FeedbackConfigs feedbackConfigs = turnConfig.Feedback;
        feedbackConfigs.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfigs.SensorToMechanismRatio = 1/ModuleConstants.turnToTurnMotorGearRatio;
        feedbackConfigs.RotorToSensorRatio = 1.0;

        MotorOutputConfigs motorOutputConfigs = turnConfig.MotorOutput;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        
        Slot0Configs pidConfigs = turnConfig.Slot0;
        pidConfigs.kP = kP;
        pidConfigs.kS = kS;

        ClosedLoopGeneralConfigs closedLoopGeneralConfigs = turnConfig.ClosedLoopGeneral;
        closedLoopGeneralConfigs.ContinuousWrap = true;
        
        // MotionMagicConfigs motionMagicConfigs = turnConfig.MotionMagic;
        // motionMagicConfigs.MotionMagicAcceleration = 1;

        turnMotor.getConfigurator().apply(turnConfig);
        // turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);

        // SparkMaxConfig driveConfig = new SparkMaxConfig();
        // driveConfig.idleMode(IdleMode.kCoast);
        // driveConfig.inverted(driveMotorReversed);
        // driveConfig.smartCurrentLimit(30);
        
        // SparkMaxConfig turnConfig = new SparkMaxConfig();
        // turnConfig.idleMode(IdleMode.kCoast);
        // turnConfig.inverted(turnMotorReversed);
        // turnConfig.smartCurrentLimit(20);
        

        // turnMotor.configure(turnConfig, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        turnPIDController = new PIDController(kP, 0, 0);
        turnPIDController.enableContinuousInput(0, 2*Math.PI);
        turnFeedforward = new SimpleMotorFeedforward(kS, 0.0);
        // drivePIDController = new PIDController(ModuleConstants.kPDriving, 0, 0);
        
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble()*ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPositionRadians() {
        return Math.toRadians(getAbsoluteEncoderDegree());
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble()*ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningVelocity() {
        return turnMotor.getVelocity().getValueAsDouble()*ModuleConstants.kTurnEncoderRot2Rad;
    }

    public double getAbsoluteEncoderDegree() {
        // using CANCoder so get direct value
        return (Units.rotationsToDegrees(absoluteEncoder.getAbsolutePosition().getValueAsDouble()) + 180) % 360;
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        resetTurn();
    }

    public void resetTurn(){
        double position = getAbsoluteEncoderDegree();
        turnMotor.setPosition(position * ModuleConstants.turnToTurnMotorGearRatio);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPositionRadians()));
    }

    public void setDesiredState(SwerveModuleState state) {
        boolean isXLocked = state.angle.getDegrees() == -135 || state.angle.getDegrees() == -45 || state.angle.getDegrees() == 45 || state.angle.getDegrees() == 135; 
        if (Math.abs(state.speedMetersPerSecond) < 0.001 && !isXLocked) {
            stop();
            return;
        }

        //Just in case -LEWI// private boolean isXLockAngle(double degrees) {
                        //     double[] xLockAngles = {45, -45, 135, -135};
                        //     for (double target : xLockAngles) {
                        //         if (Math.abs(degrees - target) < 2.0) { // 2-degree tolerance
                        //             return true;
                        //         }
                        //     }
                        //     return false;
                        // }

        
        state.optimize(getState().angle);

        // Only allow tuning and offset changes when NOT connected to FMS
        if (!DriverStation.isFMSAttached()) {

            // Encoder offset
            double CANCoderOffset = encoderOffsetEntry.get();

            if (lastOffset != CANCoderOffset) {
                lastOffset = CANCoderOffset;

                CANcoderConfiguration CANconfig = new CANcoderConfiguration();

                CANconfig.MagnetSensor.MagnetOffset = CANCoderOffset;
                CANconfig.MagnetSensor.SensorDirection =
                    SensorDirectionValue.CounterClockwise_Positive;

                absoluteEncoder.getConfigurator().apply(CANconfig);
            }

            // Turn PID tuning
            double turnP = turnKPEntry.get();
            double turnS = turnKSEntry.get();

            if (lastPTurn != turnP || lastkSTurn != turnS) {

                Slot0Configs pidConfigs = new Slot0Configs()
                    .withKP(turnP)
                    .withKS(turnS);

                turnMotor.getConfigurator().apply(pidConfigs);

                turnPIDController.setP(turnP);
                turnFeedforward.setKs(turnS);

                lastPTurn = turnP;
                lastkSTurn = turnS;

                System.out.println(
                    "Updated PID for module " + absoluteEncoderID
                    + ": kP=" + turnP + ", kS=" + turnS
                );
            }
        }

        double wantedAngleDegrees = (state.angle.getDegrees() + 360) % 360;
        wantedAnglePub.set(wantedAngleDegrees);
        currentSpeedPub.set(getState().speedMetersPerSecond);
        wantedSpeedPub.set(state.speedMetersPerSecond);
        driveCurrentPub.set(driveMotor.getStatorCurrent().getValueAsDouble());
        driveVelocityPub.set(driveMotor.getVelocity().getValueAsDouble());
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // driveMotor.set(0);
        // driveMotor.set(drivePIDController.calculate(getState().speedMetersPerSecond, state.speedMetersPerSecond));
        // TODO: CHANGE THIS TO PID

        double error = getTurningPositionRadians() - Math.toRadians(wantedAngleDegrees);
        // wrap error to shortest
        if (error > Math.PI) {
            error -= 2*Math.PI;
        } else if (error < -Math.PI) {
            error += 2*Math.PI;
        }
        turnErrorPub.set(error);
        // double wantedMotorPosition = state.angle.getRotations();
        // turnMotor.setControl(turnPositionVoltage.withPosition(wantedMotorPosition));
        turnMotor.setVoltage(turnPIDController.calculate(0, error) + turnFeedforward.calculate(error));
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivePosition(),
            Rotation2d.fromRadians(getTurningPositionRadians()));
      }
}