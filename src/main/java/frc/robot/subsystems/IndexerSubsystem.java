// This is the subsystem for the indexer and kicker

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleEntry;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;

public class IndexerSubsystem extends SubsystemBase {
    private SparkClosedLoopController indexerPIDController;
    private SparkFlex indexerMotor;
    private SparkFlex beltMotor;

    private double lastKP = 0.0;
    private double lastKD = 0.0;
    private double lastKV = 0.0;

    // NetworkTables
    private NetworkTable indexerTable;
    private NetworkTable telemetryTable;
    private NetworkTable pidTable;
    private NetworkTable testTable;

    // Telemetry publishers
    private DoublePublisher indexerVelocityPub;
    private DoublePublisher beltVelocityPub;
    private DoublePublisher beltCurrentPub;

    // PID entries (read/write)
    private DoubleEntry kPEntry;
    private DoubleEntry kDEntry;
    private DoubleEntry kVEntry;

    // Test voltage entries
    private DoubleEntry indexerVoltageEntry;
    private DoubleEntry beltVoltageEntry;
    
    public IndexerSubsystem() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        indexerTable = nt.getTable("Indexer");

        telemetryTable = indexerTable.getSubTable("Telemetry");
        pidTable = indexerTable.getSubTable("PID");
        testTable = indexerTable.getSubTable("Test");

        // Telemetry
        indexerVelocityPub =
            telemetryTable.getDoubleTopic("Indexer velocity").publish();

        beltVelocityPub =
            telemetryTable.getDoubleTopic("Belt velocity").publish();

        beltCurrentPub =
            telemetryTable.getDoubleTopic("Belt current").publish();

        // PID tuning
        kPEntry = pidTable.getDoubleTopic("kP").getEntry(IndexerConstants.kPIndex);
        kDEntry = pidTable.getDoubleTopic("kD").getEntry(IndexerConstants.kDIndex);
        kVEntry = pidTable.getDoubleTopic("kV").getEntry(IndexerConstants.kVIndex);

        // Test voltages
        indexerVoltageEntry =
            testTable.getDoubleTopic("Indexer voltage").getEntry(10.0);

        beltVoltageEntry =
            testTable.getDoubleTopic("Belt voltage").getEntry(10.0);

        indexerMotor = new SparkFlex(IndexerConstants.indexerMotorLeaderID, MotorType.kBrushless);
        indexerPIDController = indexerMotor.getClosedLoopController();
        SparkBaseConfig indexerConfig = new SparkFlexConfig().smartCurrentLimit(55, 20); // NEO_Vortex (80 A but was 40 A)
        indexerConfig.idleMode(IdleMode.kCoast);
        indexerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(IndexerConstants.kPIndex)
            .d(IndexerConstants.kDIndex)
            .outputRange(-1, 1)
            .feedForward.kV(IndexerConstants.kVIndex);
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        beltMotor = new SparkFlex(IndexerConstants.beltMotorID, MotorType.kBrushless);
        SparkBaseConfig beltConfig = new SparkFlexConfig().smartCurrentLimit(35, 20); // NEO_Vortex
        beltConfig.inverted(true);
        beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIndexerAtRPMVoltage(double indexerRPM, double beltVoltage) { // Can change the speed for each motor independently
        indexerPIDController.setSetpoint(indexerRPM, ControlType.kVelocity);
        beltMotor.setVoltage(beltVoltage); // 3.0
    }

    public void runIndexerUsingTestvoltage() {
        double indexVoltage = indexerVoltageEntry.get();
        double indexBeltVoltage = beltVoltageEntry.get();

        indexerMotor.setVoltage(indexVoltage);
        beltMotor.setVoltage(indexBeltVoltage);
    }

    public void setIndexerVelocity(double indexerRPM) {
        indexerPIDController.setSetpoint(indexerRPM, ControlType.kVelocity);
    }
    
    public void stopIndexer() {
        indexerMotor.stopMotor();
        beltMotor.stopMotor();
    }

    @Override
    public void periodic() {

        // Publish telemetry
        indexerVelocityPub.set(indexerMotor.getEncoder().getVelocity());
        beltVelocityPub.set(beltMotor.getEncoder().getVelocity());
        beltCurrentPub.set(beltMotor.getOutputCurrent());
        
        // Only allow PID tuning when NOT connected to FMS
        if (!DriverStation.isFMSAttached()) {

            double kPIndex = kPEntry.get();
            double kDIndex = kDEntry.get();
            double kVIndex = kVEntry.get();

            if (kPIndex != lastKP || kDIndex != lastKD || kVIndex != lastKV) {

                lastKP = kPIndex;
                lastKD = kDIndex;
                lastKV = kVIndex;

                SparkBaseConfig indexerConfig =
                    new SparkFlexConfig().smartCurrentLimit(55, 20);

                indexerConfig.idleMode(IdleMode.kCoast);

                indexerConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .p(kPIndex)
                    .d(kDIndex)
                    .outputRange(-1, 1)
                    .feedForward.kV(kVIndex);

                indexerMotor.configure(
                    indexerConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
                );
            }
        }
    }

    public double getDashboardBeltVoltage() {
        return beltVoltageEntry.get();
    }

    public double getDashboardIndexVoltage() {
        return indexerVoltageEntry.get();
    }
}
