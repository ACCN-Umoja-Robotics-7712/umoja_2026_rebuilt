// This is the subsystem for the indexer and kicker

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    
    public IndexerSubsystem() {
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
        double indexVoltage = SmartDashboard.getNumber("TEST Indexer with applied voltage", 10);
        double indexBeltVoltage = SmartDashboard.getNumber("TEST Indexer belt with applied voltage", 10);
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
        SmartDashboard.putNumber("Indexer velocity", indexerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Indexer Belt velocity", beltMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Indexer Belt current", beltMotor.getOutputCurrent());
        
        
        double kPIndex = SmartDashboard.getNumber("kP Index", IndexerConstants.kPIndex);
        double kDIndex = SmartDashboard.getNumber("kD Index", IndexerConstants.kDIndex);
        double kVIndex = SmartDashboard.getNumber("kV Index", IndexerConstants.kVIndex);

        
        SmartDashboard.putNumber("kP Index", kPIndex);
        SmartDashboard.putNumber("kD Index", kDIndex);
        SmartDashboard.putNumber("kV Index", kVIndex);

        if (kPIndex != lastKP || kDIndex != lastKD || kVIndex != lastKV) {
            lastKP = kPIndex;
            lastKD = kDIndex;
            lastKV = kVIndex;
            SparkBaseConfig indexerConfig = new SparkFlexConfig().smartCurrentLimit(55, 20); // NEO_Vortex (80 A but was 40 A)
            indexerConfig.idleMode(IdleMode.kCoast);
            indexerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(kPIndex)
                .d(kDIndex)
                .outputRange(-1, 1)
                .feedForward.kV(kVIndex);

            indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        double indexVoltage = SmartDashboard.getNumber("TEST Indexer with applied voltage", 10);
        double indexBeltVoltage = SmartDashboard.getNumber("TEST Indexer belt with applied voltage", 10);
        SmartDashboard.putNumber("TEST Indexer with applied voltage", indexVoltage);
        SmartDashboard.putNumber("TEST Indexer belt with applied voltage", indexBeltVoltage);
    }

}
