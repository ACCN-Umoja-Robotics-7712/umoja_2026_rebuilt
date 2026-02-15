package frc.robot.subsystems;

import com.studica.frc.AHRS;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;


import choreo.trajectory.SwerveSample;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Colors;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.LimelightHelpers;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDegree,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightDriveReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightDriveReversed,
        DriveConstants.kBackRightTurningEncoderReversed, 
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDegree, 
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final Pigeon2 gyro = new Pigeon2(100, "CANivore");
    private TrajectoryConfig trajectoryConfig;
    public PIDController shootController;
    public PIDController xController;
    public PIDController yController;
    public ProfiledPIDController thetaController;
    public HolonomicDriveController holonomicDriveController;
    public final Timer timer = new Timer();
    private double wantedAngle = 0;
    
    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, new Rotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),  
            backLeft.getPosition(),
            backRight.getPosition()
        }, new Pose2d()
    );
    
    RobotConfig config;

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.enableLogging(true);

                zeroHeading();
                resetEncoders();
                // frontRight.driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
                // frontRight.turnMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
            }catch (Exception e) {
            }
        }).start();
    
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
            e.printStackTrace();
            System.out.println("ROBOT GAVE UP PLEASE FIX CONFIG");
            return;
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> setModuleStatesFromSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(Constants.AutoConstants.kPXController, Constants.AutoConstants.kIXController, 0), // Translation PID constants
                        new PIDConstants(Constants.AutoConstants.kPThetaController, Constants.AutoConstants.kIThetaController, 0.0) // Rotation PID constants
                ), // TODO: Auto PID
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        
        trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 3. Define PID controllers for tracking trajectory
        shootController = new PIDController(0.1, 0, 0);
        
        xController = new PIDController(DriveConstants.kPDrive, DriveConstants.kIDrive, 0);
        yController = new PIDController(DriveConstants.kPDrive, DriveConstants.kIDrive, 0);
        thetaController = new ProfiledPIDController(
                DriveConstants.kPTurning, DriveConstants.kITurning, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(0,360);
        
        holonomicDriveController = new HolonomicDriveController(xController, yController, thetaController);
    }
    // Assuming this is a method in your drive subsystem
   public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + thetaController.calculate(pose.getRotation().getDegrees(), Math.toDegrees(sample.heading))
        );

        // Apply the generated speeds
        setModuleStatesFromSpeeds(speeds);
    }

    public void resetEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void zeroHeading(){
        RobotContainer.wantedAngle = -1;
        gyro.reset();
    }

    public double getHeading(){
        double yaw = -gyro.getYaw();

        // if blue flip
        if (!DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)) {
            yaw += 180;
        }

        //Changes the -180->0 to 180->360 (0 to 180 stays the same)
        double heading = Math.IEEEremainder(yaw, 360);
        // double heading = (-gyro.getYaw() + 360)%180;
        if(heading < 0){
            heading = 180 + (180+heading);
        }
        SmartDashboard.putNumber("HEADING", heading);
        return heading;
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        System.out.println("ODOMETRY RESET");
        poseEstimator.resetPosition(Rotation2d.fromDegrees(getHeading()), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> nearestPosePublisher = NetworkTableInstance.getDefault().getStructTopic("NearestPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> nearestLeftPosePublisher = NetworkTableInstance.getDefault().getStructTopic("NearestLeftPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> nearestRightPosePublisher = NetworkTableInstance.getDefault().getStructTopic("NearestRightPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> nearestReefPublisher = NetworkTableInstance.getDefault().getStructTopic("Nearest Reef", Pose2d.struct).publish();
    StructPublisher<Pose2d> nearestStationPublisher = NetworkTableInstance.getDefault().getStructTopic("Nearest Station", Pose2d.struct).publish();


    StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<Pose2d> allPointsPublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("AllPosesArray", Pose2d.struct).publish();

    @Override
    public void periodic() {
        
        SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        
        swerveStatePublisher.set(moduleStates);
        SmartDashboard.putNumber("FL", frontLeft.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("FR", frontRight.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("BL", backLeft.getAbsoluteEncoderDegree());
        SmartDashboard.putNumber("BR", backRight.getAbsoluteEncoderDegree());

        SmartDashboard.putNumber("GAME STATE", RobotContainer.gameState);

        poseEstimator.update(Rotation2d.fromDegrees(getHeading()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
        
        String tagLeftLimelightName = Constants.LimelightConstants.tagName;
        String driverLimelightName = Constants.LimelightConstants.driverName;
        String tagRightLimelightName = Constants.LimelightConstants.gamePieceName;
        boolean hasTargetsLeft = LimelightHelpers.getTargetCount(tagLeftLimelightName) != 0;
        boolean hasTargetsRight = LimelightHelpers.getTargetCount(tagRightLimelightName) != 0;
        boolean isDisabled = RobotContainer.gameState == GameConstants.Robot;
        boolean isNonGameTeleop = RobotContainer.gameState == GameConstants.TeleOp && DriverStation.getMatchType() == MatchType.None;
        
        boolean isAuto = RobotContainer.gameState == GameConstants.Auto;
       
        LimelightHelpers.SetRobotOrientation(tagLeftLimelightName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(tagRightLimelightName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate leftMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tagLeftLimelightName);
        LimelightHelpers.PoseEstimate rightMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tagRightLimelightName);

        boolean doRejectLeftUpdate = false;
        boolean doRejectRightUpdate = false;

        if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            doRejectLeftUpdate = true;
            doRejectRightUpdate = true;
        }
        double visionTrustValue = 0.1;
        if (RobotContainer.gameState == GameConstants.Robot) {
            visionTrustValue = 0;
        }
        if (leftMT2 != null) {
            if (leftMT2.tagCount == 0)
            {
                doRejectLeftUpdate = true;
            }
            if (!doRejectLeftUpdate)
            {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustValue,visionTrustValue,9999999));
                poseEstimator.addVisionMeasurement(
                    leftMT2.pose,
                    leftMT2.timestampSeconds);
                doRejectRightUpdate = true;
            }
        }
        if (rightMT2 != null) {
            if (rightMT2.tagCount == 0)
            {
                doRejectRightUpdate = true;
            }
            if (!doRejectRightUpdate)
            {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustValue+1,visionTrustValue+1,9999999));
                poseEstimator.addVisionMeasurement(
                    rightMT2.pose,
                    rightMT2.timestampSeconds);
            }
        }
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setModuleStatesFromSpeeds(ChassisSpeeds speeds){
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    public Pose2d offsetPoint(Pose2d pose, double sideOffset) {
        Transform2d transform = new Transform2d(0, sideOffset, new Rotation2d(0));
        return pose.transformBy(transform);
    }
    
    public Pose2d offsetPoint(Pose2d pose, double sideOffset, double forwardOffset) {
        Transform2d transform = new Transform2d(forwardOffset, sideOffset, new Rotation2d(0));
        return pose.transformBy(transform);
    }

    // rotation in degrees
    public Pose2d offsetPoint(Pose2d pose, double sideOffset, double forwardOffset, double rotationOffset) {
        Transform2d transform = new Transform2d(forwardOffset, sideOffset, new Rotation2d(Math.toRadians(rotationOffset)));
        return pose.transformBy(transform);
    }

    public void alignWithTag(Double targetX, Double ySpeed, Double turningSpeed) {
        double currentX = 0;
        double xError = targetX - currentX;

        Pose2d pose = getPose();

        // ChassisSpeeds robotLOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        ChassisSpeeds speeds = new ChassisSpeeds(
            shootController.calculate(xError, 0),
            ySpeed,
            turningSpeed
        );
        SmartDashboard.putNumber("TARGET X ERROR", xError);

        setModuleStatesFromSpeeds(speeds);
    }
}
