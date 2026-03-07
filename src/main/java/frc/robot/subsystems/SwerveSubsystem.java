package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants;
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

    CANBus CANivore = new CANBus("CANivore");
    private final Pigeon2 gyro = new Pigeon2(1, CANivore);
    public PIDController shootController;
    public PIDController xController;
    public PIDController yController;
    public ProfiledPIDController thetaController;
    public HolonomicDriveController holonomicDriveController;
    public TrajectoryConfig trajectoryConfig;
    public final Timer timer = new Timer();
    // turretToTargetAngle current robot, assume turret is 0;
    private double turretToTargetAngle = 0;
    private double turretToTargetHoodValue = 0;
    private double turretToTargetRPMValue = 0;

    private double headingOffset = 0;
    
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

                setHeading(0);
                resetEncoders();
                // frontRight.driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
                // frontRight.turnMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
            }catch (Exception e) {
            }
        }).start();
    
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        // try{
            config = Constants.robotConfig;
        // } catch (Exception e) {
        // // Handle exception as needed
        //     e.printStackTrace();
        //     System.out.println("ROBOT GAVE UP PLEASE FIX CONFIG");
        //     return;
        // }

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

                    // var alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    //     return alliance.get() == DriverStation.Alliance.Red;
                    // }
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

    public void setHeading(double offset){
        RobotContainer.wantedAngle = -1;
        gyro.reset();
        gyro.setYaw(offset);
        System.out.println("Gyro Reset");
    }

    public double getHeading(){
        double yaw = gyro.getYaw().getValueAsDouble();

        // if red flip
        if (!DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue)) {
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

    // This is the same as getHeading but does not apply the alliance flip. Use this when you need the actual gyro reading rather than the field-relative heading
    public double getGlobalHeading() {
        double yaw = gyro.getYaw().getValueAsDouble();

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
    StructPublisher<Pose2d> turretPublisher = NetworkTableInstance.getDefault().getStructTopic("TurretPose", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> allPointsPublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("AllPosesArray", Pose2d.struct).publish();

    StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();


    public void publishRobotPositions() {
        ArrayList<Pose2d> allPoints = new ArrayList<>();
        // allPoints.add(new Pose2d(3.568, 7.602, new Rotation2d(0)));
        // allPoints.add(new Pose2d(3.568, 7.602, new Rotation2d(0)));
        allPoints.add(Constants.SHOOTING_POSES.BLUE_DEPOT_CENTER);

        allPoints.add(Constants.SHOOTING_POSES.BLUE_DEPOT_CORNER);

        allPoints.add(Constants.SHOOTING_POSES.BLUE_HALF_LEFT);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_HALF_RIGHT);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_TRENCH_LEFT);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_TRENCH_RIGHT);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_HUB_CENTER);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_OUTPOST_CENTER);
        
        allPoints.add(Constants.SHOOTING_POSES.BLUE_TOWER_CENTER);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_DEPOT_CENTER);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_DEPOT_CORNER);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_HALF_LEFT);

        allPoints.add(Constants.SHOOTING_POSES.RED_HALF_RIGHT);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_HUB_CENTER);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_NEUTRAL_LEFT);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_NEUTRAL_RIGHT);
        
        allPoints.add(Constants.SHOOTING_POSES.RED_NEUTRAL_RIGHT);
        
        allPointsPublisher.set(allPoints.toArray(new Pose2d[0]));
    }

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
       
        double up = Constants.TurretConstants.upOffset;
        // double yaw =  RobotContainer.shooterTurretSubsystem.getAngle();
        double yaw = 0;
        double pitch = Constants.TurretConstants.pitchOffset;
        double roll = Constants.TurretConstants.rollOffset;
        double forward = Constants.TurretConstants.turretCenterToCameraCentreLength + Math.sin(Math.toRadians(yaw)) * TurretConstants.turretCenterFromRobotCenterForwardLength;
        double side = Constants.TurretConstants.turretCenterToCameraCentreLength + -Math.cos(Math.toRadians(yaw)) * TurretConstants.turretCenterFromRobotCenterSideLength;
        LimelightHelpers.setCameraPose_RobotSpace(LimelightConstants.turretName, forward, side, up, roll, pitch, yaw + 180);
        Pose3d turretCameraPose3d = LimelightHelpers.getCameraPose3d_RobotSpace(LimelightConstants.turretName);
        Pose2d turretCameraRobotPose = turretCameraPose3d.toPose2d();

        LimelightHelpers.SetRobotOrientation(LimelightConstants.turretName, getHeading(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate turretMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.turretName);

        SmartDashboard.putNumber("TURRET ANGLE", yaw);
        boolean rejectTurretUpdate = false;

        if (Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 360 || RobotContainer.shooterTurretSubsystem.getVelocity() > 360) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            rejectTurretUpdate = true;
        }
        double visionTrustValue = 0.7;
        if (RobotContainer.gameState == GameConstants.Disabled) {
            visionTrustValue = 0;
        }
        if (turretMT2 != null) {
            if (turretMT2.tagCount == 0) {
                rejectTurretUpdate = true;
            } else if (turretMT2.tagCount == 1) {
                visionTrustValue += 2;
            }

            if (!rejectTurretUpdate)
            {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustValue,visionTrustValue,9999999));
                poseEstimator.addVisionMeasurement(
                    turretMT2.pose,
                    turretMT2.timestampSeconds);
            }
        }
        
        String limelightName = LimelightConstants.climbName;
        LimelightHelpers.SetRobotOrientation(limelightName, getHeading(), 0, 0, 0, 0, 0);
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, LimelightConstants.gamePieceForward, LimelightConstants.gamePieceSide, LimelightConstants.gamePieceHeight, 0, LimelightConstants.gamePieceAngle, 180);
        LimelightHelpers.PoseEstimate gpMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.gamePieceName);

        boolean rejectGPUpdate = false;

        if (Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > 360) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
            rejectGPUpdate = true;
        }

        double visionTrustGPValue = 0.7;
        if (RobotContainer.gameState == GameConstants.Disabled) {
            visionTrustGPValue = 0;
        }
        if (gpMT2 != null) {
            if (gpMT2.tagCount == 0) {
                rejectGPUpdate = true;
            } else if (gpMT2.tagCount == 1) {
                visionTrustGPValue += 2;
            }
            if (!rejectGPUpdate)
            {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTrustGPValue,visionTrustGPValue,9999999));
                poseEstimator.addVisionMeasurement(
                    gpMT2.pose,
                    gpMT2.timestampSeconds);
                System.out.println("UPDATING POSE");
            }
        }

        Pose2d currentPose = getPose();

        Translation2d turretCameraFieldTranslation = turretCameraRobotPose.getTranslation().plus(currentPose.getTranslation());
        Pose2d turretCameraFieldPose = new Pose2d(turretCameraFieldTranslation, turretCameraRobotPose.getRotation().plus(currentPose.getRotation()));
        turretPublisher.set(turretCameraFieldPose);
        posePublisher.set(currentPose);
        
        double[] angleDistance = updateRobotAngleDistanceToTarget(Constants.SHOOTING_POSES.BLUE_HUB_POSE);
        this.turretToTargetAngle = angleDistance[0];
        SmartDashboard.putNumber("WANTED TARGET ANGLE", turretToTargetAngle);
        double distanceToTarget = angleDistance[1];
        SmartDashboard.putNumber("WANTED TARGET DISTANCE", distanceToTarget);
        double[] rpmHoodValues = updateRPMHoodValues(distanceToTarget);
        this.turretToTargetRPMValue = rpmHoodValues[0];
        this.turretToTargetHoodValue = rpmHoodValues[1];
        // corner ~5.7m, tower ~3.1m, trench ~3.8m
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

        // Pose2d pose = getPose();

        // ChassisSpeeds robotLOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        ChassisSpeeds speeds = new ChassisSpeeds(
            shootController.calculate(xError, 0),
            ySpeed,
            turningSpeed
        );
        SmartDashboard.putNumber("TARGET X ERROR", xError);

        setModuleStatesFromSpeeds(speeds);
    }
    
    public double[] updateRobotAngleDistanceToTarget(Pose2d targetPose) {
        Translation2d toTag = targetPose.getTranslation().minus(RobotContainer.swerveSubsystem.getPose().getTranslation());
        double turretAngleToTarget = Units.radiansToDegrees(Math.atan2(toTag.getY(), toTag.getX()) + Math.PI);
        double distanceToTarget = toTag.getDistance(new Translation2d(0, 0));
        return new double[] { turretAngleToTarget, distanceToTarget};// subtract robot heading to get turret angle relative to robot forward
    }

    public double[] updateTurretAngleDistanceToTarget(Pose2d targetPose) {
        double[] angleDistance = updateRobotAngleDistanceToTarget(targetPose);
        double angle = angleDistance[0];
        double distance = angleDistance[1];
        return new double[] {angle - RobotContainer.swerveSubsystem.getGlobalHeading(), distance};
    }
    

    public double getTurretToTargetAngle() {
        return turretToTargetAngle;
    }

    public double getTurretToTargetRPMValue() {
        return turretToTargetRPMValue;
    }

    public double getTurretToTargetHoodValue() {
        return turretToTargetHoodValue;
    }

    public double[] updateRPMHoodValues(double distanceToTarget) {
        // Example calculation for RPM and hood values based on distance
        double rpm = 1000 + (distanceToTarget * 50); // Example formula
        double hoodValue = 15 + (distanceToTarget * 0.1); // Example formula
        return new double[] {rpm, hoodValue};
    }
}
