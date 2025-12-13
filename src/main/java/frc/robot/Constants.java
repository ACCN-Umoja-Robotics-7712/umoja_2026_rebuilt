// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {   

    public final class USB{
        public static final int DRIVER_CONTROLLER = 0;      // Driver Controller USB ID
        public static final int OPERATOR_CONTROLLER = 1;    // Operator controller USB ID
        public static final int OPERATOR_LY = 1;
        public static final int OPERATOR_LX = 0;
        public static final int OPERATOR_RY = 5;
        public static final int OPERATOR_RX = 4;
        public static final int OPERATOR_RT = 3;
        public static final int OPERATOR_LT = 2;
    }

    public final class ModuleConstants{
        public static final double kWheelDiameterMeters = 0.1143; // 4.5 inches
        // gear ratio from thrifty swerve https://thethriftybot.com/products/thrify-swerve gear ratio options (pinion size 12 + second stage gear 16t? (only confirmed pinion))
        public static final double kDriveMotorGearRatio = 1/6.0;
        public static final double kTurningMotorGearRatio = 1 / 21.42857142857143;//(12*14)/(72*50) based on #of teeth
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; // Math.PI * kWheelDiameterMeters = Circumference
        public static final double kTurnEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double 
        kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurnEncoderRPM2RadPerSec = kTurnEncoderRot2Rad / 60;
        public static final double kPTurning = 0.6;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(29-2.5); // 29 inches
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(27.375-2.5); // 27 + 3 med notch
        // Distance between front and back wheels

        public static final double kRobotRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2)) / 2;

        //TODO: Update to positive left positive forward FL, Fr, BL, BR
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 20;
        public static final int kFrontLeftTurningMotorPort = 21;

        public static final int kBackLeftDriveMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 11;

        public static final int kFrontRightDriveMotorPort = 26;
        public static final int kFrontRightTurningMotorPort = 25;

        public static final int kBackRightDriveMotorPort = 16;
        public static final int kBackRightTurningMotorPort = 15;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveReversed = false;
        public static final boolean kBackLeftDriveReversed = false;
        public static final boolean kFrontRightDriveReversed = false;
        public static final boolean kBackRightDriveReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 17;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 22;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 27;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // OLD OFFSETS
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRot = 0.849;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRot = 0.597;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRot = 0.148;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRot = 0.613;

        // OFFSETS
        // If robot is positively off, subtract
        // Else, add
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegree = 61.645715;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetDegree = 212.642091;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetDegree = 163.585028;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetDegree = 89.405267;
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegree = 0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDegree = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDegree = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetDegree = 0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.0;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kSlowButtonDriveModifier = 0.15;
        public static final double kSlowButtonTurnModifier = 0.10;
        public static final double teleSpeed = 0.45;
        public static final double teleTurnSpeed = 0.3;

        public static final double kPDrive = 1.5;
        public static final double kIDrive = 0.01;

        public static final double kPTurning = 5;
        public static final double kITurning = 0.05;



        public static final double kPDrift = 0.045;
        public static final double kIDrift = 0.0065; //Changed before test
    }


    public static final class RobotStates{

        public static final int NONE = -1;
        public static final int PICKUP = 0;
        public static final int L1 = 1;
        public static final int L2 = 2;
        public static final int L3 = 3;
        public static final int L4 = 4; 
        public static final int CLIMB_READY = 5;
        public static final int CLIMBING = 6;
        public static final int REMOVE = 7;
    }

    public static final class DeepClimbStates{ //Not used
        
        public static final double NONE = 01; 
        public static final double ZERO = 0; 
        public static final double READY = 0; // encoder value
        public static final double CLIMBING = 0; // encoder value
    }

    public static final class DeepClimbConstants{

        public static final int climbMotorID = 3;
        public static final double deepClimbBottomLimit = 0;
        public static final double kP = 0;
        public static final double kI = 0;
    }

    public static final class CoralConstants {

        public static final int coralIntakeMotorID = 1;
        public static final int coralArmMotorID = 2;

        public static final double coralArmBottomLimit = 5.0;
        public static final double coralArmTopLimit = 140; // TODO: Change this
        public static final double coralArmElevatorLimit = 81;

        public static final double kP = 0.1;
        
        public static final double armPickupPosition = 57;

        public static final double shootPositionL1 = 0;
        public static final double shootPositionL23 = 0;
        public static final double shootPositionL4 = 0;

        public static final double hasCoralProximity = 130;

        public static final double coralCurrentDiff = 0.05;

    }

    public static final class CoralArmStates {
        
        public static final double NONE = -1;
        public static final double PICKUP = 54.5;
        public static final double L1 = 10.5;
        public static final double L23 = 36;
        public static final double L4 = 33; // TODO: Fix value
        public static final double CLIMB = 0; // TODO: Find climb value
        public static final double ALGAE = 26.5;
        public static final double Barge_1 = 26; //New Addittion, W.I.P.
        public static final double Barge_2 = 35; // New Addition, W.I.P.
    }

    public static final class CoralIntakeStates {
        
        public static final double NONE = -1;
        public static final double INTAKE = 1;
        public static final double READY = 2;
        public static final double SHOOTING = 3;

    }

    public static final class ElevatorConstants {
        public static final int leftMotorID = 50;
        public static final int rightMotorID = 51;

        public static final double elevatorTopLimit = 152.0; 
        public static final double elevatorBottomLimit = 5.0;

        public static final double kP = 0.1; // TODO: Change/test to 0.13
        public static final double kI = 0.0005; 

        public static final double slowDown = 0.8;

        public static final double elevatorArmLimit = 42;

        public static final int elevatorMotor1ID = 8;
        public static final int elevatorMotor2ID = 9;

        public static final TrapezoidProfile.Constraints kElevatorConstraints =
            new TrapezoidProfile.Constraints(
                125,
                100);

    }

    public static final class ElevatorStates {

        public static final double NONE = -1;
        public static final double L1 = 5;
        public static final double L2 = 12;
        public static final double ALGAE = 48;
        public static final double L3 = 67-4;
        public static final double L4 = 150;
        public static final double CLIMB = 0;
        public static final double BARGE = 151;
    };

    public static final class LEDConstants {
        public static final int numLEDsPerStrip = 24;
        public static final int numLEDsPerColor = 3;
    }

    public static final class GameConstants {
        public static final int Robot = 0;
        public static final int Auto = 1;
        public static final int TeleOp = 2;
    }


    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;

        // public static final double kPXController = 5;
        // public static final double kPYController = 5;
        // public static final double kPThetaController = 2.5;

        // public static final double kIXController = 0.35; //0.06
        // public static final double kIYController = 0.35; // 0.06
        // public static final double kIThetaController = 0.2;
        public static final double kPXController = 4.8;// 1.5
        public static final double kPYController = 4.8; // 1.5
        public static final double kPThetaController = 2.5; // 2.5

        public static final double kIXController = 0.01; //0.01 
        public static final double kIYController = 0.01; // 0.01
        public static final double kIThetaController = 0.01;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final double firstWait = 0.4;
        public static final double secondWait = 0.9;
        public static final double stationWait = 0.4;
    }

    public static final class XBoxConstants {
        
        // Buttons
        public static final int A = 1; 
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;

        public static final int LB = 5;
        public static final int R1 = 6;
        
        public static final int PAGE = 7;
        public static final int MENU = 8;

        public static final int L3 = 9;
        public static final int R3 = 10;

        // Axes

        public static final int LX = 0;
        public static final int LY = 1; 

        public static final int LT = 2;
        public static final int RT = 3;

        public static final int RX = 4; 
        public static final int RY = 5;

    }
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final int kDriverRB = 6;

        public static final double kDeadband = 0.05;

        //BUTTONS

        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int L3 = 9;
        public static final int R3 = 10;

        //AXES

        public static final int LX = 0;
        public static final int LY = 1;
        public static final int LT = 2;

        public static final int RT = 3;
        public static final int RX = 4;
        public static final int RY = 5;
    }

    public static final class LimelightConstants {
        public static final String tagName = "limelight-tags";
        public static final String gamePieceName = "limelight-gp";
        public static final String driverName = "driver";
        public static final int Estimate_Distance = 20;
    }

    public static final class Colors {
        public static final Color red = Color.kRed;
        public static final Color blue = Color.kBlue;
        public static final Color green = new Color(0, 255, 0);
        public static final Color white = Color.kWhite;
        public static final Color uRed = new Color(20, 0 , 0);
        public static final Color uDarkOrange = new Color(254,17,1);
        public static final Color uGreen = new Color(0, 7, 0);
        public static final Color uOrange = new Color(255, 25, 0);
        public static final Color uGold = Color.kGold; //new Color(255, 215, 0);
        public static final Color REEFSCAPE_COLOR = Color.kAliceBlue;
        public static final Color[] uColors = {uRed, uDarkOrange, uGreen, uOrange};
    }

    
    public static final class AprilTagPositions {
        // all in m, 0 is left
        public static final Pose2d redPickupLeft1 = new Pose2d(16.697198, 0.65532, new Rotation2d(126));
        public static final Pose2d redPickupRight2 = new Pose2d(16.697198, 7.3964799999999995, new Rotation2d(234));
        public static final Pose2d redProcessor3 = new Pose2d(11.560809999999998, 8.05561, new Rotation2d(270));
        public static final Pose2d redReefLeft6 = new Pose2d(13.474446, 3.3063179999999996, new Rotation2d(30));
        public static final Pose2d redReefCenter7 = new Pose2d(13.890498, 4.0259, new Rotation2d(0));
        public static final Pose2d redReefRight8 = new Pose2d(13.474446, 4.745482, new Rotation2d(30));
        public static final Pose2d redReefBackRight9 = new Pose2d(12.643358, 4.745482, new Rotation2d(120));
        public static final Pose2d redReefBackCenter10 = new Pose2d(12.227305999999999, 4.0259, new Rotation2d(180));
        public static final Pose2d redReefBackLeft11 = new Pose2d(12.643358, 3.3063179999999996, new Rotation2d(240));
        public static final Pose2d bluePickupRight12 = new Pose2d(0.851154, 0.65532, new Rotation2d(54));
        public static final Pose2d bluePickupLeft13 = new Pose2d(0.851154, 7.3964799999999995, new Rotation2d(306));
        public static final Pose2d blueProcessor16 = new Pose2d(5.9875419999999995, -0.0038099999999999996, new Rotation2d(90));
        public static final Pose2d blueReefRight17 = new Pose2d(4.073905999999999, 3.3063179999999996, new Rotation2d(240));
        public static final Pose2d blueReefCenter18 = new Pose2d(3.6576, 4.0259, new Rotation2d(180));
        public static final Pose2d blueReefLeft19 = new Pose2d(4.073905999999999, 4.745482, new Rotation2d(120));
        public static final Pose2d blueReefBackLeft20 = new Pose2d(4.904739999999999, 4.745482, new Rotation2d(60));
        public static final Pose2d blueReefBackCenter21 = new Pose2d(5.321046, 4.0259, new Rotation2d(0));
        public static final Pose2d blueReefBackRight22 = new Pose2d(4.904739999999999, 3.3063179999999996, new Rotation2d(300));
    }

    public static final class Measurements {
        // all in m
        // robot l/w 28.5 by 28.5 inches
        // bumper width ~= 3 inches
        // TODO: Need to add potential intake front distance
        public static final double robotCenterToFront = 0.45085 + 0.03 + 0.2; // robot length/2 + bumper width = 14.25 + 3.5 = 17.75 inches  
        // public static final double robotCenterToFront = 0.50085; // Test
        public static final double robotSideOffset = 0.0254; // intake is 1 inch to the left so move robot 1 inch to the right
        public static final double branchOffset = 0.1651; // 6.5 inches
        public static final double coralStationDivotOffset = 0.2032; // 8 inches
    }

    public static final class RobotPositions {
        public static final Pose2d redCenter = new Pose2d(11.62, 4.0259, new Rotation2d(0));
        public static final Pose2d redCenterSafe = new Pose2d(11.35, 4.0259, new Rotation2d(0));
        
        public static final Pose2d redPickupLeft1 = new Pose2d(AprilTagPositions.redPickupLeft1.getX()-Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(54))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)), 
        AprilTagPositions.redPickupLeft1.getY()+Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(54)) - Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)), new Rotation2d(Units.degreesToRadians(306)));

        public static final Pose2d redPickupRight2 = new Pose2d(AprilTagPositions.redPickupRight2.getX()-Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(54)) + Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)), 
        AprilTagPositions.redPickupRight2.getY()-Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(54))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)), new Rotation2d(Units.degreesToRadians(54)));

        public static final Pose2d redProcessor3 = new Pose2d(AprilTagPositions.redProcessor3.getX()+Measurements.robotSideOffset, AprilTagPositions.redProcessor3.getY()-Measurements.robotCenterToFront, new Rotation2d(Units.degreesToRadians(90)));

        public static final Pose2d redReefLeft6 = new Pose2d(AprilTagPositions.redReefLeft6.getX()+Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), 
        AprilTagPositions.redReefLeft6.getY()-Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), new Rotation2d(Units.degreesToRadians(120)));

        public static final Pose2d redReefCenter7 = new Pose2d(AprilTagPositions.redReefCenter7.getX()+Measurements.robotCenterToFront, AprilTagPositions.redReefCenter7.getY()+Measurements.robotSideOffset, new Rotation2d(Units.degreesToRadians(180)));

        public static final Pose2d redReefRight8 = new Pose2d(AprilTagPositions.redReefRight8.getX()+Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(60))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), 
        AprilTagPositions.redReefRight8.getY()+Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.cos(Units.degreesToRadians(60)), new Rotation2d(Units.degreesToRadians(240)));

        public static final Pose2d redReefBackRight9 = new Pose2d(AprilTagPositions.redReefBackRight9.getX()-Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(60))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)),
        AprilTagPositions.redReefBackRight9.getY()+Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(60))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), new Rotation2d(Units.degreesToRadians(300)));

        public static final Pose2d redReefBackCenter10 = new Pose2d(AprilTagPositions.redReefBackCenter10.getX()-Measurements.robotCenterToFront, AprilTagPositions.redReefBackCenter10.getY()-Measurements.robotSideOffset, new Rotation2d(Units.degreesToRadians(0)));

        public static final Pose2d redReefBackLeft11 = new Pose2d(AprilTagPositions.redReefBackLeft11.getX()-Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), 
        AprilTagPositions.redReefBackLeft11.getY()-Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(60))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), new Rotation2d(Units.degreesToRadians(60)));

        public static final Pose2d bluePickupRight12 = new Pose2d(AprilTagPositions.bluePickupRight12.getX()+Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(54))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)),
        AprilTagPositions.bluePickupRight12.getY()+Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(54))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)), new Rotation2d(Units.degreesToRadians(234)));

        public static final Pose2d bluePickupLeft13 = new Pose2d(AprilTagPositions.bluePickupLeft13.getX()+Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(54))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)),
        AprilTagPositions.bluePickupLeft13.getY()-Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(54)) + Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(54)), new Rotation2d(Units.degreesToRadians(126)));

        public static final Pose2d blueProcessor16 = new Pose2d(AprilTagPositions.blueProcessor16.getX()-Measurements.robotSideOffset, AprilTagPositions.blueProcessor16.getY()+Measurements.robotCenterToFront, new Rotation2d(Units.degreesToRadians(270)));

        public static final Pose2d blueReefRight17 = new Pose2d(AprilTagPositions.blueReefRight17.getX()-Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), 
        AprilTagPositions.blueReefRight17.getY()-Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(60))-Measurements.robotSideOffset*Math.cos(Units.degreesToRadians(60)), new Rotation2d(Units.degreesToRadians(60)));

        public static final Pose2d blueReefCenter18 = new Pose2d(AprilTagPositions.blueReefCenter18.getX()-Measurements.robotCenterToFront, AprilTagPositions.blueReefCenter18.getY()-Measurements.robotSideOffset, new Rotation2d(Units.degreesToRadians(0)));

        public static final Pose2d blueReefLeft19 = new Pose2d(AprilTagPositions.blueReefLeft19.getX()-Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(60))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), 
        AprilTagPositions.blueReefLeft19.getY()+Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(60))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), new Rotation2d(Units.degreesToRadians(300)));

        public static final Pose2d blueReefBackLeft20 = new Pose2d(AprilTagPositions.blueReefBackLeft20.getX()+Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(60))-Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), 
        AprilTagPositions.blueReefBackLeft20.getY()+Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), new Rotation2d(Units.degreesToRadians(240)));

        public static final Pose2d blueReefBackCenter21 = new Pose2d(AprilTagPositions.blueReefBackCenter21.getX()+Measurements.robotCenterToFront, AprilTagPositions.blueReefBackCenter21.getY()+Measurements.robotSideOffset, new Rotation2d(Units.degreesToRadians(180)));

        public static final Pose2d blueReefBackRight22 = new Pose2d(AprilTagPositions.blueReefBackRight22.getX()+Measurements.robotCenterToFront*Math.cos(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)),
        AprilTagPositions.blueReefBackRight22.getY()-Measurements.robotCenterToFront*Math.sin(Units.degreesToRadians(60))+Measurements.robotSideOffset*Math.sin(Units.degreesToRadians(60)), new Rotation2d(Units.degreesToRadians(120)));        

    }

    public static final Pose2d[] bluePickUpPositions = { RobotPositions.bluePickupRight12, RobotPositions.bluePickupLeft13};
    public static final Pose2d[] blueReefPositions = { RobotPositions.blueReefRight17, RobotPositions.blueReefCenter18, RobotPositions.blueReefLeft19, RobotPositions.blueReefBackLeft20, RobotPositions.blueReefBackCenter21, RobotPositions.blueReefBackRight22};
    public static final Pose2d blueProcessorPosition = RobotPositions.blueProcessor16;
    public static final Pose2d[] redPickUpPositions = { RobotPositions.redPickupLeft1, RobotPositions.redPickupRight2};
    public static final Pose2d[] redReefPositions = { RobotPositions.redReefLeft6, RobotPositions.redReefCenter7, RobotPositions.redReefRight8, RobotPositions.redReefBackRight9, RobotPositions.redReefBackCenter10, RobotPositions.redReefBackLeft11};
    public static final Pose2d redProcessorPosition = RobotPositions.redProcessor3;

    // TODO: Update at comp
    public static final class BranchOffsets { // (Side offset -ve right, forward offset -ve back)
        public static final Pair<Double, Double> blueBranchAOffset = new Pair<Double, Double>(-0.03, 0.0);
        public static final Pair<Double, Double> blueBranchBOffset =  new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> blueBranchCOffset =  new Pair<Double, Double>(-0.06 /* -0.1 (to the right) */, 0.0);
        public static final Pair<Double, Double> blueBranchDOffset =  new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> blueBranchEOffset =  new Pair<Double, Double>(-0.08 /* -0.03 */, 0.0);
        public static final Pair<Double, Double> blueBranchFOffset = new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> blueBranchGOffset = new Pair<Double, Double>(-0.07, 0.0);
        public static final Pair<Double, Double> blueBranchHOffset = new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> blueBranchIOffset = new Pair<Double, Double>(-0.07, 0.0);
        public static final Pair<Double, Double> blueBranchJOffset = new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> blueBranchKOffset = new Pair<Double, Double>(-0.05, 0.0);
        public static final Pair<Double, Double> blueBranchLOffset = new Pair<Double, Double>(0.0, 0.0);
        
        public static final Pair<Double, Double> redBranchAOffset = new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> redBranchBOffset = new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> redBranchCOffset = new Pair<Double, Double>(-0.09, 0.0);
        public static final Pair<Double, Double> redBranchDOffset = new Pair<Double, Double>(0.03, 0.0);
        public static final Pair<Double, Double> redBranchEOffset = new Pair<Double, Double>(-0.04, 0.0);
        public static final Pair<Double, Double> redBranchFOffset = new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> redBranchGOffset = new Pair<Double, Double>(-0.07, 0.0);
        public static final Pair<Double, Double> redBranchHOffset = new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> redBranchIOffset = new Pair<Double, Double>(-0.07, 0.0);
        public static final Pair<Double, Double> redBranchJOffset = new Pair<Double, Double>(0.0, 0.0);
        public static final Pair<Double, Double> redBranchKOffset = new Pair<Double, Double>(-0.05, 0.0);
        public static final Pair<Double, Double> redBranchLOffset = new Pair<Double, Double>(0.0, 0.0);
    }

    public static final PathConstraints pathConstraints = new PathConstraints(4.8, 1.3, 540, 720);
}
