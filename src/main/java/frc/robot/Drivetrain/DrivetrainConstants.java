package frc.robot.Drivetrain;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DrivetrainConstants {
//-------------------------------------------------------------------SPEEDS-----------------------------------------------------------------------------------
    public static final double defaultDriveSpeed = 1; //default speed that will be used in meters/second
    public static final double defaultRotSpeed = Math.PI; //default speed that will be used for rotation in radians/second
    public static final double maxDriveSpeed = 3.0; // Maximum drive speed in meters/second
    public static final double maxRotSpeed = 2.0; // Maximum rotation speed in radians per second
    public static final double maxDriveAcceleration = 2.0; // Maximum drive acceleration in meters per second squared
    public static final double maxRotAcceleration = 1.0; // Maximum rotation acceleration in radians per second squared
//-----------------------------------------------------------MECHANICAL Information: Default--Phoenix-----------------------------------------------------------------------------------
     //swerve module locations relative to the robot's center in meters
     //TODO: Update based on your teams chassis size.
     public static final double distanceOfWheelModuleFromCenter = 0.301625;
     //create the location of each module
     public static final Translation2d frontLeftLocation = new Translation2d(distanceOfWheelModuleFromCenter, distanceOfWheelModuleFromCenter);
     public static final Translation2d frontRightLocation = new Translation2d(distanceOfWheelModuleFromCenter, -distanceOfWheelModuleFromCenter);
     public static final Translation2d backLeftLocation = new Translation2d(-distanceOfWheelModuleFromCenter, distanceOfWheelModuleFromCenter);
     public static final Translation2d backRightLocation = new Translation2d(-distanceOfWheelModuleFromCenter, -distanceOfWheelModuleFromCenter);
     public static final Translation2d[] SwerveModulePositions = {frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation};
     //wheel stuff
     public static final double wheelDiameterMeters = 0.0762;
     public static final int drivingMotorPinionTeeth = 14;
     public static final int drivingMotorSpurTeeth = 21;
     //IDs
     //TODO:Update based on you team's motor IDs. fl: front left, fr: front right, bl: back left, br: back right
     public static final int flDrivingID = 1;
     public static final int flTurningID = 2;
     public static final int frDrivingID = 3;
     public static final int frTurningID = 4;
     public static final int blDrivingID = 5;
     public static final int blTurningID = 6;
     public static final int brDrivingID = 7;
     public static final int brTurningID = 8;
  
//-----------------------------------------------------------------PIDS-----------------------------------------------------------------------------------
    //PIDs for turning during velocity control
    //TODO:Tune these PIDs for your robot
    public static final double turningVelocityP = 1;
    public static final double turningVelocityI = 0;
    public static final double turningVelocityD = 0.02;
    public static final double turningVelocityF = 0;
    //PIDs for driving during velocity control + ff control
    public static final double drivingVelocityP = 0.08;
    public static final double drivingVelocityI = 0;
    public static final double drivingVelocityD = 0.04;
    public static final double drivingVelocityF = 0.195;

//-----------------------------------------------------STUFF TO TEST/ MESS WITH-----------------------------------------------------------------------------------
    //go to a point stuff
    public static final double atPointTarget = 0.05;
    public static final double slowDistance = 0.1; /** The distance at which the robot will begin to slow down when using goToPoint*/
    public static final double atRotTarget = 0.14;
    public static final double slowAngle = 0.2;
    public static final double stoppedVelocity = 0.02;//velocity considered stopped, will be tested on each individual module
    public static final double multiplierConstant = 0.25;/*When the trapezoid profile is done the robot will fix position if it's off by too much
    the normalized vector for velocity will be multiplied by distance/multiplierConstant in that case */
    public static final double rotMultiplierConstant = 0.1; //similar to the one above-ish but it's for rotation
    
    //current limits
    public static final int drivingCurrentLimit = 80;//in amps
    public static final int turningCurrentLimit = 40;//in amps, actual stall limit is 100
    //deadband for joystick control
    public static final double inputDeadband = 0.12;
   
//-----------------------------------------------------INVERSION, VERY IMPORTANT-----------------------------------------------------------------------------------
    public static final boolean invertTurningMotors = false;
    // public static final boolean invertDrivingMotors = false; Use firmware client instead.
    public static final boolean invertTurningEncoders = true;
    // public static final boolean invertDrivingEncoders = false; inversion can't be set for driving encoders

//----------------------------------------------------------------MISC.-----------------------------------------------------------------------------------
    //behavior when the robot is not moving
    public static final IdleMode turningIdleMode = IdleMode.kBrake;
    public static final IdleMode drivingIdleMode = IdleMode.kBrake;
    //calculations for encoders
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    public static final double drivingMotorReduction = (45.0 * drivingMotorSpurTeeth) / (drivingMotorPinionTeeth * 15);
    public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI) / drivingMotorReduction; // meters
    public static final double drivingEncoderVelocityFactor = drivingEncoderPositionFactor / 60.0; // meters per second
    //Objects that are reused and never change
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation, 
        frontRightLocation, 
        backLeftLocation, 
        backRightLocation
    );
    public static final TrapezoidProfile xProfile= new TrapezoidProfile(
        new TrapezoidProfile.Constraints(DrivetrainConstants.maxDriveSpeed, DrivetrainConstants.maxDriveAcceleration)
    );
    public static final TrapezoidProfile yProfile= new TrapezoidProfile(
        new TrapezoidProfile.Constraints(DrivetrainConstants.maxDriveSpeed, DrivetrainConstants.maxDriveAcceleration)
    );
    public static final TrapezoidProfile rotProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(DrivetrainConstants.maxRotSpeed, DrivetrainConstants.maxRotAcceleration)
    );
    public static final SwerveModuleState[] holdSwerveStates = {
            new SwerveModuleState(0.0, new Rotation2d(Math.PI/4)), //fl
            new SwerveModuleState(0.0, new Rotation2d(-Math.PI/4)),//fr
            new SwerveModuleState(0.0, new Rotation2d(-Math.PI/4)),//bl
            new SwerveModuleState(0.0, new Rotation2d(Math.PI/4)) //br
    }; //Swerve states for the holdPosition command.
 
}
