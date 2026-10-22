
package frc.robot.Drivetrain;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class MAXSwerveModule{
    public SparkMax turningMotor;
    public SparkFlex drivingMotor;
    public AbsoluteEncoder turningEncoder;
    public RelativeEncoder drivingEncoder;
    public SparkClosedLoopController turningPIDController;
    public SparkClosedLoopController drivingPIDFController;

    public MAXSwerveModule(int drivingMotorControllerID, int turningMotorControllerID){
        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig
         .inverted(DrivetrainConstants.invertTurningMotors)
         .smartCurrentLimit(DrivetrainConstants.turningCurrentLimit)
         .idleMode(DrivetrainConstants.turningIdleMode);
        turningConfig.absoluteEncoder
         .inverted(DrivetrainConstants.invertTurningEncoders)
         .positionConversionFactor(2*Math.PI)
         .velocityConversionFactor(2*Math.PI/60);
        turningConfig.closedLoop
         .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
         .positionWrappingEnabled(true)
         .positionWrappingInputRange(0, 2*Math.PI)
        //  .allowedClosedLoopError(0.05)
         .minOutput(-1)
         .maxOutput(1)
         .pid(DrivetrainConstants.turningVelocityP, DrivetrainConstants.turningVelocityI, DrivetrainConstants.turningVelocityD)
         .feedForward
            .kV(DrivetrainConstants.turningVelocityF);

         SparkFlexConfig drivingConfig = new SparkFlexConfig();
         drivingConfig
        //   .inverted(DrivetrainConstants.invertDrivingMotors)
          .smartCurrentLimit(DrivetrainConstants.drivingCurrentLimit)
          .idleMode(DrivetrainConstants.drivingIdleMode);
        drivingConfig.encoder
        //  .inverted(DrivetrainConstants.invertDrivingEncoders) Inversion can't be set for driving encoder
         .positionConversionFactor(DrivetrainConstants.drivingEncoderPositionFactor)
         .velocityConversionFactor(DrivetrainConstants.drivingEncoderVelocityFactor);
        drivingConfig.closedLoop
         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
         .pid(DrivetrainConstants.drivingVelocityP, DrivetrainConstants.drivingVelocityI, DrivetrainConstants.drivingVelocityD)
         .minOutput(-1)
         .maxOutput(1)
         .positionWrappingEnabled(false)
         .feedForward
            .kV(DrivetrainConstants.drivingVelocityF);
        
        turningMotor = new SparkMax(turningMotorControllerID, MotorType.kBrushless);
        drivingMotor = new SparkFlex(drivingMotorControllerID, MotorType.kBrushless);

        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        drivingMotor.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        turningEncoder = turningMotor.getAbsoluteEncoder();
        drivingEncoder = drivingMotor.getEncoder();
        turningPIDController = turningMotor.getClosedLoopController();
        drivingPIDFController = drivingMotor.getClosedLoopController();
    }
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(drivingEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
    }
}
