//----------------------------------------------------------------IMPORTS-----------------------------------------------------------------------------------
package frc.robot.Drivetrain;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.ComponentsOld.AreaEffects.AreaEffectsHandler;
import frc.robot.Components.PositionComponent;

public class Drivetrain extends SubsystemBase {
//---------------------------------------------------------------VARIABLES-----------------------------------------------------------------------------------
    private static Drivetrain instance;
    private SwerveDriveKinematics kinematics = DrivetrainConstants.kinematics;
    private MAXSwerveModule[] MAXSwerveModules = {
        new MAXSwerveModule(DrivetrainConstants.flDrivingID, DrivetrainConstants.flTurningID),
        new MAXSwerveModule(DrivetrainConstants.frDrivingID, DrivetrainConstants.frTurningID),
        new MAXSwerveModule(DrivetrainConstants.blDrivingID, DrivetrainConstants.blTurningID),
        new MAXSwerveModule(DrivetrainConstants.brDrivingID, DrivetrainConstants.brTurningID)
    };
    private double driveSpeed = DrivetrainConstants.defaultDriveSpeed;
    private double rotSpeed = DrivetrainConstants.defaultRotSpeed;
//------------------------------------------------------CONSTRUCTOR, SINGLETON, & PERIODIC-----------------------------------------------------------------------------------
    
    private Drivetrain() {}
    /**
    * getInstance method for the drivetrain. Follows the singleton design pattern
    * @author Giahna C.
    * @return The instance of the drivetrain
    */

    public static Drivetrain getInstance(){
        if(instance == null){
            instance = new Drivetrain();
        }
        return instance;
    }
    
    @Override 
    public void periodic(){}
//-----------------------------------------------------------------SETTING PIDS-----------------------------------------------------------------------------------
    /**
    * Sets the PIDs for the drivetrain motor controllers based on velocity. setDrivePositionPIDs automatically calls this method.
    * @author Giahna C.
    * @param vx The velocity for the bot along the x axis according to NWU
    * @param vy The velocity for the bot along the y axis according to NWU
    * @param rot The velocity for the bot along the z axis (rotation)
    * @param DriveUsingNormalizedVectors If vx and vy are in the interval [-1,1], this should likely be true. The program will apply the speed before setting
    * the PIDs. If vx and vy are in meters/ second, set this to false.
    * @param TurnUsingNormalizedVectors If rot is in the interval [-1,1], this should likely be true. The program will apply the rotation speed before settings
    * this PID. If rot is in radians/ second, set this to false.
    * @return The instance of the drivetrain
    */
    public void setVelocityPIDs(double vx, double vy, double rot, boolean DriveUsingNormalizedVectors, boolean TurnUsingNormalizedVectors){
        double speed;
        double rSpeed;
        if (DriveUsingNormalizedVectors){
            speed = driveSpeed;
        } 
        else {
            vx = MathUtil.clamp(vx, -DrivetrainConstants.maxDriveSpeed, DrivetrainConstants.maxDriveSpeed);
            vy = MathUtil.clamp(vy, -DrivetrainConstants.maxDriveSpeed, DrivetrainConstants.maxDriveSpeed);
            speed = 1;
        }

        if (TurnUsingNormalizedVectors) rSpeed = rotSpeed;
        else {
            rot = MathUtil.clamp(rot, -DrivetrainConstants.maxRotSpeed, DrivetrainConstants.maxRotSpeed);
            rSpeed = 1;
        }
        // System.out.println("speed: " + PositionComponent.getChassisSpeeds().omegaRadiansPerSecond);
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx * speed, vy * speed, rot * rSpeed, PositionComponent.getPose2d().getRotation());
        SwerveModuleState[] swerveStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        for(int i = 0; i < 4; i++){
            Rotation2d currentAngle = new Rotation2d(MAXSwerveModules[i].turningEncoder.getPosition());
            swerveStates[i].optimize(currentAngle);
            // cosine compensation, optional
            swerveStates[i].speedMetersPerSecond *= swerveStates[i].angle.minus(currentAngle).getCos();
            // System.out.println(swerveStates[i].angle.getRadians());
            MAXSwerveModules[i].turningPIDController.setSetpoint(swerveStates[i].angle.getRadians(), ControlType.kPosition);
            MAXSwerveModules[i].drivingPIDFController.setSetpoint(swerveStates[i].speedMetersPerSecond, ControlType.kVelocity);
        }
    }
//----------------------------------------------------------SPEED RELATED METHODS-----------------------------------------------------------------------------------
    /**
     * Checks each MAXSwerveModule to see if it's moving or not. If one of them is moving it returns false.
     * @return whether or not the robot's driving motors all have a negligable velocity
     */
    public boolean isRobotStopped(){
        for(int i =0; i<4; i++){
            if(MAXSwerveModules[i].drivingEncoder.getVelocity() > DrivetrainConstants.stoppedVelocity) return false;
        }
        return true;
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] returnArray = new SwerveModulePosition[4];
        for(int i=0; i<4; i++){
            returnArray[i] = MAXSwerveModules[i].getSwerveModulePosition(); 
        }
        return returnArray;
    }

    public SwerveModuleState[] getSwerveModuleStates(){
        SwerveModuleState[] returnArray = new SwerveModuleState[4];
        for(int i=0; i<4; i++){
            returnArray[i] = new SwerveModuleState(
                MAXSwerveModules[i].drivingEncoder.getVelocity(),
                new Rotation2d(MAXSwerveModules[i].turningEncoder.getPosition())
            ); 
        }
        return returnArray;
    }
}
