package frc.robot.commands.limelightCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;

public class alignSpecificXandYLeftCamera extends Command {

    private final VisionSubsystem VISION_SUBSYSTEM; 
    private final DriveSubsystem DRIVE_SUBSYSTEM; 

    private final PIDController strafePID; 
    private final PIDController drivePID; 
    private final PIDController rotationPID; 

    private final boolean endCommand; 

    private double measuredValueX; 
    private double measuredValueY; 

    private double strafeSpeed; 
    private double driveSpeed;
    private double rotationSpeed; 


    // TARGETS VALUES 
    private final double targetValueX; 
    private final double targetValueY;
    private double targetAngle; 


    // RANGE 
    private boolean inRangeX; 
    private boolean inRangeY; 

    private int targetId; 

    /** Creates a new alignmentCommand. */
    public alignSpecificXandYLeftCamera(DriveSubsystem drive, VisionSubsystem vision, boolean end, double targetOffsetX, double targetOffsetY, int id) { // NEW: Accepts a locked heading from `TurnToAprilTagCommand`
        this.DRIVE_SUBSYSTEM = drive; 
        this.VISION_SUBSYSTEM = vision; 

        this.drivePID = new PIDController(VisionConstants.driveAlignKp,VisionConstants.driveAlignKi, VisionConstants.driveAlignKd); 
        this.strafePID = new PIDController(VisionConstants.strafeAlignKp,VisionConstants.strafeAlignKi, VisionConstants.strafeAlignKd);
        this.rotationPID = new PIDController(VisionConstants.rotAlignKp,VisionConstants.rotAlignKi, VisionConstants.rotAlignKd); // NEW: Controls unintended turning

        this.endCommand = end; 

        this.targetValueX = targetOffsetX;
        this.targetValueY = targetOffsetY;  

        this.targetId = id; 
  
        addRequirements(DRIVE_SUBSYSTEM, VISION_SUBSYSTEM);
        
    }

    @Override
    public void initialize() {
        drivePID.reset();
        strafePID.reset();
        rotationPID.reset(); // NEW: Reset rotation PID

        VISION_SUBSYSTEM.setLeftPipeline(VisionConstants.aprilTagPipeline);
        inRangeX = false; 
        inRangeY = false; 

        targetAngle = DRIVE_SUBSYSTEM.getHeading(); 
    }

    @Override
    public void execute() {
        if(VISION_SUBSYSTEM.limelightLeftTargetSeen()){
            measuredValueY = VISION_SUBSYSTEM.getLeftSpecificTagTy(targetId);
            measuredValueX = VISION_SUBSYSTEM.getLeftSpecificTagTx(targetId); 
            
            
            // X-direction (Strafing)
            if (Math.abs(targetValueX - measuredValueX) <= VisionConstants.xTol) { 
                strafeSpeed = 0; 
                inRangeX = true; 
            } else {
                strafeSpeed = strafePID.calculate(measuredValueX, targetValueX);
            }
            strafeSpeed = MathUtil.clamp(strafeSpeed, -0.25, 0.25); 

            

            // Y-direction (Forward movement)
            if (Math.abs(targetValueY - measuredValueY) <= VisionConstants.yTol) { 
                driveSpeed = 0; 
                inRangeY = true; 
            } else {
                driveSpeed = drivePID.calculate(measuredValueY, targetValueY);
            }
            driveSpeed = MathUtil.clamp(driveSpeed, -0.35, 0.35);

  


            // LOCK HEADING 
            double currentHeading = DRIVE_SUBSYSTEM.getHeading();
            rotationSpeed = rotationPID.calculate(currentHeading, targetAngle);
            rotationSpeed = MathUtil.clamp(rotationSpeed, -0.2, 0.2); 

            SmartDashboard.putNumber("Align Strafe Speed", strafeSpeed); 
            SmartDashboard.putNumber("Align Drive Speed", driveSpeed);
            SmartDashboard.putNumber("Align Rotation Speed", rotationSpeed);
            SmartDashboard.putBoolean("In Range X", inRangeX); 
            SmartDashboard.putBoolean("In Range Y", inRangeY);



           DRIVE_SUBSYSTEM.drive(driveSpeed, strafeSpeed, rotationSpeed, false);

        } else {
            driveSpeed = 0;
            strafeSpeed = 0;
            rotationSpeed = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        DRIVE_SUBSYSTEM.drive(0, 0, 0, false); 

    }

    @Override
    public boolean isFinished() {
        if(endCommand){
            return true; 
        } else if(inRangeX && inRangeY){
            return true; 
        } else {
            return false; 
        }
    }
}


