package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase{
    
    public LimelightSubsystem () {

        // Basic targeting data
        double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
        double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
        boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

        double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
        double tync = LimelightHelpers.getTYNC("");  // Vertical  offset from principal pixel/point to target in degrees        
        
        

    }
}
