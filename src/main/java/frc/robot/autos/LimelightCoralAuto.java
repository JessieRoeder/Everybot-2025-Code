package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;

public class LimelightCoralAuto extends Command {
    private DriveSubsystem m_drive;
    private RollerSubsystem m_roller;
    private ArmSubsystem m_arm;
    private Timer timer;

    private double auto_seconds = 15;

    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.03;                    // how hard to turn toward the target
    final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

    /**
     * This auto will have the robot drive forwards, stop, then drop the coral into L1
     * 
     * @param drive
     * @param roller
     * @param arm
     */
    public LimelightCoralAuto(DriveSubsystem drive, RollerSubsystem roller, ArmSubsystem arm)
    {
        m_drive = drive;
        m_roller = roller;
        m_arm = arm;
        
        timer = new Timer();

        addRequirements(m_drive);
        addRequirements(m_roller);
        addRequirements(m_arm);

        
    }

    @Override
  public void initialize() {
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart();
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    /**
     * We always want to hold the arm up duirng the auto to ensure the rollers
     */ 
    m_arm.runArm(ArmConstants.ARM_HOLD_UP);

    //get the LimeLight data
    double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC("");  // Vertical  offset from principal pixel/point to target in degrees

  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
    // stop drive motors
    m_drive.driveArcade(0.0, 0.0, false);
    m_roller.runRoller(0);
    timer.stop();
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // check if timer exceeds seconds, when it has this will return true indicating
    // this command is finished
    return timer.get() >= auto_seconds;
  }
}
