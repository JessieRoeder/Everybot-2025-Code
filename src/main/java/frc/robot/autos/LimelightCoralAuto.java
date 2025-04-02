package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCoralAuto extends Command {
    private DriveSubsystem m_drive;
    private RollerSubsystem m_roller;
    private ArmSubsystem m_arm;
    private LimelightSubsystem m_vision;
    private Timer timer;

    private double auto_seconds = 15;

    /**
     * This auto will have the robot drive forwards, stop, then drop the coral into L1
     * 
     * @param drive
     * @param roller
     * @param arm
     * @param vision
     */
    public LimelightCoralAuto(DriveSubsystem drive, RollerSubsystem roller, ArmSubsystem arm, LimelightSubsystem vision)
    {
        m_drive = drive;
        m_roller = roller;
        m_arm = arm;
        m_vision = vision;
        
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

    /**
     * While this timer is less than drive_seconds, the robot will obey the command inside
     */
    if(timer.get() < drive_seconds)
    {
        m_drive.driveArcade(-0.3, 0.0,false);
    }
    /**
     * Once the timer is greater than drive_seconds but less than exjest seconds,
     * the code inside will run, here we stop the drivetrain and exjest the coral.
     */
    else if(timer.get() > drive_seconds && timer.get() < exjest_seconds)
    {
        m_drive.driveArcade(0.0, 0.0,false);
        m_roller.runRoller(RollerConstants.ROLLER_CORAL_OUT);
    }
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
