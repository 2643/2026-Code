package frc.robot.commands.Storage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Storage.Indexer;
import frc.robot.subsystems.Storage.Wheel;

/**
 * Runs the flywheel immediately, then adds the indexer after 1.5 seconds.
 * Runs until interrupted — pair with StopShoot or use a race/deadline group in auto.
 */
public class Shoot extends Command {

    private final Timer timer = new Timer();
    private boolean indexerStarted = false;

    public Shoot() {
        addRequirements(RobotContainer.m_Storage);
    }

    @Override
    public void initialize() {
        indexerStarted = false;
        RobotContainer.m_Storage.resetTimer();
        RobotContainer.m_Storage.setWheel(Wheel.ON);
        RobotContainer.m_Storage.setIndexer(Indexer.OFF);
        timer.restart();
    }

    @Override
    public void execute() {
        // Always drive the flywheel while this command is running
        RobotContainer.m_Storage.moveWheel(Constants.StorageConstants.attackSpeed);

        if (!indexerStarted && timer.hasElapsed(1.5)) {
            RobotContainer.m_Storage.setIndexer(Indexer.ON);
            indexerStarted = true;
        }

        // Drive the indexer every loop once it has been started
        if (indexerStarted) {
            RobotContainer.m_Storage.moveIndexer(Constants.StorageConstants.indexSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        RobotContainer.m_Storage.setWheel(Wheel.OFF);
        RobotContainer.m_Storage.moveWheel(0);
        RobotContainer.m_Storage.setIndexer(Indexer.OFF);
        RobotContainer.m_Storage.moveIndexer(0);
        RobotContainer.m_Storage.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted by StopShoot or a race/deadline group
    }
}
