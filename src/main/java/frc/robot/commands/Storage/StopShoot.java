package frc.robot.commands.Storage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Storage.Indexer;
import frc.robot.subsystems.Storage.Wheel;

/**
 * Immediately stops both the flywheel and indexer.
 */
public class StopShoot extends Command {

    public StopShoot() {
        addRequirements(RobotContainer.m_Storage);
    }

    @Override
    public void initialize() {
        RobotContainer.m_Storage.setWheel(Wheel.OFF);
        RobotContainer.m_Storage.moveWheel(0);
        RobotContainer.m_Storage.setIndexer(Indexer.OFF);
        RobotContainer.m_Storage.moveIndexer(0);
        RobotContainer.m_Storage.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
