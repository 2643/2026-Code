package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

public class ToggleMotorCommand extends CommandBase {
  private final Storage storage;

  public ToggleMotorCommand(Storage storage) {
    this.storage = storage;
    addRequirements(storage);
  }

  @Override
  public void initialize() {
    storage.toggleMotor(); // Use the toggleMotor method from the Storage subsystem
  }

  @Override
  public boolean isFinished() {
    return true; // Command finishes immediately after toggling
  }
}
