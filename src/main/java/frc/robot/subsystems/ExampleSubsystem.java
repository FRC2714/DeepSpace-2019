package frc.robot.subsystems;

import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class ExampleSubsystem extends SubsystemModule {

    public ExampleSubsystem() { 
        registerCommands(); 
    }

    @Override public void run() {

    }

    @Override public void registerCommands() {

        new SubsystemCommand(this.registeredCommands, "example_command") {

			@Override
			public void initialize() {

			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {

			}
		};
    }
}