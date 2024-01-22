package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Eyes;

import edu.wpi.first.wpilibj2.command.Command;


public class TrackAprilTag extends Command {    
    
    private Eyes eyes;

    public TrackAprilTag(Eyes eyes) {

        this.eyes = eyes;
        addRequirements(eyes);

    }

    @Override
    public void execute() {

        eyes.readAprilTag();
    }
}