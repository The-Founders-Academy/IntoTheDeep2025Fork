package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.current.subsytems.TestLiftSubsystem;

public class Jan13LiftCommand extends CommandBase {
    private TestLiftSubsystem m_lift;

    public Jan13LiftCommand(TestLiftSubsystem lift) {
        m_lift = lift;
    }

    @Override
    public void initialize() {
        m_lift.setTargetExtension(TestLiftSubsystem.LiftParams.TestDistanceTicks);
    }

    @Override
    public void execute() {
        m_lift.move(0.5);
    }

    @Override
    public boolean isFinished() {
        return m_lift.atTargetExtension();
    }

    @Override
    public void end(boolean interrupted) {
        m_lift.stop();
    }
}
