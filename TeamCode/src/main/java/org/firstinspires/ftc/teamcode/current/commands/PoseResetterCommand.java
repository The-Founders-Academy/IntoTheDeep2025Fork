package org.firstinspires.ftc.teamcode.current.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.current.subsytems.Mecanum2025;

public class PoseResetterCommand extends CommandBase {

    public PoseResetterCommand(Pose2d pose, Mecanum2025 mecanumDrive) {
        mecanumDrive.resetPose(pose);
    }

}
