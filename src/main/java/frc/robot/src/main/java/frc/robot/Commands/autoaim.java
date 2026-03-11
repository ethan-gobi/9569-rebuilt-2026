package frc.robot.Commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;
import swervelib.math.SwerveMath;

public class autoaim extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final int tagId;

    private static final double kP = 0.02;

    public autoaim(
            SwerveSubsystem swerve,
            DoubleSupplier forwardSupplier,
            DoubleSupplier strafeSupplier,
            int tagId) {
        this.swerve = swerve;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.tagId = tagId;

        addRequirements(swerve);
    }


    @Override
    public void execute() {
        double forward = forwardSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();

        double turn = 0.0;

        Optional<Double> yawOpt = swerve.getYawToTag(tagId);

        if (yawOpt.isPresent()) {
            double yaw = yawOpt.get();
            turn = -yaw * kP * swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
        }

        swerve.getSwerveDrive().drive(
                SwerveMath.scaleTranslation(
                        new Translation2d(
                                forward * swerve.getSwerveDrive().getMaximumChassisVelocity(),
                                strafe * swerve.getSwerveDrive().getMaximumChassisVelocity()),
                        0.8),
                turn,
                true,
                false);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.getSwerveDrive().drive(
                new Translation2d(0.0, 0.0),
                0.0,
                true,
                false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
