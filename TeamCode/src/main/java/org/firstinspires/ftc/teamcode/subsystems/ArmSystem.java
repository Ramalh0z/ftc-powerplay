package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotConfiguration;

public final class ArmSystem {

    public static void movimentarBraço(Gamepad driver, DestemidosHardware robot)
    {
        // Gira a base do robô
        robot.motorCentro.setPower(-driver.right_stick_x * RobotConfiguration.usoDoMotorCentro);

        double controlPower = driver.left_stick_y; //* RobotConfiguration.usoDasGarras;
        robot.motorBraçoA.setPower(controlPower);
        robot.motorBraçoB.setPower(controlPower);

    }
}
