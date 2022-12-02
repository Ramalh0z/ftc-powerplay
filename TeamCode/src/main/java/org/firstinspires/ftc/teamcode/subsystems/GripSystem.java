package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotConfiguration;

public final class GripSystem {
    public static void coletarCones(Gamepad player2, DestemidosHardware robot)
    {
        // segurar o cone
        double garraA_position;
        double garraB_position;
        if (player2.right_bumper) {
            garraA_position = RobotConfiguration.posiçãoGarraA_Fechada;
            garraB_position = RobotConfiguration.posiçãoGarraB_Fechada;

        } else {
            garraA_position = RobotConfiguration.posiçãoGarraA_Solta;
            garraB_position = RobotConfiguration.posiçãoGarraB_Solta;
        }
        robot.servoGarraA.setPosition(garraA_position);
        robot.servoGarraB.setPosition(garraB_position);

        double current_mao_position;
        if (player2.left_bumper) {
            current_mao_position = RobotConfiguration.posiçãoLimiteInferiorMão;

        } else {
            current_mao_position = RobotConfiguration.posiçãoInicialMão;
        }
        robot.servoMão.setPosition(current_mao_position);
    }
}