package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.DestemidosHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotConfiguration;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public final class GripSystem {
    public static void coletarCones(Gamepad player2, DestemidosHardware robot)
    {
        // segurar o cone
        double garraA_position;
        double garraB_position;
        if (player2.x) {
            garraA_position = RobotConfiguration.posiçãoGarraA_Fechada;
            garraB_position = RobotConfiguration.posiçãoGarraB_Fechada;

        } else {
            garraA_position = RobotConfiguration.posiçãoGarraA_Solta;
            garraB_position = RobotConfiguration.posiçãoGarraB_Solta;
        }
        robot.servoGarraA.setPosition(garraA_position);
        robot.servoGarraB.setPosition(garraB_position);

        double current_mao_position;
        if (player2.right_trigger > 0.0) {
            double movimento_mao = MathUtils.Clamp(
                    player2.right_trigger,
                    RobotConfiguration.posiçãoInicialMão,
                    RobotConfiguration.posiçãoLimiteInferiorMão
            );

            current_mao_position = movimento_mao;

        } else {
            current_mao_position = RobotConfiguration.posiçãoInicialMão;
        }
        robot.servoMão.setPosition(current_mao_position);
    }
}