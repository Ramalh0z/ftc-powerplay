package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Mecanum Robot - Extensão da abstração que temos do nosso
 * robô, com ênfase mais nos atuadores que complementam o robô
 * 
 * TODO (ramalho): trazer mais mecanismos "padronizados" relacionados 
 * a movimentação omnidirecional para cá, e possivlemente separar estes
 * atuadores para um local bem mais apropriado
*/

public class MecanumRobot extends BaseRobot {
    /*
     * (ramalho): Assim como na base, o padrão dos atuadores tem que ser
     * respeitado, pois por baixo dos panos, não existe distinção entre
     * atuador e motor da base... mas para nós sim!
     * 
     * ordem dos atuadores
     *      0 - ArmA
     *      1 - ArmB
     *      2 - Garra
     *      3 - Motor do Pato
     * 
     * todos no EXPANSION HUB
     */
    public final DcMotorEx[] atuadores = new DcMotorEx[4];

    // Método princiapl para preparar os atuadores, desativa os encoders por padrão.
    public void inicializarExtraHardware(HardwareMap hardwareMap) {

        // mapeando os atuadores
        atuadores[0] = hardwareMap.get(DcMotorEx.class,"armA");
        atuadores[1] = hardwareMap.get(DcMotorEx.class,"armB");
        atuadores[2] = hardwareMap.get(DcMotorEx.class,"hand");
        atuadores[3] = hardwareMap.get(DcMotorEx.class,"pato");

        // configurando os encoders
        configEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER, atuadores);

        // configurando a direção
        atuadores[0].setDirection(DcMotor.Direction.FORWARD);
        atuadores[1].setDirection(DcMotor.Direction.REVERSE);
        atuadores[2].setDirection(DcMotor.Direction.FORWARD);

        // a "frente" dele é sempre mais rápida
        // então invertemos para engolir mais rápído
        atuadores[3].setDirection(DcMotor.Direction.REVERSE);
    }
}
