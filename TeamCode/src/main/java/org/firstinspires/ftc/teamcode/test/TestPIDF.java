package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.modules.ControladorPID;
import org.firstinspires.ftc.teamcode.robots.BaseRobot;

/*
 * TestPIDF - OpMode dedicado a testar o comportamento do controlador PID + Termo F
 * que já vem por padrão nos motores, porém é considerado DESCONTINUADO a partir da
 * versõa 7.0 do SDK, como visto nesse documento aqui: 
 * https://github.com/FIRST-Tech-Challenge/ftcdocs/blob/main/docs/source/programming_resources/shared/pidf_coefficients/pidf-coefficients.rst
 */

@Autonomous(name = "TestPIDF", group = "Test")
public class TestPIDF extends OpMode {

    // Componentes do robô
    private final BaseRobot robot = new BaseRobot();
    private BNO055IMU sensorIMU;
    
    // Controlador com os coeficienetes pré-defidos
    private final ControladorPID controladorPID = new ControladorPID(1.5, 0.0001, 0.0);

    @Override
    public void init() {

        // setup padrão
        robot.inicializarHardware(hardwareMap);
        sensorIMU = hardwareMap.get(BNO055IMU.class, "imu");

        // dessa vez, iremos sobrescrever a configuração dos encoders
        robot.resetRodasEncoder();
        robot.configEncoders(DcMotor.RunMode.RUN_USING_ENCODER, robot.motores); 

    }

    @Override
    public void loop() {

        // calcula o pid de acordo com referência "estática" e a velocidade do momento em cada motor
        double setpoint = 100;

        double resultado0 = controladorPID.calcularPID(setpoint, robot.motores[0].getVelocity());
        double resultado1 = controladorPID.calcularPID(setpoint, robot.motores[1].getVelocity());
        double resultado2 = controladorPID.calcularPID(setpoint, robot.motores[2].getVelocity());
        double resultado3 = controladorPID.calcularPID(setpoint, robot.motores[3].getVelocity());

        // e usa cada resultado pra afinar a velocidade
        // OBS: o negativo é apenas para motores da esquerda (que são inrvertidos)
        robot.motores[0].setVelocity(-resultado0);
        robot.motores[1].setVelocity(resultado1);
        robot.motores[2].setVelocity(-resultado2);
        robot.motores[3].setVelocity(resultado3);
    }
}
