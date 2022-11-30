package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotConfiguration;
import org.firstinspires.ftc.teamcode.hardware.RobotConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.RobotLogger;

import java.util.List;

@Config
@TeleOp(name = "FTC_4", group = "TeleOps")
public class FTC_4 extends OpMode {

    // Componentes essenciais do robô
    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final DcMotorEx[] atuadores = new DcMotorEx[4];
    private Servo mao;
    private Servo garraA;
    private Servo garraB;

    private List<LynxModule> hubs;

    @Override
    public void init() {

        // setup inicial do ftc-dashboard
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // configura os bulk reads para um "cache" de valores automático
        // para mais informações, veja: https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        hubs = hardwareMap.getAll(LynxModule.class);

        // TODO: organizar a sequência das portas de forma bem mais intuitiva. (ramalho)
        // mapeando cada motor e atuador, usando o ID definido no robô
        motors[0] = hardwareMap.get(DcMotorEx.class,"DF");  // porta 0 - controlHub;
        motors[1] = hardwareMap.get(DcMotorEx.class,"DT");  // porta 1 - controlHub;
        motors[2] = hardwareMap.get(DcMotorEx.class,"EF");  // porta 3 - expansion;
        motors[3] = hardwareMap.get(DcMotorEx.class,"ET");  // porta 3 - controlHub;

        atuadores[0] = hardwareMap.get(DcMotorEx.class,"centro");   // porta 0 - rev2
        atuadores[2] = hardwareMap.get(DcMotorEx.class,"braçoA");   // porta 2 - rev2
        atuadores[3] = hardwareMap.get(DcMotorEx.class,"braçoB");   // porta 3 - rev2

        // Confgurando os servos
        mao    = hardwareMap.get(Servo.class, "mão");
        garraA = hardwareMap.get(Servo.class, "garraA");
        garraB = hardwareMap.get(Servo.class, "garraB");

        motors[2].setDirection(DcMotorEx.Direction.REVERSE);
        motors[3].setDirection(DcMotorEx.Direction.REVERSE);

        // braçoB
        atuadores[2].setDirection(DcMotorSimple.Direction.REVERSE);
        atuadores[3].setDirection(DcMotorSimple.Direction.FORWARD);

        mao.setDirection(Servo.Direction.REVERSE);
        garraA.setDirection(Servo.Direction.REVERSE);
        garraB.setDirection(Servo.Direction.FORWARD);

        // NOTE: na maioria das vezes, usar os encoders comuns já é o sufciente (ramalho)
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Configurando os atuadores
        atuadores[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atuadores[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atuadores[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        atuadores[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atuadores[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atuadores[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        atuadores[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atuadores[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atuadores[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        // inputs da movimentação
        double y    = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double x    = gamepad1.left_stick_x * RobotConstants.kCorretorJoystickX;
        double giro = -gamepad1.right_stick_x;

        // limita os motores na faixa dos [-1, 1]
        double maximo = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(giro), 1);

        double leftFrontPower  = (y + giro + x) / maximo;
        double rightFrontPower = (y - giro - x) / maximo;
        double leftBackPower   = (y - giro + x) / maximo;
        double rightBackPower  = (y + giro - x) / maximo;

        //
        motors[2].setPower(leftFrontPower);
        motors[0].setPower(rightFrontPower);
        motors[1].setPower(leftBackPower);
        motors[3].setPower(rightBackPower);

        // segurar o cone
        if(gamepad2.x) {
            garraA.setPosition(RobotConfiguration.posiçãoGarraA_Fechada);
            garraB.setPosition(RobotConfiguration.posiçãoGarraB_Fechada);

        } else {
            garraA.setPosition(RobotConfiguration.posiçãoGarraA_Solta);
            garraB.setPosition(RobotConfiguration.posiçãoGarraB_Solta);
        }

        if(gamepad2.right_trigger > 0.0) {
            double movimento_mao = MathUtils.Clamp(gamepad2.right_trigger,
                    RobotConfiguration.posiçãoInicialMão, RobotConfiguration.posiçãoLimiteInferiorMão);
            mao.setPosition(movimento_mao);

        } else {
            mao.setPosition(RobotConfiguration.posiçãoInicialMão);
        }

        telemetry.addData("Motor DF", motors[0].getPower());
        telemetry.addData("Motor DT", motors[1].getPower());
        telemetry.addData("Motor EF", motors[2].getPower());
        telemetry.addData("Motor ET", motors[3].getPower());

        RobotLogger.debugControles(telemetry, gamepad1, gamepad2);
        /*

        telemetry.addLine("ATUADORES: Estátisticas");
        telemetry.addData("Mão - Posição:", mao.getPosition());

        telemetry.addData("braçoA - posição:", atuadores[2].getCurrentPosition());
        telemetry.addData("braçoA - força:", atuadores[2].getPower());
        telemetry.addData("braçoA - velocidade:", atuadores[2].getVelocity());
        telemetry.addData("braçoA - corrente:", atuadores[2].getCurrent(CurrentUnit.MILLIAMPS));

        telemetry.addData("braçoB - posição:", atuadores[3].getCurrentPosition());
        telemetry.addData("braçoB - força:", atuadores[3].getPower());
        telemetry.addData("braçoB - velocidade:", atuadores[3].getVelocity());
        telemetry.addData("braçoB - corrente:", atuadores[3].getCurrent(CurrentUnit.MILLIAMPS));

        telemetry.addData("centro - posição:", atuadores[0].getCurrentPosition());
        telemetry.addData("centro - força:", atuadores[0].getPower());
        telemetry.addData("centro - velocidade:", atuadores[0].getVelocity());
        telemetry.addData("centro - corrente:", atuadores[0].getCurrent(CurrentUnit.MILLIAMPS));
        */
    }
}