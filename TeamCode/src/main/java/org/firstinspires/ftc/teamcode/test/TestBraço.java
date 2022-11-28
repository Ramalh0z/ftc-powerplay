package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.RobotLogger;

import java.util.List;

@TeleOp(name = "Test Braço")
public class TestBraço extends OpMode {
    private final DcMotorEx[] atuadores = new DcMotorEx[4];
    private Servo garraA;
    private Servo garraB;
    private Servo mao;

    private List<LynxModule> hubs;

    static final int CORE_HEX_TICKS = 288;
    static final int HD_HEX_TICKS = 1120;

    @Override
    public void init() {
        // configura os bulk reads para um "cache" de valores automático
        // para mais informações, veja: https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        hubs = hardwareMap.getAll(LynxModule.class);

        atuadores[0] = hardwareMap.get(DcMotorEx.class,"centro");   // porta 0 - rev2
        atuadores[1] = hardwareMap.get(DcMotorEx.class,"puxa");     // porta 1 - rev2
        atuadores[2] = hardwareMap.get(DcMotorEx.class,"braçoA");   // porta 2 - rev2
        atuadores[3] = hardwareMap.get(DcMotorEx.class,"braçoB");   // porta 3 - rev2

        // Confgurando os servos
        garraA = hardwareMap.get(Servo.class, "garraA");
        garraB = hardwareMap.get(Servo.class, "garraB");
        mao    = hardwareMap.get(Servo.class, "mão");

        atuadores[2].setDirection(DcMotorEx.Direction.REVERSE);
        mao.setDirection(Servo.Direction.REVERSE);

        //garraA.setDirection(Servo.Direction.REVERSE);
        garraB.setDirection(Servo.Direction.REVERSE);

        for (DcMotorEx atuador : atuadores) {
            atuador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            atuador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        atuadores[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atuadores[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atuadores[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        atuadores[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        // Gira a base do robô
        atuadores[0].setPower(-gamepad2.right_stick_x / 1.4);

        // Levantar o braço
        atuadores[2].setPower(-gamepad2.left_stick_y * 0.9);
        atuadores[3].setPower(-gamepad2.left_stick_y * 0.9);

        // girar a mão
        if(gamepad2.y) {
            mao.setPosition(0.2);
        } else if(gamepad2.a) {
            mao.setPosition(-0.2);
        } else {
            mao.setPosition(0.15);
        }

        if(gamepad2.x) { // no 1 ele se abre
            garraA.setPosition(-0.5);
            garraB.setPosition(0.6);
        } else { // no zero ele fecha
            garraA.setPosition(0.0);
            garraB.setPosition(1.0);
        }

        //int target_posA = (CORE_HEX_TICKS / 4) + atuadores[2].getTargetPosition();
        //int target_posB = (CORE_HEX_TICKS / 4) + atuadores[3].getTargetPosition();

        /*
        if(gamepad2.right_bumper) {

            atuadores[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            atuadores[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            atuadores[2].setTargetPosition( target_posA );
            atuadores[3].setTargetPosition( target_posB );

            atuadores[2].setPower(1.0);
            atuadores[3].setPower(1.0);

            atuadores[3].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            atuadores[2].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if(gamepad2.left_bumper) {

            atuadores[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            atuadores[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            atuadores[2].setTargetPosition( -target_posA );
            atuadores[3].setTargetPosition( -target_posB );

            atuadores[2].setPower(-1.0);
            atuadores[3].setPower(-1.0);

            atuadores[3].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            atuadores[2].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
         */

        RobotLogger.debugMotorPower(telemetry, atuadores);
        RobotLogger.debugMotorPosition(telemetry, atuadores);

        telemetry.addLine("SERVOS \n");
        telemetry.addData("Posição - GarraA: ", garraA.getPosition());
        telemetry.addData("Posição - GarraB: ", garraB.getPosition());
        telemetry.addData("Posição - Mão: ", mao.getPosition());
    }
}
