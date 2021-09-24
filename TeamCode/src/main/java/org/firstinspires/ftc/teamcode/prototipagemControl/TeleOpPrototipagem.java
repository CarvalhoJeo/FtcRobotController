package org.firstinspires.ftc.teamcode.prototipagemControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Teleoperado to test", group="Linear TesteOp")
public class  TeleOpPrototipagem extends LinearOpMode{

    //Instanciação de objetos
    ElapsedTime runtime = new ElapsedTime();
    HardwareClassProtipagem hard = new HardwareClassProtipagem();

    //Referência de oritenação do gyro
    Orientation angles;

    //Respectivamente eixos do gamepad y, x, x outro analógico
    double drive, turn, giro;
    double pi = 3.1415926;
    double power = 0;

    //Vetor para potência do motor
    private final double[] poder = new double[4];

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Inicia o hardware do robô
        hard.hardwareGeral(hardwareMap);

        hard.motorEsquerda.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hard.motorDireita.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hard.motorEsquerdaTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hard.motorDireitaTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();

        //Espera o botão start na DS
        waitForStart();

        while (opModeIsActive()) {

            /*
             * =============================================================================
             *                            FIELD ORIENTED DRIVE
             * =============================================================================
             */

            //Variáveis gamepad
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x * 1.5;
            giro = gamepad1.right_stick_x;

            //Calculo para field oriented
            processamentoGame(drive, turn);

            //Valores para movimentação com mechanum (lados espelhados)
            //Motor Esquerda Frente;
            poder[0] = drive + turn + giro;
            //Motor Esquerda trás;
            poder[1] = drive - turn + giro;
            //Motor Direita Frente;
            poder[2] = drive - turn - giro;
            //Motor Direita trás;
            poder[3] = drive + turn - giro;

            //Verificar se algum valor é maior que 1
            if (Math.abs(poder[0]) > 1 || Math.abs(poder[1]) > 1
                    || Math.abs(poder[2]) > 1 || Math.abs(poder[3]) > 1) {

                //Achar o maior valor
                double max;
                max = Math.max(Math.abs(poder[0]), Math.abs(poder[1]));
                max = Math.max(Math.abs(poder[2]), max);
                max = Math.max(Math.abs(poder[3]), max);

                //Não ultrapassar +/-1 (proporção);
                poder[0] /= max;
                poder[1] /= max;
                poder[2] /= max;
                poder[3] /= max;
            }

            //Metodo setPower que manda força para os motores.
            hard.motorEsquerda.setPower(poder[0]);
            hard.motorEsquerdaTras.setPower(poder[1]);
            hard.motorDireita.setPower(poder[2]);
            hard.motorDireitaTras.setPower(poder[3]);

            /*
             * =============================================================================
             *                          CONTROLE DE VELOCIDADE
             * =============================================================================
             */

            while (power < 1 && power > -1){
                if (gamepad1.dpad_left){
                    power =- 0.1;
                }else if(gamepad1.dpad_right){
                    power =+ 0.1;
                }
                if (power > 1){
                    power = 1;
                }
            }
            if (power > 1){
                power = 1;
            }else if(power < -1){
                power = -1;
            }
            
            /*
             * =============================================================================
             *                                MOTORES
             * =============================================================================
             */

            while (!gamepad1.dpad_up){
                if(gamepad1.a){
                    hard.motorP0.setPower(power);
                }else if(gamepad1.b){
                    hard.motorP1.setPower(power);
                }else if(gamepad1.x){
                    hard.motorP2.setPower(power);
                }else if(gamepad1.y){
                    hard.motorP3.setPower(power);
                }
            }

            /*
             * =============================================================================
             *                                SERVOS
             * =============================================================================
             */

            //Programação para servo limitado

            while (gamepad1.dpad_up){
                if(gamepad1.a){
                    hard.servoP0.setPosition(1);
                }else if(gamepad1.b){
                    hard.servoP1.setPosition(1);
                }else if(gamepad1.x){
                    hard.servoP2.setPosition(1);
                }else if(gamepad1.y){
                    hard.servoP3.setPosition(1);
                }else if(gamepad1.left_bumper){
                    hard.servoP4.setPosition(1);
                }else if(gamepad1.right_bumper){
                    hard.servoP4.setPosition(1);
                }else {

                    hard.servoP0.setPosition(0);
                    hard.servoP1.setPosition(0);
                    hard.servoP2.setPosition(0);
                    hard.servoP3.setPosition(0);
                    hard.servoP4.setPosition(0);
                    hard.servoP5.setPosition(0);
                }
            }

            //Programação para servo continuo

            /*
            while (gamepad1.dpad_up){
                if(gamepad1.a){
                    hard.servoP0.setPower(power);
                }else if(gamepad1.b){
                    hard.servoP1.setPower(power);
                }else if(gamepad1.x){
                    hard.servoP2.setPower(power);
                }else if(gamepad1.y){
                    hard.servoP3.setPower(power);
                }else if(gamepad1.left_bumper){
                    hard.servoP4.setPower(power);
                }else if(gamepad1.right_bumper){
                    hard.servoP5.setPower(power);
                }else {
                    hard.servoP0.setPower(0);
                    hard.servoP1.setPower(0);
                    hard.servoP2.setPower(0);
                    hard.servoP3.setPower(0);
                    hard.servoP4.setPower(0);
                    hard.servoP5.setPower(0);
                }
            }*/

        }
    }

    private void processamentoGame ( double driveP, double turnP){
        double angle = gyroCalculate() * pi / 180;
        drive = driveP * Math.cos(angle) - turnP * Math.sin(angle);
        turn = driveP * Math.sin(angle) + turnP * Math.cos(angle);
    }

    private double gyroCalculate () {
        angles = HardwareClassProtipagem.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
