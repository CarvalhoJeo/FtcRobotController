package org.firstinspires.ftc.teamcode.prototipagemControl;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareClassProtipagem {

    /*
     * =============================================================================
     *                                MOTORES
     * =============================================================================
     */

    public DcMotor motorEsquerda, motorDireita, motorEsquerdaTras, motorDireitaTras; //Motores do expansion de baixo
    public DcMotor motorP0, motorP1, motorP2, motorP3; //Motores do expansion de cima

    /*
     * =============================================================================
     *                                 SERVOS
     * =============================================================================
     */

    public Servo servoP0,servoP1,servoP2,servoP3,servoP4,servoP5; //Servos do expansion de cima
    //public CRServo servoP0,servoP1,servoP2,servoP3,servoP4,servoP5; //Servos Continuos do expansion de cima


    //Referências de hardware e gyro
    static BNO055IMU imu;
    HardwareMap hwMap   =  null;

    public void hardwareGeral(HardwareMap ahwMap) {
        //Referência hardware
        hwMap = ahwMap;

        /*
         * =============================================================================
         *                                  GYRO
         * =============================================================================
         */

        //Configura o gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        //Imu no Drive Station
        imu = hwMap.get(BNO055IMU.class, "imu");

        //Inicializa os parametros do gyro
        imu.initialize(parameters);

        /*
         * =============================================================================
         *                                  ACIONADORES
         * =============================================================================
         */

        //Pega o nome das variáveis no Dv.
        motorEsquerda = hwMap.get(DcMotor.class, "motor_Esquerda");
        motorEsquerdaTras = hwMap.get(DcMotor.class, "motor_EsquerdaTras");
        motorDireita = hwMap.get(DcMotor.class, "motor_Direita");
        motorDireitaTras = hwMap.get(DcMotor.class, "motor_DireitaTras");
        motorP0 = hwMap.get(DcMotor.class, "motor_P0");
        motorP1 = hwMap.get(DcMotor.class, "motor_P1");
        motorP2 = hwMap.get(DcMotor.class, "motor_P2");
        motorP3 = hwMap.get(DcMotor.class, "motor_P3");
        servoP0 = hwMap.get(Servo.class, "servo_P0");
        servoP1 = hwMap.get(Servo.class, "servo_P1");
        servoP2 = hwMap.get(Servo.class, "servo_P2");
        servoP3 = hwMap.get(Servo.class, "servo_P3");
        servoP4 = hwMap.get(Servo.class, "servo_P4");
        servoP5 = hwMap.get(Servo.class, "servo_P5");
        /*
        servoP0 = hwMap.get(CRServo.class, "servo_P0");
        servoP1 = hwMap.get(CRServo.class, "servo_P1");
        servoP2 = hwMap.get(CRServo.class, "servo_P2");
        servoP3 = hwMap.get(CRServo.class, "servo_P3");
        servoP4 = hwMap.get(CRServo.class, "servo_P4");
        servoP5 = hwMap.get(CRServo.class, "servo_P5");
         */

        //Direção dos motores
        motorEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdaTras.setDirection(DcMotor.Direction.REVERSE);
        motorDireitaTras.setDirection(DcMotor.Direction.FORWARD);
        motorP0.setDirection(DcMotor.Direction.FORWARD);
        motorP1.setDirection(DcMotor.Direction.FORWARD);
        motorP2.setDirection(DcMotor.Direction.FORWARD);
        motorP3.setDirection(DcMotor.Direction.FORWARD);
        servoP0.setDirection(Servo.Direction.FORWARD);
        servoP1.setDirection(Servo.Direction.FORWARD);
        servoP2.setDirection(Servo.Direction.FORWARD);
        servoP3.setDirection(Servo.Direction.FORWARD);
        servoP4.setDirection(Servo.Direction.FORWARD);
        servoP5.setDirection(Servo.Direction.FORWARD);
        /*
        servoP0.setDirection(CRServo.Direction.FORWARD);
        servoP1.setDirection(CRServo.Direction.FORWARD);
        servoP2.setDirection(CRServo.Direction.FORWARD);
        servoP3.setDirection(CRServo.Direction.FORWARD);
        servoP4.setDirection(CRServo.Direction.FORWARD);
        servoP5.setDirection(CRServo.Direction.FORWARD);
         */

        //Reseta os encoder, por que são usados em dois OpModes
        motorEsquerda.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerdaTras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireitaTras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Coloca para se mexer contando o encoder
        motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsquerdaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDireitaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
