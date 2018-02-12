package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Arib on 10/6/2016.
 */
public class Drivetrain {
    public DcMotor motorBR;
    public DcMotor motorBL;
    public DcMotor motorFR;
    public DcMotor motorFL;


    LinearOpMode opMode;

    public Sensor sensor;

    int nullValue;
    double angleError;

    public boolean reversed;

    private final String LOG_TAG = "DriveTrain";
    public Drivetrain(LinearOpMode opMode)throws InterruptedException {
        this.opMode = opMode;
        nullValue = 0;
        motorBL = this.opMode.hardwareMap.dcMotor.get("BL");
        motorBR = this.opMode.hardwareMap.dcMotor.get("BR");
        motorFL = this.opMode.hardwareMap.dcMotor.get("FL");
        motorFR = this.opMode.hardwareMap.dcMotor.get("FR");
        reversed = false;
        this.opMode.telemetry.addData(LOG_TAG + "init", "finished drivetrain init");
        this.opMode.telemetry.update();
        sensor = new Sensor(opMode);
        this.opMode.telemetry.addData(LOG_TAG + "init", "init finished");
        this.opMode.telemetry.update();
    }

    public void startMotors(double ri, double le) throws InterruptedException {
        if(reversed) {
            motorBL.setPower(-ri);
            motorFL.setPower(ri);
            motorBR.setPower(le);
            motorFR.setPower(-le);
        } else {
            motorBL.setPower(le);
            motorFL.setPower(-le);
            motorBR.setPower(-ri);
            motorFR.setPower(ri);
        }
    }

    public void moveFowardToLine(double ri, double le) throws InterruptedException {
        moveFowardToLine(ri, le, 10000);
    }

    public boolean moveFowardToLine(double ri, double le, int timeout) throws InterruptedException {

        //create time variable for timeout
        ElapsedTime time = new ElapsedTime();
        time.startTime();

        //while we haven't found the line and haven't reached our failsafe
        //failsafe meaning we haven't gone beyond our time limit
        //keep moving at given powers
        while(!sensor.isRightLine() && time.milliseconds() < timeout) {

            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();

            startMotors(ri, le);
            opMode.idle();
        }
        boolean toReturn = false;
        if(time.milliseconds() >= timeout) {
            toReturn = true;
        }
        stopMotors();
        opMode.telemetry.update();
        angleError = sensor.getGyroYaw();
        return toReturn;
    }

    public void moveForwardToWall(double pow, double powTwo, int encoderVal, int timeout, int angleTo) throws InterruptedException {
        //get the current angle and where we start
        double angle = Math.abs(sensor.getGyroYaw());
        double startAngle = angleTo;

        //set the power to the first powers movement
        double power = pow;

        //reset the encoders
        resetEncoders();

        //soft reset the encoders
        setNullValue();

        //create a time variable
        ElapsedTime time = new ElapsedTime();
        time.startTime();

        //set how long we want to accelerate
        int targetTime = 2000;
        int currentEncoder = 0;

        double increaseTick;

        //accelerate for a target time increasing power by percentage
        while(time.milliseconds() < targetTime) {
            increaseTick = (time.milliseconds() / targetTime) / pow;
            angle = Math.abs(sensor.getGyroYaw());

            if(angle < startAngle - 1) {
                startMotors((increaseTick * .6), (increaseTick));
            } else if(angle > startAngle + 1) {
                startMotors((increaseTick), (increaseTick * .6));
            } else {
                startMotors(increaseTick, increaseTick);
            }
            opMode.telemetry.update();
        }

        //move the full distance at the first power
        while(currentEncoder < encoderVal && time.milliseconds() < timeout) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            currentEncoder = getEncoderAvg() - nullValue;

            Range.clip(power, -1, 1);

            if(angle < startAngle - 1) {
                startMotors((power * .6), (power));
            } else if(angle > startAngle + 1) {
                startMotors((power), (power * .6));
            } else {
                startMotors(power, power);
            }
            opMode.idle();
        }

        //give into the wall a little
        while(angle > startAngle - 3) {
            startMotors(.2, 0);
            angle = Math.abs(sensor.getGyroYaw());
        }

        //set new values for correction
        startAngle = startAngle - 3;
        startMotors(power, power);
        power = powTwo;
        time.reset();

        //run into the wall and detect
        while (Math.abs(startAngle - angle) < 10 && time.milliseconds() < timeout) {
            angle = Math.abs(sensor.getGyroYaw());
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();

            if(power > 0) {
                if (angle < Math.abs(startAngle) - 2) {
                    startMotors((power * .6), (power));
                } else if (angle > Math.abs(startAngle) + 2) {
                    startMotors((power), (power * .6));
                } else {
                    startMotors(power, power);
                }
            }
            else if(power < 0) {
                if (angle < Math.abs(startAngle) - 2) {
                    startMotors(((power * .6)), power);
                } else if (angle > Math.abs(startAngle) + 2) {
                    startMotors((power), (power * .6));
                } else {
                    startMotors(power, power);
                }
            }

            opMode.idle();
        }

        stopMotors();

        angleError = sensor.getGyroYaw();
        opMode.telemetry.update();

    }

    public void moveBackwardToWall(double pow, double powTwo, int encoderVal, int timeout, int angleTo) throws InterruptedException {
        double angle = Math.abs(sensor.getGyroYaw());
        double startAngle = angleTo;

        double power = pow;

        resetEncoders();

        setNullValue();

        //create time object
        ElapsedTime time = new ElapsedTime();
        time.reset();
        time.startTime();

        //declare acceleration time
        int targetTime = 2000;
        int currentEncoder = 0;

        double increaseTick;

        //accelerate for the given time
        while(time.milliseconds() < targetTime) {
            increaseTick = (time.milliseconds() / targetTime) / pow;
            angle = Math.abs(sensor.getGyroYaw());

            if(angle < startAngle - 2) {
                startMotors((increaseTick), (increaseTick * .65));
            } else if(angle > startAngle + 2) {
                startMotors((increaseTick * .65), (increaseTick));
            } else {
                startMotors(increaseTick, increaseTick);
            }
            opMode.telemetry.update();
        }

        //move for ethe set distance
        while(encoderVal > currentEncoder && time.milliseconds() < timeout) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            currentEncoder = getEncoderAvg() - nullValue;

            Range.clip(power, -1, 1);

            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();

            if(angle < startAngle - 2) {
                startMotors((power), (power * .6));
            } else if(angle > startAngle + 2) {
                startMotors((power * .6), (power));
            }  else {
                startMotors(power, power);
            }

            opMode.idle();
        }

        //give into the wall for a little bit
        while(angle < startAngle + 5) {
            startMotors(-.2, 0);
            angle = Math.abs(sensor.getGyroYaw());
        }

        //reset variables
        startAngle = 137 + 5;
        power = powTwo;
        startMotors(power, power);
        time.reset();

        //move until we detect the wall
        while (Math.abs(startAngle - angle) < 10 && time.milliseconds() < timeout) {
            angle = Math.abs(sensor.getGyroYaw());
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();

            if(power > 0) {
                if (angle < Math.abs(startAngle) - 2) {
                    startMotors((power * .65), (power));
                } else if (angle > Math.abs(startAngle) + 2) {
                    startMotors((power), (power * .65));
                } else {
                    startMotors(power, power);
                }
            }
            else if(power < 0) {
                if (angle < Math.abs(startAngle) - 1) {
                    startMotors((power), (power * .75));
                } else if (angle > Math.abs(startAngle) + 1) {
                    startMotors((power *.75), (power));
                } else {
                    startMotors(power, power);
                }
            }

            opMode.idle();
        }

        stopMotors();

        angleError = sensor.getGyroYaw();
        opMode.telemetry.update();

    }

    public void moveForwardToWallEnc(double pow, int timeout) throws InterruptedException {
        double angle = Math.abs(sensor.getGyroYaw());
        double startAngle = angle;

        double power = pow;

        int tick = 1;

        resetEncoders();

        int startEncoder = Math.abs(motorFL.getCurrentPosition());

        int endEncoder;

        ElapsedTime time = new ElapsedTime();
        time.startTime();
        while (Math.abs(angle - startAngle) < 10 && time.milliseconds() < timeout) {
            angle = Math.abs(sensor.getGyroYaw());
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();


            if(time.milliseconds() > 250 * tick) {
                endEncoder = Math.abs(motorFL.getCurrentPosition());
                if(startEncoder + 250 < endEncoder) {
                    break;
                }
            }

            if (angle < startAngle - 1) {
                startMotors((power * .75), power);
            } else if (angle > startAngle + 1) {
                startMotors(power, (power * .75));
            } else {
                startMotors(power, power);
            }

            opMode.idle();
        }

        stopMotors();

        angleError = sensor.getGyroYaw();
        opMode.telemetry.update();

    }

    public void basicTurn(double pow, double angle) throws InterruptedException {
        basicArc(pow, -pow, angle);
    }

    public void basicArc(double powR, double powL, double angle) throws InterruptedException {
        double currentAngle = sensor.getGyroYaw();

        if(powR > 0) {
            while(currentAngle < angle) {
                startMotors(powR, powL);
                currentAngle = sensor.getGyroYaw();

                opMode.telemetry.update();
                opMode.idle();
            }
        } else {
            while(currentAngle > angle) {
                startMotors(powR, powL);
                currentAngle = sensor.getGyroYaw();

                opMode.telemetry.update();
                opMode.idle();
            }
        }

        stopMotors();
    }

    public void basicArcB(double powR, double powL, double angle) throws InterruptedException {
        double currentAngle = sensor.getGyroYaw();

        if(powR > 0) {
            while(currentAngle > angle) {
                startMotors(powR, powL);
                currentAngle = sensor.getGyroYaw();

                opMode.telemetry.update();
                opMode.idle();
            }
        } else {
            while(currentAngle < angle) {
                startMotors(powR, powL);
                currentAngle = sensor.getGyroYaw();

                opMode.telemetry.update();
                opMode.idle();
            }
        }

        stopMotors();
    }

    public void moveForwardUntilZero(double pow, double timeout) throws InterruptedException {

        double angle = sensor.getGyroYaw();

        ElapsedTime time = new ElapsedTime();
        time.startTime();
        while(Math.abs(angle) > 2 && time.milliseconds() < timeout) {
            startMotors(-.3, -.5);

            opMode.telemetry.addData("current", "zeroout");
            opMode.telemetry.update();

            opMode.idle();
        }

        stopMotors();
    }

    public void stopMotors() throws InterruptedException {
        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }

    public void setNullValue() {
        nullValue = getEncoderAvg();
    }

    public void reverse() {
        reversed = !reversed;
    }

    public int getEncoderAvg() {
        return (int)((3.0/2.0)*((Math.abs(motorBR.getCurrentPosition())) + Math.abs(motorBL.getCurrentPosition()))) / 4;
    }

    public void resetEncoders() throws InterruptedException {

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();

    }

    public void moveForward(double pow, int encoderVal, double timeout) throws InterruptedException {
        moveForward(pow, pow, encoderVal, timeout);
    }
    public void moveForward(double powR, double powL, int encoderVal, double timeout) throws InterruptedException {
//        sensor.resetGyro();
        double angle;
        double startAngle = Math.abs(sensor.getGyroYaw());
        opMode.telemetry.update();

        double error;
        double power;

        resetEncoders();

        setNullValue();

        opMode.resetStartTime();

        ElapsedTime time = new ElapsedTime();
        time.startTime();

        int currentEncoder = getEncoderAvg() - nullValue;
        while(encoderVal > currentEncoder && time.milliseconds() < timeout) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            if(powL < 0)
                power = (((powL + powR) / 2) * error) - .2;
            else
                power = (((powL + powR) / 2) * error) + .2;

            Range.clip(power, -1, 1);

            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.addData("error", error);
            opMode.telemetry.update();

            startMotors(powR, powL);
            opMode.idle();
        }
        stopMotors();
        opMode.telemetry.update();
        angleError = sensor.getGyroYaw();
    }

    public void moveBackward(double pow, int encoderVal, int timeout, int angleTo) throws InterruptedException {
//        sensor.resetGyro();
        double angle;
        double startAngle = Math.abs(angleTo);
        opMode.telemetry.update();

        double error;
        double power;

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();


        setNullValue();

        nullValue = 0;

        ElapsedTime time = new ElapsedTime();
        time.startTime();

        int currentEncoder = getEncoderAvg() - nullValue;
        while(encoderVal > currentEncoder && time.milliseconds() < timeout) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            if(pow < 0)
                power = (pow * error) - .2;
            else
                power = (pow * error) + .2;


            Range.clip(power, -1, 1);

            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();

            if(angle < startAngle - 2) {
                startMotors((power), (power * .75));
            } else if(angle > startAngle + 2) {
                startMotors((power * .75), (power));
            } else {
                startMotors(power, power);
            }

            startMotors(power, power);
            opMode.idle();
        }
        stopMotors();
        opMode.telemetry.update();
        angleError = sensor.getGyroYaw();
    }

    public void moveBackward(double pow, int encoderVal, int timeout) throws InterruptedException {
//        sensor.resetGyro();
        double angle;
        double startAngle = Math.abs(sensor.getGyroYaw());
        opMode.telemetry.update();

        double error;
        double power;

        resetEncoders();

        setNullValue();

        nullValue = 0;

        ElapsedTime time = new ElapsedTime();
        time.startTime();

        int currentEncoder = getEncoderAvg() - nullValue;
        while(encoderVal > currentEncoder && time.milliseconds() < timeout) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            if(pow < 0)
                power = (pow * error) - .3;
            else
                power = (pow * error) + .3;


            Range.clip(power, -1, 1);

            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();

            if(power > 0) {
                if (angle < startAngle - 1) {
                    startMotors((power * .65), (power));
                } else if (angle > startAngle + 1) {
                    startMotors((power), (power * .65));
                } else {
                    startMotors(power, power);
                }
            }
            else if(power < 0) {
                if (angle < startAngle - 1) {
                    startMotors((power), (power * .65));
                } else if (angle > startAngle + 1) {
                    startMotors((power *.65), (power));
                } else {
                    startMotors(power, power);
                }
            }
            opMode.idle();
        }
        stopMotors();
        opMode.telemetry.update();
        angleError = sensor.getGyroYaw();
    }

    public boolean moveBackwardWithPitchSaftey(double pow, int encoderVal, int timeout) throws InterruptedException {
//        sensor.resetGyro();
        double angle;
        double startAngle = Math.abs(sensor.getGyroYaw());
        opMode.telemetry.update();

        double error;
        double power;

        resetEncoders();

        setNullValue();

        nullValue = 0;

        ElapsedTime time = new ElapsedTime();
        time.startTime();

        int currentEncoder = getEncoderAvg() - nullValue;
        boolean works = true;
        while(encoderVal > currentEncoder && time.milliseconds() < timeout) {
            opMode.telemetry.update();
            angle = Math.abs(sensor.getGyroYaw());

            currentEncoder = getEncoderAvg() - nullValue;

            error = (double) (encoderVal - currentEncoder) / encoderVal;

            if(pow < 0)
                power = (pow * error) - .3;
            else
                power = (pow * error) + .3;


            Range.clip(power, -1, 1);

            opMode.telemetry.addData("Power", power);
            opMode.telemetry.addData("LeftPower", motorBL.getPower());
            opMode.telemetry.addData("RightPower", motorBR.getPower());
            opMode.telemetry.update();

            if(power > 0) {
                if (angle < startAngle - 1) {
                    startMotors((power * .65), (power));
                } else if (angle > startAngle + 1) {
                    startMotors((power), (power * .65));
                } else {
                    startMotors(power, power);
                }
            }
            else if(power < 0) {
                if (angle < startAngle - 1) {
                    startMotors((power), (power * .65));
                } else if (angle > startAngle + 1) {
                    startMotors((power *.65), (power));
                } else {
                    startMotors(power, power);
                }
            }
            opMode.idle();
            if(sensor.getGyroPitch() > 5) {
                stopMotors();
                opMode.telemetry.update();
                works = false;
                break;
            }
        }
        stopMotors();
        opMode.telemetry.update();
        angleError = sensor.getGyroYaw();
        return works;
    }


    public void rotatePReset(double pow, int deg) throws InterruptedException {

        double power = pow;
        double angleTo = deg;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        sensor.resetGyro();

        opMode.resetStartTime();

        currentAngle = 0;

        while(Math.abs(currentAngle) < Math.abs(angleTo) - 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(angleTo) - Math.abs(currentAngle);
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .008) + .15;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0015);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .0225);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            if(angleTo > 0)
                power *= -1;

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);
            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }


    public void rotateP(double pow, int deg) throws InterruptedException {

        double power = pow;
        double angleTo = deg;
        double error;
        double inte = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        currentAngle = 0;

        double kP = .09;
        double kI = .045;
        double kD = 0;
        while(Math.abs(currentAngle) < Math.abs(angleTo) - 1) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(angleTo) - Math.abs(currentAngle);

            power = (pow * (error) * .009) + kP;
            inte = ((opMode.getRuntime()) * kI);
            der = (error - previousError) / opMode.getRuntime() * kD;

            power = power + inte + der;

            if(angleTo > 0)
                power *= -1;

            Range.clip(power, -1, 1);
            startMotors(-power, power);

            opMode.telemetry.addData("error", error);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inte);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePDefense(double pow, int deg) throws InterruptedException {

        double power = pow;
        double angleTo = deg;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(Math.abs(currentAngle) > Math.abs(angleTo + 5)) {
            currentAngle = sensor.getGyroYaw();
            error = currentAngle - angleTo;
            opMode.telemetry.addData("error", error);
            power = (Math.abs(pow) * (error) * .009) + .08;                      //update p values
            inte = ((opMode.getRuntime()) * error * .002);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .045);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power - inteNoE + der;

            if(pow > 0) {
                power *= -1;
            }

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePDefenseB(double pow, int deg) throws InterruptedException {

        double power = pow;
        double angleTo = deg;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(Math.abs(currentAngle) > Math.abs(angleTo + 5)) {
            currentAngle = sensor.getGyroYaw();
            error = currentAngle - angleTo;
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .009) + .08;                      //update p values
            inte = ((opMode.getRuntime()) * error * .002);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .045);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePZero(double pow) throws InterruptedException {

        double power = pow;
        double angleTo = 0;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(Math.abs(currentAngle) > 9) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .01) + .13;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0020);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .03);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            power *= -1;        //-1 is right

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePZeroRev(double pow) throws InterruptedException {

        double power = pow;
        double angleTo = 0;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(Math.abs(currentAngle) > 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .02) + .125;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0020);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .075);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE - der;

            power *= 1;        //-1 is right

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePZeroB(double pow) throws InterruptedException {

        double power = pow;
        double angleTo = 0;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(Math.abs(currentAngle) > 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .025) + .175;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0020);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .075);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            power *= -1;        //-1 is right

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePZeroRevB(double pow) throws InterruptedException {

        double power = pow;
        double angleTo = -180;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;
        double error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        while(error > 6) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(Math.abs(angleTo) - Math.abs(currentAngle));
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .01) + .1;                      //update p values
            inte = ((opMode.getRuntime()) * error * .0015);         //update inte value
            inteNoE = ((opMode.getRuntime()) * .05);
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            power *= 1;        //-1 is right

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        opMode.telemetry.update();
        stopMotors();
    }

    public void rotatePB(double pow, int deg) throws InterruptedException {

        double power = pow;
        double angleTo = deg;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        currentAngle = 0;

        while(Math.abs(currentAngle) < Math.abs(angleTo) - 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(angleTo) - Math.abs(currentAngle);
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .0075) + .05;                   //update p values
            inte = ((opMode.getRuntime()) * error * .007);          //update inte value
            inteNoE = ((opMode.getRuntime()) * .055); //.03
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            if(angleTo > 0)
                power *= -1;

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle + " " + angleTo);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        stopMotors();
        opMode.telemetry.addData("finished", "done");
        opMode.telemetry.update();
    }

    public void rotatePDefenseTwo(double pow, int deg) throws InterruptedException {

        double power = pow;
        double angleTo = deg;
        double error;
        double inte = 0;
        double inteNoE = 0;
        double der;

        double currentAngle = sensor.getGyroYaw();
        double previousError = angleTo - currentAngle;

        opMode.telemetry.addData("Current Angle", currentAngle + "");
        opMode.telemetry.addData("Angle To", angleTo + "");
        opMode.telemetry.update();

        opMode.resetStartTime();

        currentAngle = 0;

        while(Math.abs(currentAngle) < Math.abs(angleTo) - 2) {
            currentAngle = sensor.getGyroYaw();
            error = Math.abs(angleTo) - Math.abs(currentAngle);
            opMode.telemetry.addData("error", error);
            power = (pow * (error) * .0092) + .025;                   //update p values
            inte = ((opMode.getRuntime()) * error * .005);          //update inte value
            inteNoE = ((opMode.getRuntime()) * .03); //.03
            der = (error - previousError) / opMode.getRuntime() * 0; //update der value

            power = power + inteNoE + der;

            if(angleTo > 0)
                power *= -1;

            Range.clip(power, -1, 1);
            startMotors(-power, power);
            opMode.telemetry.addData("PID", power);
//            opMode.telemetry.addData("integral", inte);
            opMode.telemetry.addData("integral without error", inteNoE);
            opMode.telemetry.addData("angle", currentAngle + " " + angleTo);

            opMode.telemetry.update();
            previousError = error;
            opMode.idle();
        }

        stopMotors();
        opMode.telemetry.addData("finished", "done");
        opMode.telemetry.update();
    }

}

