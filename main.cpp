#include "mbed.h"

// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
#include "pm2_drivers/DebounceIn.h"

// servo driver
#include "pm2_drivers/Servo.h"

// DC motordrivers
#include "pm2_drivers/DCMotor.h"

#include "pm2_drivers/FastPWM/FastPWM.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(USER_BUTTON); // create DebounceIn object to evaluate the user button
                                     // falling and rising edge
void toggle_do_execute_main_fcn();   // custom function which is getting executed when user
                                     // button gets pressed, definition below

// function declaration, definition at the end
float ir_sensor_compensation(float ir_distance_mV);

// set up states for state machine
    enum RobotState {
        INITIAL,
        EXECUTION,
        SLEEP,
        EMERGENCY
    } robot_state = RobotState::INITIAL;



// main runs as an own thread
int main()
{
    

    // attach button fall function address to user button object, button has a pull-up resistor
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 1000; // define main task period time in ms e.g. 20 ms, there for
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    //Creating servo object
    Servo servo_D0(PB_D0);
    Servo servo_D1(PB_D1);

    // create object to enable power electronics for the DC motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

//--- motor M1: Open-loop ---
    // FastPWM pwm_M1(PB_PWM_M1); // create FastPWM object to command motor M1
    // pwm_M1.write(0.75f); // apply 6V to the motor

  //--- motor M2: Close-loop object declearions ---
     const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    //                                 // 6.0f V if you only use one battery pack
    // const float gear_ratio_M2 = 78.125f; // gear ratio
    // const float kn_M2 = 180.0f / 12.0f;  //  [rpm/V]
    // it is assumed that only one motor is available, there fore
    // we use the pins from M1, so you can leave it connected to M1
    // DCMotor motor_M2(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M2, kn_M2, voltage_max);
    // enable the motion planner for smooth movement
    
    // limit max. velocity to half physical possible velocity
    //  motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    //  motor_M2.enableMotionPlanner(true);
    //  // limit max. acceleration to half of the default acceleration
    // motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);

// ---------- motor M3: closed loop for position control ------------
    const float gear_ratio_M3 = 100.0f; // gear ratio
    const float kn_M3 = 140.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, there fore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor motor_M3(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M3, kn_M3, voltage_max);
    // enable the motion planner for smooth movement
    motor_M3.enableMotionPlanner(true);
    // limit max. velocity to half physical possible velocity
    motor_M3.setMaxVelocity(motor_M3.getMaxPhysicalVelocity() * 0.5f);

    

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // Reely S-0090 (Calibrated by me)
    float servo_D0_ang_min = 0.035f; // carefull, these values might differ from servo to servo
    float servo_D0_ang_max = 0.12f;
    // reely S0090 (Not calibrated)
    float servo_D1_ang_min = 0.0325f;
    float servo_D1_ang_max = 0.1175f;

    // servo.setNormalisedPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setNormalisedPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);

    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button, you
                                    // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
                                   // is a defined potential

    // led on nucleo board
    DigitalOut user_led(USER_LED);


    // ir distance sensor (we are using this inplace of Ultrasonic sensor)
    float ir_distance_mV = 0.0f; // define a variable to store measurement (in mV)
    float ir_distances_cm = 0.0f;
    float ir_distance_cm = 0.0f; //to handle possible mathhs error
    AnalogIn ir_analog_in(PC_2); // create AnalogIn object to read in the infrared distance sensor
                             // 0...3.3V are mapped to 0...1

    // min and max IR sensor reading, (ir_distance_min, ir_distance_max) -> (servo_min, servo_max)
    float ir_distance_min = 4.0f;
    float ir_distance_max = 40.0f;

    // initialize servo with the input value
    float servo_input = 0.0f;
    int servo_counter = 0; // define servo counter, this is an additional variable
                       // used to command the servo
    const int loops_per_seconds = static_cast<int>(ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

    // additional led
    // create DigitalOut object to command extra led, you need to add an aditional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via a resistor
    DigitalOut led1(PB_9);

    // start timer
    main_task_timer.start();
    

    // this loop will run forever
    while (true) {
        main_task_timer.reset();
        // print to the serial terminal
        // printf("IR distance mV: %f \n", ir_distance_mV);
        // printf("IR distance mV: %f IR distance cm: %f \n", ir_distance_mV, ir_distance_cm);
        // printf("Pulse width: %f \n", servo_input);
        // printf("Motor velocity: %f \n", motor_M2.getVelocity());
        printf("Motor position: %f \n", motor_M3.getRotation());

       

        if (do_execute_main_task) {

            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;
            

            // read analog input
            ir_distance_mV = 1.0e3f * ir_analog_in.read() * 3.3f;
            ir_distances_cm = ir_sensor_compensation(ir_distance_mV);

            // enable hardwaredriver DC motors: 0 -> disabled, 1 -> enabled
            enable_motors = 1; // setting this once would actually be enough

            // Test that the default motor driver does not activate the motion planner
            // motor_M2.setVelocity(motor_M2.getMaxVelocity() * 0.8f); // set speed setpoint to half physical possible velocity
            motor_M3.setRotation(3.0f);

            // read us sensor distance, only valid measurements will update us_distance_cm
           
            if (ir_distances_cm > 0.0f) {
                ir_distance_cm = ir_distances_cm;
            }

             // enable the servos (commented off because it is now defined in machine state)
            // if (!servo_D0.isEnabled())
            //     servo_D0.enable();
            // if (!servo_D1.isEnabled())
            //     servo_D1.enable();

            // command the servos
            // servo_D0.setNormalisedPulseWidth(servo_input);
            // servo_D1.setNormalisedPulseWidth(servo_input);

            //<------- state machine ------>
            switch (robot_state) {
                case RobotState::INITIAL:
                    printf("INITIAL\n");
                     // enable the servo
                    if (!servo_D0.isEnabled())
                        servo_D0.enable();
                    robot_state = RobotState::EXECUTION;

                    break;
                case RobotState::EXECUTION:
                    printf("EXECUTION\n");
                    // function to map the distance to the servo movement (us_distance_min, us_distance_max) -> (0.0f, 1.0f)
                    servo_input = (ir_distance_cm - ir_distance_min) / (ir_distance_max - ir_distance_min);
                    // values smaller than 0.0f or bigger than 1.0f ar constrained to the range (0.0f, 1.0f) in setNormalisedPulseWidth
                    servo_D0.setNormalisedPulseWidth(servo_input);

                     // if the mechanical button is pressed go to EMERGENCY
                    if (mechanical_button.read()) {
                        robot_state = RobotState::EMERGENCY;
                    }

                    break;

                case RobotState::SLEEP:
                    printf("SLEEP\n");
                    // if the measurement is within the min and max limits go to EXECUTION
                    if ((ir_distance_cm  > ir_distance_min) && (ir_distance_cm < ir_distance_max)) {
                        robot_state = RobotState::EXECUTION;
                    }


                    break;

                case RobotState::EMERGENCY:
                    printf("EMERGENCY\n");
                    // the transition to the emergency state causes the execution of the commands contained
                    // in the outer else statement scope, and since do_reset_all_once is true the system undergoes a reset
                    toggle_do_execute_main_fcn();

                    break;
                default:
                    break;
            }

            // calculate inputs for the servos for the next cycle
            if ((servo_input < 1.0f) &&                     // constrain servo_input to be < 1.0f
                (servo_counter % loops_per_seconds == 0) && // true if servo_counter is a multiple of loops_per_second
                (servo_counter != 0))                       // avoid servo_counter = 0
                servo_input += 0.005f;
            servo_counter++;

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;

                // IR sensor reset
                ir_distance_mV = 0.0f;
                ir_distances_cm = 0.0f;
                robot_state = RobotState::INITIAL;
                // motor_M2.setVelocity(motor_M2.getMaxVelocity() * 0.2f);


                // Servo reset
                servo_D0.disable();
                servo_D1.disable();
                servo_input = 0.0f;
            }
        }

        // toggling the user led
        user_led = !user_led;

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}

float ir_sensor_compensation(float ir_distance_mV)
{
    // insert values that you got from the MATLAB file
    static const float a = 2.574e+04f;
    static const float b = -29.37f;

    // avoid division by zero by adding a small value to the denominator
    if (ir_distance_mV + b == 0.0f)
        ir_distance_mV -= 0.001f;

    return a / (ir_distance_mV + b);
}