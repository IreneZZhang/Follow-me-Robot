/* -------------------------------------------- *
Follow me Robot: 
This project builds a robot that follows people walking in front of it. 

The pinout of the MSP430 is shown below
   Pin Map - MSP430G2553
                ________________
           ----| P1.0     P2.6 |----AIN2
           ----| P1.1     P2.7 |----AIN1
      ECHO1----| P1.2     TEST |----TST
      TRIG1----| P1.3     RSI  |----RSI
      ECHO2----| P1.4     P1.7 |----STBY
      TRIG2----| P1.5     P1.6 |----BIN1
      ECHO3----| P2.0     P2.5 |----BIN2
       PWMA----| P2.1     P2.4 |----PWMB
      TRIG3----| P2.2     P2.3 |----
               |_______________|

 * -------------------------------------------- */

#include <msp430.h>

/* Motor Pins*/
#define PWMA BIT1
#define PWMB BIT4
#define AIN1 BIT7
#define AIN2 BIT6
#define BIN1 BIT6
#define BIN2 BIT5
#define STBY BIT7

/* Sensor Pins */
#define TRIG1 BIT3
#define ECHO1 BIT2
#define TRIG2 BIT5
#define ECHO2 BIT4
#define TRIG3 BIT2
#define ECHO3 BIT0


/* Function Prototypes */
void setup();
void drive(int motor, int direction, int speed);
void backward(int motor1, int motor2, int speed1, int speed2);
void forward(int motor1, int motor2, int speed1, int speed2);
void turnLeft(int direction, int speed);
void turnRight(int direction, int speed);
void set_speed1(int speed);
void set_speed2(int speed);
void stop();
void pause(int unit);


/* Global Variables */
int right_dist, middle_dist, left_dist;
int flag1=0;
int time1=0;

// main function
int distance1();
int distance2();
int distance3();
int t0 = 1;
int t1 = 3;
int t2 = 10;

void setup()
{
    /* Set up timers */
    BCSCTL3 |= LFXT1S_2;                  // ACLK = VLO
    WDTCTL = WDT_ADLY_16;                // WDT, ACLK/8192, interval timer
    IE1 |= WDTIE;                              // Enable WDT interrupt

    /* Clock setup */
    BCSCTL1 = CALBC1_1MHZ;                    // Set range
    DCOCTL = CALDCO_1MHZ;                     // Set DCO step + modulation */
    BCSCTL2 |= DIVS_3;                        // Divide SMCLK by 8

    TACCTL0 = CCIE;                           // CCR0 interrupt enabled
    TACCR0 = 30;                           // Set interrupt flag
    TACTL = TASSEL_2 + MC_1;            // SMCLK, upmode. Divide the clock by 8.

    TA1CTL = TASSEL_1 + MC_1;           // Select ACLK peripheral module
    TA1CCTL1 = OUTMOD_7;                // Select Set/Reset PWM mode
    TA1CCTL2 = OUTMOD_7;
    TA1CCR0 = 20;                       // PWM frequency


    /* Set up pins */
    // Motor Pins
    P1DIR |= STBY + BIN1;
    P2DIR |= PWMA + PWMB + AIN1 + AIN2 + BIN2;
    P1SEL &= ~(STBY + BIN1 + BIT1);    // use pins as GPIO
    P2SEL &= ~(BIN2 + AIN1 + AIN2);

    // Set up PWMs
    P2SEL |= PWMA + PWMB;         // Select primary peripheral module for PWM
    P2SEL2 &= ~(PWMA + PWMB);

    // Ultrasonic Sensor Pins
    P1DIR &= ~(ECHO1 +  ECHO2);
    P1DIR |= TRIG1 + TRIG2;

    P2DIR &= ~ECHO3;
    P2DIR |= TRIG3;

    P1DIR |= BIT0 + BIT1;   // temp led set up
    P2DIR |= BIT3;

}

/* 
 * Set the speed of MotorA 
 *  speed is an positive integer no greater than TA1CCR0
 */
void set_speed1(int speed)
{
    TA1CCR1 = speed;
}

/* Set the speed of MotorB
 *  speed is a positive integer no greater than TA1CCR0
 */
void set_speed2(int speed)
{
    TA1CCR2 = speed;
}

/*
 * Make 1 or both motors go forward with speed chosen 
 * motor1: 1 when motor 1 is selected; 0 when not selected 
 * motor2: 1 when motor 2 is selected; 0 when not selected 
 * speed1: a positive integer no greater than TA1CCR0
 * speed2: a positive integer no greater than TA1CCR0 
 */
void forward(int motor1, int motor2, int speed1, int speed2)
{
    P1OUT |= STBY;
    // Motor A as the argument
    if (motor1 && motor2) {
        P2OUT |= AIN1;
        P2OUT &= ~AIN2;
        P1OUT &= ~BIN1;
        P2OUT |= BIN2;

        TA1CCR1 = speed1;
        TA1CCR2 = speed2;
    } else if (motor1) {
        P2OUT |= AIN1;
        P2OUT &= ~AIN2;

        TA1CCR1 = speed1;
        TA1CCR2 = 0;
    } else if (motor2) {
        P1OUT &= ~BIN1;
        P2OUT |= BIN2;

        TA1CCR2 = speed2;
        TA1CCR1 = 0;
    } else {
        stop();
    }
}

/*
 * Make 1 or both motors go backward with speed chosen 
 * motor1: 1 when motor 1 is selected; 0 when not selected 
 * motor2: 1 when motor 2 is selected; 0 when not selected 
 * speed1: a positive integer no greater than TA1CCR0
 * speed2: a positive integer no greater than TA1CCR0 
 */
void backward(int motor1, int motor2, int speed1, int speed2)
{
    // Enable motor motion 
    P1OUT |= STBY;
    // 
    if (motor1 && motor2) {
        P2OUT &= ~AIN1;
        P2OUT |= AIN2;
        P1OUT |= BIN1;
        P2OUT &= ~BIN2;

        set_speed1(speed1);
        set_speed2(speed2);
    } else if (motor1) {
        P2OUT &= ~AIN1;
        P2OUT |= AIN2;

        set_speed1(speed1);
        set_speed2(0);
    } else if (motor2) {
        P1OUT |= BIN1;
        P2OUT &= ~BIN2;

        set_speed1(0);
        set_speed2(speed2);
    } else {
        stop();
    }
}

/*
 * Make the robot turn left with speed chosen 
 *  direction: 1 for forward direct 0 for backward
 */
void turnLeft(int direction, int speed)
{
    if (direction) {
        forward(1, 1, 0, speed);
    } else {
        backward(1, 1, 0, speed);
    }
}

/*
 * Make the robot turn right with speed chosen 
 *  1 for forward direction 0 for backward
 */
void turnRight(int direction, int speed)
{
    if (direction) {
        forward(1, 1, speed, 0);
    } else {
        backward(1, 1, speed, 0);
    }
}

/*
 * Make the robot stop 
 */
void stop()
{
    P1OUT &= ~STBY;
}

/*
 * Pause for a while
 */
void pause(int unit)
{
    unsigned int i;
    if (unit > 0) {
        for (i = unit; i > 0; i--) {
            __bis_SR_register(LPM3_bits);
        }
    }

}

/* 
 * Send pulse from the first sensor and measure the response 
 */ 
int distance1()
{
    // Send 20us trigger pulse

    P1OUT &= ~TRIG1;
    //__bis_SR_register(LPM0_bits);
    time1 = 0;
    while (time1 < 2);

    P1OUT |= TRIG1;
    //__bis_SR_register(LPM0_bits);
    time1 = 0;
    while (time1 < 2);
    P1OUT &= ~TRIG1;

    // Wait for start of echo pulse
    while ((P1IN & ECHO1) == 0);

    // Measure how long the pulse is
    int d = 0;
    while ((P1IN & ECHO1) > 0)
    {
        // The following delay was worked out by trial and error
        // so that d counts up in steps corresponding to 1cm
        time1 = 0;
        while (time1 < 2);
        d = d + 1;
        if (d >= 400) 
            break;
    }
    return d;
}

/* 
 * Send pulse from the second sensor and measure the response 
 */ 
int distance2()
{
    // Send 20us trigger pulse
    P1OUT &= ~TRIG2;
    //__bis_SR_register(LPM0_bits);
    time1 = 0;
    while (time1 < 2); 

    P1OUT |= TRIG2;
    //__bis_SR_register(LPM0_bits);
    time1 = 0;
    while (time1<2); 
    P1OUT &= ~TRIG2;

    // Wait for start of echo pulse
    while ((P1IN & ECHO2) == 0);

    // Measure how long the pulse is
    int d = 0;
    while ((P1IN & ECHO2) > 0) {
        // The following delay was worked out by trial and error
        // so that d counts up in steps corresponding to 1cm
        time1=0;
        while (time1<2){}
        d = d + 1;
        if (d >= 400) break;
        flag1++;
    }

    return d;
}

/* 
 * Send pulse from the third sensor and measure the response 
 */ 
int distance3()
{
    // Send 20us trigger pulse
    P2OUT &= ~TRIG3;
    time1 = 0;
    while (time1 < 2); 

    P2OUT |= TRIG3;
    time1=0;
    while(time1<2){}
    P2OUT &= ~TRIG3;

    // Wait for start of echo pulse
    while ((P2IN & ECHO3) == 0); 

    // Measure how long the pulse is
    int d = 0;
    while((P2IN & ECHO3) > 0) {
        // The following delay was worked out by trial and error
        // so that d counts up in steps corresponding to 1cm
        time1 = 0;
        while (time1 < 2) {}
        d = d + 1;
        if (d >= 400) break;
    }

    return d;
}

int main(void)
{
     setup();

    __bis_SR_register(GIE);             // Enable interrupts

    while (1) {

        middle_dist = distance2();
        left_dist = distance3();
        right_dist = distance1();


        if (middle_dist < t0) {
            backward(1, 1, 10, 10);
            pause(3);
        }
        else if (middle_dist < t1) {
            stop();
            pause(5);

        } else if (middle_dist < t2) {
            forward(1, 1, 10, 10);
            pause(5);

        } else {  // out of bound
            if (left_dist < t2 && right_dist > t2) {
                turnLeft(1, 7);
                pause(5);
            } else if (left_dist > t2 && right_dist < t2) {
                turnRight(1, 7);
                pause(5);
            } else if (left_dist < t1 && right_dist > t2) {
                turnRight(0, 7);
                pause(5);
            } else if (left_dist > t2 && right_dist < t1) {
                turnLeft(0, 7);
                pause(5);
            } else if (left_dist > t2 && right_dist < t2) {
                backward(1, 1, 10, 10);
                pause(5);
            } else {
                stop();
            }
        }

        time1=0;
        while (time1 < 50) {}

    }
}

// Watchdog Timer interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) watchdog_timer (void)
#else
#error
#endif
{
    __bic_SR_register_on_exit(LPM3_bits);
}

// Timer A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
    time1++;
}
