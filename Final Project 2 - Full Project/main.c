#include "nu32dip.h"
// include other header files here
#include "encoder.h"
#include "utilities.h"
#include "current_control.h"
#include "ina219.h"
#include "i2c_master_noint.h"
#include "position_control.h"


#define BUF_SIZE 200
#define SAMPLE_TIME 6 // set by Nick
#define MAX_MESSAGE_LENGTH 200

// helper functions
void OC1_Startup();
void TMR3_Startup();
void TMR4_Startup();
void ADC_Startup(); // taken from pic32_oscope code
unsigned int adc_sample_convert(int pin); // taken from pic32_oscope code

// static volatile int curr_control_counter;
// static volatile float eint = 0;
// volatile int curr_control_counter = 0;

// volatile float current_Kp = 0;
// volatile float current_Ki = 0;

volatile float current_needed = 0; // for position control

int final_len = 1000;

volatile int pos_length;

volatile float Waveform[1000]; // waveform
volatile int posArray[1000];


volatile float ref_current_array[100];
volatile float actual_current_array[1000];

volatile float actual_motor_angle[1000];
volatile float ref_motor_angle[1000];

// for position control
static volatile float eprev = 0;
static volatile float eint_position = 0.0;

// for current control
static volatile float eint = 0.0;
static volatile float reference = 200.0;

// Interupts
void __ISR(_TIMER_3_VECTOR, IPL5SOFT) Current_Control(void) {
  static volatile int curr_control_counter = 0;
  // float eint = 0;
  // static volatile float eint = 0.0;
  // static volatile float reference = 200.0;
  switch (get_mode()) {
    case PWM:
    {
      // cur_pwm = get_pwm();
      // duty cycle and direction bit are set according to the value -100 to 100 specified by user
      if (cur_pwm < 0) { // want to flip it so that it's positive and motor changes direction
        DIRECTION = 1;
        // LATAbits.LATA1 = 0; // was originally initialized as 1
        OC1RS = (int) ((float) cur_pwm/-100.0 * 2400);
        // OC1RS = -2400 * cur_pwm/100; // invert what it was originally
        
      }
      else {// pwm is zero or positive
        // cur_pwm = get_pwm();
        // NU32DIP_WriteUART1("here");
        DIRECTION = 0;
        //LATAbits.LATA1 = 1;
        OC1RS = (int) ((float) cur_pwm/100.0 * 2400);
      }
      break;
    }
    case IDLE:
    {
      // PWM duty cycle and direction bit put H-bridge in brake mode
      OC1RS = 0; // never goes high (always low) so motor never gets current and turns on
      break;
    }
    case ITEST:
    {
      if ((curr_control_counter == 25) || (curr_control_counter == 50) || (curr_control_counter == 75)) {
        reference = -(reference);
      }

      // add values to arrays for future plotting
      ref_current_array[curr_control_counter] = reference;
      actual_current_array[curr_control_counter] = INA219_read_current();

      //float current_cur = INA219_read_current();
      volatile float err = ref_current_array[curr_control_counter] - actual_current_array[curr_control_counter];
      eint = eint + err; // calculating integral of error

      // ant-windup
      if (eint > 2000.0) {
          eint = 2000.0;
      }
      else if (eint < -2000.0) {
          eint = -2000.0;
      }

      // Finding Next U
      float u = current_Kp * err + current_Ki * eint;
      float unew = u;
      if (u > 100.0) {
          u = 100.0;
      }
      else if (u < -100.0) {
        u = -100.0;
      }

      if (u < 0) { // want to flip it so that it's positive and motor changes direction
        DIRECTION = 1;
        OC1RS = (int) ((float) (u/-100.0) * 2400);
      }
      else {// pwm is zero or positive
        DIRECTION = 0;
        OC1RS = (int) ((float) (u/100.0) * 2400);
      }

      if (curr_control_counter == 99) {
        reference = 200.0;
        curr_control_counter = 0;
        eint = 0;
        set_mode(IDLE);
      }
      curr_control_counter += 1;
      break;
    }
    case HOLD:
    {
      float reference_current = current_needed;
      float actual_current = INA219_read_current();

      //float current_cur = INA219_read_current();
      volatile float err = reference_current - actual_current;
      eint = eint + err; // calculating integral of error

      // ant-windup
      if (eint > 2000.0) {
          eint = 2000.0;
      }
      else if (eint < -2000.0) {
          eint = -2000.0;
      }

      // Finding Next U
      float u = current_Kp * err + current_Ki * eint;
      float unew = u;
      if (u > 100.0) {
          u = 100.0;
      }
      else if (u < -100.0) {
        u = -100.0;
      }

      if (u < 0) { // want to flip it so that it's positive and motor changes direction
        DIRECTION = 0;
        OC1RS = (int) ((float) (u/-100.0) * 2400);
      }
      else {// pwm is zero or positive
        DIRECTION = 1;
        OC1RS = (int) ((float) (u/100.0) * 2400);
      }

      break;
    }
    case TRACK:
    {
      float reference_current = current_needed;
      float actual_current = INA219_read_current();

      //float current_cur = INA219_read_current();
      volatile float err = reference_current - actual_current;
      eint = eint + err; // calculating integral of error

      // ant-windup
      if (eint > 2000.0) {
          eint = 2000.0;
      }
      else if (eint < -2000.0) {
          eint = -2000.0;
      }

      // Finding Next U
      float u = current_Kp * err + current_Ki * eint;
      float unew = u;
      if (u > 100.0) {
          u = 100.0;
      }
      else if (u < -100.0) {
        u = -100.0;
      }

      if (u < 0) { // want to flip it so that it's positive and motor changes direction
        DIRECTION = 0;
        OC1RS = (int) ((float) (u/-100.0) * 2400);
      }
      else {// pwm is zero or positive
        DIRECTION = 1;
        OC1RS = (int) ((float) (u/100.0) * 2400);
      }

      break;
    }
  }
  // clear interrupt flag
  IFS0bits.T3IF = 0;
}

void __ISR(_TIMER_4_VECTOR, IPL4SOFT) Position_Control(void) {
  static volatile int traj_pos_counter = 0;
  switch(get_mode()) {
    // static volatile float eprev = 0;
    // static volatile float eint_position = 0.0;
    case HOLD:
    {
      // read encoder count
      WriteUART2("a"); 
      while (!get_encoder_flag()) {
        ;
      }
      set_encoder_flag(0);
      int enc_count = get_encoder_count();
      // convert to degrees
      float actual_ang = (float) enc_count * (360.0/384); // 96 lines, 4 states per line so total resolution is 360/384

      // calculate angle difference / change needed to match desired angle
      float err_position = desired_angle - actual_ang;
      // NU32DIP_WriteUART1("here");

      // need edot for PID control
      // Kp*e + KI*eint + Kd*edot
      float edot = err_position - eprev;
      eint_position = eint_position + err_position;

      // ant-windup
      if (eint_position > 2000.0) {
          eint_position = 2000.0;
      }
      else if (eint_position < -2000.0) {
          eint_position = -2000.0;
      }

      // char r[50];
      // sprintf(r, "%f\r\n",err);
      // NU32DIP_WriteUART1(r);

      current_needed = position_Kp*err_position + position_Ki*eint_position + position_Kd*edot;

      
      // char n[50];
      // sprintf(n, "%f\r\n",err);
      // NU32DIP_WriteUART1(n);

      eprev = err_position;
      break;
    }
    case TRACK:
    {
      static int counter = 0;
      float u;
      float pos_Kp, pos_Ki, pos_Kd;
      float pos = 0;
      // float traj_pos_ref = (float) Waveform[traj_pos_counter];

      // PID control
      // read encoder count
      WriteUART2("a"); 
      while (!get_encoder_flag()) {
        ;
      }
      set_encoder_flag(0);
      pos = get_encoder_count();

      float error = Waveform[counter] - (pos * 0.9375);
      // convert to degrees
      // float actual_ang = (float) enc_count * (360.0/384); // 96 lines, 4 states per line so total resolution is 360/384
      // actual_motor_angle[traj_pos_counter] = actual_ang; // saving into array to plot later
      // ref_motor_angle[traj_pos_counter] = traj_pos_ref;

      // calculate angle difference / change needed to match desired angle
      // float err_position = traj_pos_ref - actual_ang;
      // NU32DIP_WriteUART1("here");

      // need edot for PID control
      // Kp*e + KI*eint + Kd*edot
      float edot = error - eprev;
      eint_position = eint_position + error;

      // ant-windup
      if (eint_position > 2000.0) {
          eint_position = 2000.0;
      }
      else if (eint_position < -2000.0) {
          eint_position = -2000.0;
      }

      // char r[50];
      // sprintf(r, "%f\r\n",err);
      // NU32DIP_WriteUART1(r);

      current_needed = position_Kp*error + position_Ki*eint_position + position_Kd*edot;

      posArray[counter] = (int) pos;

      counter += 1;
      
      // char n[50];
      // sprintf(n, "%f\r\n",err);
      // NU32DIP_WriteUART1(n);


      // if done, reset
      if (counter == pos_length) {
        counter = 0;
        desired_angle = pos * 0.9375;
        set_mode(HOLD);
      }

      eprev = error;

      break;
    }
  }
  // clear interrupt flag
  IFS0bits.T4IF = 0;
}



int main() 
{
  char buffer[BUF_SIZE];
  NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32DIP_GREEN = 1;  // turn off the LEDs
  NU32DIP_YELLOW = 1;     

  // setting PIC to IDLE mode
  set_mode(IDLE);
  //NU32DIP_WriteUART1("here1");

  // setting direction pin
  TRISBbits.TRISB12 = 0; // set pin 23 to be an output for direction pin;
  DIRECTION = 0;

  __builtin_disable_interrupts();
  // in future, initialize modules or peripherals here
  UART2_Startup(); // startup UART2 (encoder)
  //ADC_Startup(); // startup ADC
  INA219_Startup(); // startup current sensor
  __builtin_enable_interrupts();
  OC1_Startup(); // for current sensor
  TMR3_Startup(); // for current sensor
  TMR4_Startup(); // for position control
  //NU32DIP_WriteUART1("here2");
  //unsigned int adcval;

  while(1)
  {
    //NU32DIP_WriteUART1("here3");
    NU32DIP_ReadUART1(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32DIP_YELLOW = 1;                   // clear the error LED
    //NU32DIP_WriteUART1("here");
    float current = INA219_read_current(); 
    int adcval = (current/0.86) * 1023; // ADC = current/maxCurrent * 1023, maxCurrent = 2*6/14
    switch (buffer[0]) {
      case 'a': // read current sensor (ADC counts)
        // read the ADC value
        sprintf(buffer, "%d\r\n",adcval);
        NU32DIP_WriteUART1(buffer);
        break;
      case 'b': // Read current sensor (mA)
        sprintf(buffer, "%f\r\n",current);
        NU32DIP_WriteUART1(buffer);
        break;
      case 'c': // read encoder count
        // read encoder count
        WriteUART2("a");
        while (!get_encoder_flag()) {
          ;
        }
        set_encoder_flag(0);
        int p = get_encoder_count();
        sprintf(buffer,"%d\r\n",p);
        NU32DIP_WriteUART1(buffer);
        break;
      case 'd': // Read encoder (deg)
      {
        WriteUART2("a"); // read encoder count
        while (!get_encoder_flag()) {
          ;
        }
        set_encoder_flag(0);
        int p = get_encoder_count();
        int ang = p * (360.0/384); // 96 lines, 4 states per line so total resolution is 360/384
        sprintf(buffer,"%d\r\n",ang);
        NU32DIP_WriteUART1(buffer);
        break;
      }
      case 'e': // reset encoder count
      {
        WriteUART2("b");
        break;
      }
      case 'f': // Set PWM (-100 to 100)
      {
        char m[50];
        NU32DIP_ReadUART1(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &cur_pwm);
        sprintf(m, "%d\r\n", cur_pwm);
        NU32DIP_WriteUART1(m);
        set_mode(PWM);
        break;
      }
      case 'g': // set current Kp and Ki gains
      {
        char m[50];
        // setting Kp
        //NU32DIP_WriteUART1("here1");
        NU32DIP_ReadUART1(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &current_Kp);
        //NU32DIP_WriteUART1("here2");
        sprintf(m, "%f\r\n", current_Kp);
        //NU32DIP_WriteUART1("here3");
        NU32DIP_WriteUART1(m);

        // setting Ki
        NU32DIP_ReadUART1(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &current_Ki);
        sprintf(m, "%f\r\n", current_Ki);
        NU32DIP_WriteUART1(m); 
        break;     
        }
      case 'h': // get current gains
      {
        sprintf(buffer,"%f\r\n",current_Kp);
        NU32DIP_WriteUART1(buffer);
        sprintf(buffer,"%f\r\n",current_Ki);
        NU32DIP_WriteUART1(buffer);
        break;
      }
        // // set_mode(IDLE); // set PIC to PWM mode;
        // set_mode(PWM);
        // char user_input[MAX_MESSAGE_LENGTH];
        // char message_to_user[MAX_MESSAGE_LENGTH];
        // int input_pwm;

        // // sprintf(message_to_user, "Enter PWM value: ");
        // // NU32DIP_WriteUART1(message_to_user);
        // //NU32DIP_WriteUART1("here1");
        // NU32DIP_ReadUART1(user_input, MAX_MESSAGE_LENGTH);
        // //NU32DIP_WriteUART1("here");
        // sscanf(user_input, "%d", &input_pwm);
        // // sprintf(message_to_user, "You entered %d", input_pwm);
        // // NU32DIP_WriteUART1(message_to_user);
        // NU32DIP_WriteUART1("entered pwm");
        // set_pwm(input_pwm);
        // // set_mode(PWM);
      case 'i': // set position gains
      {
        char m[50];
        // setting Kp
        NU32DIP_ReadUART1(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &position_Kp);
        //NU32DIP_WriteUART1("here2");
        sprintf(m, "%f\r\n", position_Kp);
        //NU32DIP_WriteUART1("here3");
        NU32DIP_WriteUART1(m);

        // setting Ki
        NU32DIP_ReadUART1(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &position_Ki);
        sprintf(m, "%f\r\n", position_Ki);
        NU32DIP_WriteUART1(m); 

        // setting Kd
        NU32DIP_ReadUART1(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &position_Kd);
        sprintf(m, "%f\r\n", position_Kd);
        NU32DIP_WriteUART1(m); 
        break;     
      }
      case 'j': // get position gains
      {
        sprintf(buffer,"%f\r\n",position_Kp);
        NU32DIP_WriteUART1(buffer);
        sprintf(buffer,"%f\r\n",position_Ki);
        NU32DIP_WriteUART1(buffer);
        sprintf(buffer,"%f\r\n",position_Kd);
        NU32DIP_WriteUART1(buffer);
        break;
      }
      case 'k': // test current gains
      {
        set_mode(ITEST); // triggers ISR to enter ITEST mode
        // wait while it's in ITEST mode
        while (get_mode() == ITEST) {
          ;
        }
        sprintf(buffer,"100\r\n");
        NU32DIP_WriteUART1(buffer); // python code first wants number of data points to receive
        for(int i = 0; i<=99; i++){
          sprintf(buffer,"%f %f\r\n", ref_current_array[i], actual_current_array[i]); // return the number + 1
          NU32DIP_WriteUART1(buffer);
        }
        set_mode(IDLE); 
        break;
      }
      case 'l': // Go to angle (deg)
      {
        eint_position = 0;
        eprev = 0;

        NU32DIP_ReadUART1(buffer, BUF_SIZE);
        sscanf(buffer, "%d", &desired_angle);
        sprintf(buffer, "%d\r\n", desired_angle);
        NU32DIP_WriteUART1(buffer);
        set_mode(HOLD);
        break;
      }
      case 'm': // load step trajectory
      {
        // //char input_length[100];
        // int input_data;
        // int input_size; 
        // int i;
        // NU32DIP_ReadUART1(buffer, BUF_SIZE);
        // sscanf(buffer, "%d", &input_size);
        // for (i=0; i < input_size; i++) {
        //   NU32DIP_ReadUART1(buffer, BUF_SIZE);
        //   sscanf(buffer, "%d", &input_data);
        //   Waveform[i] = input_data;
        // }
        // // writing back to Python to make sure it's working
        // for (i=0; i < input_size; i++) {
        //   sprintf(buffer, "%d\r\n", Waveform[i]);
        //   NU32DIP_WriteUART1(buffer);
        // }


        char length[50], data[1000];
        NU32DIP_ReadUART1(length, 50);
        int length1 = atoi(length);
        pos_length = length1;
        for (int i = 0; i<pos_length; i++) {
          char datastr[50];
          NU32DIP_ReadUART1(datastr, 10);
          float datapoint = atof(datastr);
          Waveform[i] = datapoint;
        }
        break;
      }
      case 'n': // load cubic trajectory
      {
      //   int input_data;
      //   int input_size; 
      //   int i;
      //   NU32DIP_ReadUART1(buffer, BUF_SIZE);
      //   sscanf(buffer, "%d", &input_size);
      //   for (i=0; i < input_size; i++) {
      //     NU32DIP_ReadUART1(buffer, BUF_SIZE);
      //     sscanf(buffer, "%d", &input_data);
      //     Waveform[i] = input_data;
      //   }
      //   for (i=0; i < input_size; i++) {
      //     sprintf(buffer, "%d\r\n", Waveform[i]);
      //     NU32DIP_WriteUART1(buffer);
      //   }

        char length[50], data[1000];
        NU32DIP_ReadUART1(length, 50);
        int length1 = atoi(length);
        pos_length = length1;
        for (int i = 0; i<pos_length; i++) {
          char datastr[50];
          NU32DIP_ReadUART1(datastr, 10);
          int datapoint = atoi(datastr);
          Waveform[i] = datapoint;
        }
        
        break;
      }
      case 'o': // Execute trajectory
      {
        char message[10000];
        int i = 0;
        eint_position = 0;
        eprev = 0;
        set_mode(TRACK);
        while (get_mode() == TRACK) {
          ;
        }
        for (i=0; i<pos_length; i++) {
          sprintf(message, "%d %d %d\r\n", pos_length-i, posArray[i], (int) Waveform[i]);
          NU32DIP_WriteUART1(message);
        }

        // sprintf(buffer,"%d\r\n", final_len); // return size of vectors
        // NU32DIP_WriteUART1(buffer);

        // set_mode(TRACK);
        
        // while(get_mode() == TRACK){ // wait while getting data
        //   ;
        // }
        
        // for (int i = 0; i < 1000; i++) {
        //   sprintf(buffer, "%f %f\r\n", ref_motor_angle[i], actual_motor_angle[i]); 
        //   NU32DIP_WriteUART1(buffer);
        // }
        // set_mode(IDLE);
        break;
      }
      case 'p': {
        // Unpower the motor
        // OC1RS = 0; // always low -> motor never powered
        set_mode(IDLE);
        break;
      }
      case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here. 
        set_mode(IDLE);
        break;
      }
      case 'r': // get mode
      {
        // char msg[BUF_SIZE];
        // msg[0];
        sprintf(buffer,"%d\r\n",get_mode());
        // if (pic_mode == 0) {
        //   sprintf(buffer,"IDLE\r\n");
        // }
        // else if (pic_mode == 1) {
        //   sprintf(buffer,"PWM\r\n");
        // }
        // else if (pic_mode == 2) {
        //   sprintf(buffer,"ITEST\r\n");
        // }
        // else if (pic_mode == 3) {
        //   sprintf(buffer,"HOLD\r\n");
        // }
        // else if (pic_mode == 4) {
        //   sprintf(buffer,"TRACK\r\n");
        // }
        NU32DIP_WriteUART1(buffer);
        break;
      }
      default:
      {
        NU32DIP_YELLOW = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}

// void ADC_Startup(){
//   ANSELAbits.ANSA1 = 1; // AN1 is an adc pin
//   AD1CON3bits.ADCS = 1; // ADC clock period is Tad = 2*(ADCS+1)*Tpb =2*2*(1/48000000Hz) = 83ns > 75ns
//   AD1CON1bits.ADON = 1;
// }

// unsigned int adc_sample_convert(int pin)
// {
//   unsigned int elapsed = 0, finish_time = 0;
//   AD1CHSbits.CH0SA = pin;
//   AD1CON1bits.SAMP = 1;
//   elapsed = _CP0_GET_COUNT();
//   finish_time = elapsed + SAMPLE_TIME;
//   while (_CP0_GET_COUNT() < finish_time)
//   {
//     ;
//   }
//   AD1CON1bits.SAMP = 0;
//   while (!AD1CON1bits.DONE)
//   {
//     ;
//   }
//   return ADC1BUF0;
// }

void OC1_Startup(void){ // for current controller
  // RPA0Rbits.RPA0R = 0b0101; // OC1 is RPA0 (pin 2)
  RPB15Rbits.RPB15R = 0b0101; // RPB15 is OC1 (pin 26)
  // RPA0Rbits.RPA0R = 0b0101; // A0 is OC1
  //RISAbits.TRISA1 = 0;
  // LATAbits.LATA1 = 0;
  T2CONbits.TCKPS = 0; // Timer2 prescaler 1 
  PR2 = 2400-1; // 48,000,000/1/20000 = 2400
  TMR2 = 0; // initial TMR2 count is 0
  OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 2400*0.25; // duty cycle = OC1RS/(PR2+1) = 25%
  OC1R = 2400*0.25; // initialize before turning OC1 on; afterward it is read-only
  T2CONbits.ON = 1; // turn on Timer2
  OC1CONbits.ON = 1; // turn on OC1
}

void TMR3_Startup(void) // starts up Timer3 (used for current controller - 5 kHz ISR)
{
  T3CONbits.TCKPS = 0; // prescalar = 1
  PR3 = 9600-1;// 48,000,000/1/5,000 = 9600
  TMR3 = 0; // start count at 0
  T3CONbits.ON = 1; // turn on Timer3

  IPC3bits.T3IP = 5;// interrupt priority 5
  IPC3bits.T3IS = 1; // interrupt priority 1
  IFS0bits.T3IF = 0; // clear the int flag
  IEC0bits.T3IE = 1; // enable INT0 by setting IEC0<3>
}

void TMR4_Startup(void) // starts up Timer4 (used for position control)
{
  T4CONbits.TCKPS = 0b010; // prescalar = 4
  PR4 = 60000-1; // 48,000,000/4/200 = 60000
  TMR4 = 0; // start count at 0
  T4CONbits.ON = 1; // turn on Timer 4

  IPC4bits.T4IP = 4; // interrupt priority level 4
  IPC4bits.T4IS = 1; // iterrupt subpriority level 1
  IFS0bits.T4IF = 0; // clear interrupt flag
  IEC0bits.T4IE = 1; // enable INT0 by setting IEC0<3>

}