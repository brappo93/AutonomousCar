#include "mbed.h"
#include "rtos.h"
#include "InterruptIn.h"
#include "MODSERIAL.h"
#include "server-cpp\telemetry.h"
#include "Timer.h"
#include "PID/PID.h"
//#include "FastAnalogIn/FastAnalogIn.h"

/* FRDM */
DigitalOut led_green(LED_GREEN);
DigitalOut led_red(LED_RED);
DigitalOut led_blue(LED_BLUE);

/* MOTOR DRIVE VARIABLES */
PwmOut motor_en(PTA4);         // ENABLE pin
DigitalOut motor_dir(PTB2);    // DIR pin
DigitalOut motor_br(PTE5);     // BRAKEZ pin
InterruptIn motor_speed(PTD5); // FG1 pin

/* VELOCITY CONTROL */
Timer velocity_timer;				//interrupt timer
float max_speed, min_speed, des_speed;	//max, min, desired speed
float curr_spd[3] = {}; //current speed
long numC = 0;	//number of interrupts
double tot_err = 0; //total error
float slow_const = 0.005;
//double Kp = 1.0;
//double Ki = 5.0;
//PID v_pid(Kp, 0.0, 0.0, 0.1);

/* SERVO VARIABLES */
PwmOut steer(PTA12);           // steering servo PWM
//PwmOut misc(PTD4);

/* STEERING VARIABLES */
//unsigned short steer_angles[128]; //map of valid steering angles
unsigned short prev_center = 64;
float s_Kp = 1.5;
float s_Ki = 0.0;
float s_Kd = 0.0;
PID s_pid(s_Kp, (s_Ki==0.0)? 0.0 : s_Kp/s_Ki , s_Kd/s_Kp , 0.008);
Timer s_timer;

/* CAMERA VARIABLES */
/* //Interrupt Camera
AnalogIn cam(PTB0);        // Cam AOUT
DigitalOut cam_si(PTC8);   // Cam SI
PwmOut cam_clk(PTA5);      // Cam CLK
InterruptIn clk_in(PTA13);  // for detecting falling edge of CLK
int cam_clk_count, clk_qt; // clock cycle counter and offset
unsigned short cam_data[128];         // for storing raw camera read data
unsigned short cam_frame[16];        // for saving a frame
unsigned short center = 64;           // for storing the line location
unsigned short line   = 64;           // for storing the line location
//unsigned short numFrames = 0;
*/
// Bit-bang Camera
AnalogIn cam(PTB0);						// Cam AOUT
DigitalOut cam_si(PTC8);			// Cam SI
DigitalOut cam_clk(PTA5);			// Cam CLK
int clk_qt;		// Clock cycle counter and offset
uint16_t cam_data[128]; 			// for storing raw camera read data
uint16_t center = 64;   			// detected line center
uint16_t line   = 64;   			// actual line location
int cam_clk_half_period;			// Period of half a cycle
uint16_t tot_bright;

/* TELEMETRY */
MODSERIAL serial(USBTX, USBRX);
MODSERIAL telemetry_serial(PTA2, PTA1);  // PTA2 as TX, PTA1 as RX
Timer tele_timer;
telemetry::MbedHal telemetry_hal(telemetry_serial);
telemetry::Telemetry telemetry_obj(telemetry_hal);
telemetry::Numeric < uint32_t > tele_time_ms(telemetry_obj, "time", "Time", "ms", 0);
telemetry::NumericArray < uint16_t, 128 > tele_linescan(telemetry_obj, "linescan", "Linescan", "ADC", 0);
telemetry::Numeric < uint16_t > tele_line(telemetry_obj, "line", "Actual Line Location", "Pixels", 0);
telemetry::Numeric < uint16_t > tele_center(telemetry_obj, "center", "Detected Line Center", "Pixels", 0);
telemetry::Numeric < uint16_t > tele_brightness(telemetry_obj, "brightness", "Average Brightness", "Units", 0);
telemetry::Numeric < float > tele_det_speed(telemetry_obj, "speed", "Observed Speed", "m/s", 0);
telemetry::Numeric < float > tele_out_pwm(telemetry_obj, "pwm", "Output PWM", "units", 0);

/* Prototypes */
//Camera
void camera_setup(void);
void dummy_read(void);
void cam_data_thread(void const *args);
//void cam_isr(void);
unsigned short find_line(unsigned short*);
//Motor
void drive_setup(void);
void speed_isr(void);
float estimate_speed(void);
void set_speed(float);
//Steering
void steer_setup(void);
void set_steer(int);
//IO
void flash_led(DigitalOut led);

/* Helpers */
float min(float n1, float n2){
	return n1<n2? n1 : n2;
}
float max(float n1, float n2){
	return n1>n2? n1 : n2;
}
/* Camera Methods */
/* Camera Setup Routine */
void camera_setup() {
	/* //For Interupt Camera
	cam_clk_count = 0;
	clk_qt = 80;
	cam_si.write(0);
	cam_clk.period_us(100);
	cam_clk.write(0.5);
	memset(cam_data, 0, 128*sizeof(unsigned short));
	clk_in.fall(&cam_isr);
	*/
	//For Bit-Bang Camera
	clk_qt = 1; //ms
	cam_si.write(0);
	cam_clk.write(0);
	cam_clk_half_period = 1; // us
	memset(cam_data, 0, 128*sizeof(unsigned short));
}

/* Dummy read for bit-bang camera */
void dummy_read(){
	int i=0;
	
	while(i < 129){
		if (i == 0) {
			cam_si.write(1); // trigger SI to start
		}
		cam_clk.write(1);
		wait_us(cam_clk_half_period);
		if (i == 0){
			cam_si.write(0); // set SI low
		}
		cam_clk.write(0);
		wait_us(cam_clk_half_period);
		i++;
	}
	//wait_us(100);
}

/* Camera Data Aqcuisition Thread */
/* void cam_data_thread(void const *args){ // Old way
	
	while(1){
		if (cam_clk_count == 0) {
			dummy_read();
			Thread::wait(clk_qt);
			cam_si.write(1); // trigger SI to start
		}
		cam_clk.write(1);
		wait_us(cam_clk_half_period);
		
		if (cam_clk_count == 0){
			cam_si.write(0); // set SI low
		}
		cam_clk.write(0);
		wait_us(cam_clk_half_period);

		if(cam_clk_count < 129){
			if (cam_clk_count < 128) 
			{
				cam_data[cam_clk_count] = cam.read_u16(); // read in cam pixel
			}
			else if (cam_clk_count == 128)
			{
				center = find_line(cam_data); // find line
				line = center;
				if ((prev_center>118 || prev_center<10) && (center-prev_center)*(center-prev_center) > 500){
					//then it lost the line
					led_blue= 0.0;
					led_red = 0.0;
					center = prev_center;
				}else{
					led_blue= 1.0;
					led_red = 1.0;
				}
				prev_center = center;
				set_steer(center);
				Thread::wait(2);
			}
			cam_clk_count++;
		}else{
			cam_clk_count = 0; // reset
		}
	}
}*/
void cam_data_thread(void const *args){ // New way
	int i;
	while(1){
		dummy_read();
		Thread::wait(clk_qt);
		cam_si.write(1); // trigger SI to start
		cam_clk.write(1);
		wait_us(cam_clk_half_period);
		cam_si.write(0); // set SI low
		cam_clk.write(0);
		wait_us(cam_clk_half_period);
		tot_bright = 0;
		for(i=0; i<128; i++){
			cam_data[i] = cam.read_u16(); // read in cam pixel
			tot_bright += cam_data[i];
			wait_us(cam_clk_half_period);
			cam_clk.write(1);
			wait_us(cam_clk_half_period);
			cam_clk.write(0);
		}
		center = find_line(cam_data); // find line
		line = center;
		if ((prev_center>118 || prev_center<10) && (center-prev_center)*(center-prev_center) > 500){
			//then it lost the line
			led_blue= 0.0;
			led_red = 0.0;
			center = prev_center;
		}else{
			led_blue= 1.0;
			led_red = 1.0;
		}
		prev_center = center;
		set_steer(center);
		Thread::wait(2);
	}
}
/* Camera Clock Interrupt Routine */
/*void cam_isr() {
	if (cam_clk_count == 0) {
		cam_si.write(1); // trigger SI to start
		cam_clk_count++;
	} else if ((cam_clk_count > 0)&&(cam_clk_count < 129+clk_qt)) {
		if (cam_clk_count == 1) {
			cam_si.write(0); // set SI low
	  }
		if (cam_clk_count < 128) {
			cam_data[cam_clk_count-1] = cam.read_u16(); // read in cam pixel
			
		} else if (cam_clk_count == 128) {
			//memcpy(cam_frame, cam_data, sizeof(int)*128);			// copy frame when done
			center = find_line(cam_data); // find line
			line = center;
			if ((center-prev_center)*(center-prev_center) > 500){
				//then it lost the line
				led_red = 0.0;
				center = prev_center;
			}else{
				led_red = 1.0;
			}
			prev_center = center;
			set_steer(center);
		}
		cam_clk_count++;
	} else {
		cam_clk_count = 0; // reset
	}
}*/
/* Line Finding Algorithm */
unsigned short find_line(unsigned short* data) {
	int max_val = -1, max_index = -1;
	int i;
	for (i = 0; i<prev_center; i++) {
		if (data[i] >= max_val) {
			max_val = data[i];
			max_index = i;
		}
	}for (i = 127; i>=prev_center; i--) {
		if (data[i] >= max_val) {
			max_val = data[i];
			max_index = i;
		}
	}
	return max_index;
}

/* Motor Methods */
/* Motor Driver Setup Routine */
void drive_setup() {
	motor_br.write(1);      // Set HIGH to disable braking
	motor_dir.write(1);     // Set HIGH for forward direction
	motor_en.period_us(40); // 25kHz
	set_speed(0.0);    // start off
	//v_pid.setInputLimits(0.0,10.0);
	//v_pid.setOutputLimits(0.0,0.6);
	//v_pid.setMode(1); //1 = AUTO mode
	//v_pid.setSetPoint(0);
	//v_pid.setBias(0.25);
	des_speed = 0.0;
	//motor_speed.rise(&speed_isr);
	motor_speed.fall(&speed_isr);
}
/* Motor Speed Interrupt Routine */
void speed_isr() {
	//t.stop();
	//int total_t = t.read_us();
	numC++;
	//t.reset();
	//est_spd = (0.003583 / total_t) * 1000000;
	//set_speed(des_spd);
	//adjust_speed(des_spd, est_spd);
	//t.start();
}
/* Estimate current velocity */
float estimate_speed(){
	velocity_timer.stop();
	float time = velocity_timer.read();
	int n = numC;
	numC = 0;
	velocity_timer.reset(); 
	velocity_timer.start();
	if (time == 0.0){
		set_speed(0.125);
		return 0.0;
	}
	// 1 falling edge per 2 "ticks", 35 "ticks" per sheel rotation, each rotation is 0.065*pi meters
	// distance(m) = n edges * (1 ticks / 1 edge)(1 rotation / 35 ticks)(0.065*pi meters / 1 rotation) = n * 0.0058344 //0.011667
	curr_spd[2] = curr_spd[1];
	curr_spd[1] = curr_spd[0];
	curr_spd[0] = n * 0.0001 / time;
	
	float detected_speed = (curr_spd[0] + curr_spd[1] + curr_spd[2]) / 3.0;
	
	//telemetry_serial.printf("%f\n", time);
	
	//float speed = n * 0.0001 / time;
	//float speed = n * 0.0358 / t.read(); //not meters lol
	//float speed = n / time;
	
	if(detected_speed==0.0){
		tot_err = 0;
	}
	
	float error = des_speed - detected_speed;
	

	tot_err+= error;

	float p0= 0.15;
	float p = 0.3 * error;
	float i = 0.01 * tot_err;
	
	float pwm = p + i + p0;
	float inp = pwm < 0.0 ? 0.0 : pwm;
	inp = inp > 0.6 ? 0.6 : inp;
	tele_out_pwm = inp;
	set_speed(inp);
	
	//if(num%10==0){
		//printfNB("Current Speed: %8.5f, Error: %8.5f, Total Error: %8.5f, P Contr: %8.5f, I Contr: %8.5f, PWM: %8.5f\r\n", speed, error, tot_err, p, i, pwm);
	//}
	//num++;
	
	return detected_speed;
}
/* Set Motor Speed */
void set_speed(float spd) {
	spd = spd > 0.6? 0.6 : spd < 0.0? 0.0 : spd;
	motor_en.write(1.0-spd); // SPD should be between 0.0 - 1.0
}

/* Steering Methods */
/* Steering Servo Setup Routine */
void steer_setup() {
	s_pid.setInputLimits(-127, 0);
	s_pid.setOutputLimits(1196, 1704);
	s_pid.setBias(1450);
	s_pid.setMode(1);
	s_pid.setSetPoint(-64);
	//int low = 1196;
	//int high = 1704;
	//1450 is middle
	//int step_size = 4;
	//for (int i=0; i<128; i++){
	//	steer_angles[i] = low + step_size * i;
	//}
	slow_const = 0.008;
	steer.period_ms(20);
	steer.pulsewidth_us(1450); //1200 to 1700 limits left to right; 1450 center0
}
/* Set Steering Angle */
void set_steer(int pos) {
	s_timer.stop();
	float s_time = s_timer.read();
	s_timer.reset();
	s_timer.start();
	if(s_time!=0){
		s_pid.setInterval(s_time);
		s_pid.setProcessValue(-pos);
		int new_steer = (int) floor(s_pid.compute());
		//telemetry_serial.printf("steer: %f\n", new_steer);
		new_steer = new_steer>1704? 1704 : new_steer<1196? 1196 : new_steer;
		steer.pulsewidth_us(new_steer);
		des_speed = max(min_speed, max_speed - abs(64-pos)*slow_const);
	}
	//Thread::wait(100);
}
/* IO */
/* Flash Green LED for diagnostics */
void flash_led(DigitalOut led) {
	led.write(0); // Note that the internal LED is active LOW
	wait(0.25);
	led.write(1);
	wait(0.25);
}

/* Main! */
int main() {
	led_blue = 1.0;
	led_red = 1.0;
	flash_led(led_green);
	
	serial.baud(115200);
  
	//tele_line.set_limits(0,127);
	
	wait(5);
	flash_led(led_green);
	flash_led(led_green);
	telemetry_obj.transmit_header();
	tele_timer.start();
	
	/* Setup routines */
	steer_setup();
	drive_setup();
	camera_setup();

	
	Thread cam_thread(cam_data_thread);
	cam_thread.set_priority(osPriorityAboveNormal);

	
	min_speed = 0.125;
	max_speed = 0.5;
	des_speed = max_speed; //0.125 ~= 1 m/s, 0.7 is VERY FAST OMG
	float curr_speed = 0;
	//float out_pwm = 0;
	
	
	for(int i =0; ; i++){
		for (uint16_t i=0; i<128; i++){
			tele_linescan[i] = cam_data[i];
		}
		tele_brightness = tot_bright/128;
		tele_time_ms = tele_timer.read_ms();
		tele_line = line;
		tele_center=center;
		//v_pid.setSetPoint(des_spd);
		curr_speed = estimate_speed();
		tele_det_speed = curr_speed;
		//v_pid.setProcessValue(curr_speed);
		//out_pwm = v_pid.compute();
		//tele_out_pwm = out_pwm;
		//set_speed(0.2);
		telemetry_obj.do_io();
		Thread::wait(10);
	}
	

	
	Thread::wait(osWaitForever);

}
