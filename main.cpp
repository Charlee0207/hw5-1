#include "mbed.h"
#include "bbcar.h"
#include <cmath>
#include <vector>
#include <queue>
#include <iostream>


#define CAR_WIDTH 11.0
#define WHEEL_DIAMETER 6.5
#define PI 3.14159
#define CAR_ANGLE_2_SINGLE_WHEEL_ANGLE (((CAR_WIDTH)*2)/(WHEEL_DIAMETER)) // CAR_WIDTH*2pi * (carAngle/360) = WHEEL_DIAMETER*pi * (wheelAngle/360)
#define CAR_ANGLE_2_BOTH_WHEEL_ANGLE ((CAR_WIDTH)/(WHEEL_DIAMETER)) // CAR_WIDTH*2pi * (carAngle/360) = WHEEL_DIAMETER*pi * (wheelAngle/360)
#define SERVO_OFFSET 40

#define SENSOR_2_AXIS 26.0
#define SENSOR_2_WHEEL 56.5 
#define BOUNDARY_THRESHOLD 1.2
#define SLIT_ANGLE 15
#define FILTER_SIZE 5

DigitalOut led1(LED1);
Ticker servo_ticker;
Ticker servo_feedback_ticker;
PwmIn servo0_f(D10), servo1_f(D13);
PwmOut servo0_c(D11), servo1_c(D12);
DigitalInOut pin8(D8);
BBCar car(servo0_c, servo0_f, servo1_c, servo1_f, servo_ticker, servo_feedback_ticker);

float minVerticalDistance=(float)0x3f3f3f3f, preDistance=0, curDistance=0;
float factor = CAR_ANGLE_2_SINGLE_WHEEL_ANGLE;
int preAngle=-0x3f3f3f3f, curAngle;
int pattern[4] = {1, 1, 1, 1};

void BOTHspin360(int angle){      
    car.servo0.targetAngle = CAR_ANGLE_2_BOTH_WHEEL_ANGLE*angle + car.servo0.angle;
    car.servo1.targetAngle = CAR_ANGLE_2_BOTH_WHEEL_ANGLE*angle + car.servo1.angle;  
}

int BOTHcheckspin360(int errorAngle_Range){
    int offset0=0, offset1=0, factor=1;  
    int speed0=0, speed1=0;

    int errorAngle0 = car.servo0.targetAngle - car.servo0.angle;
    int errorAngle1 = car.servo1.targetAngle - car.servo1.angle;

    bool continueCheck = false;

    if( abs(errorAngle0)>(int)(errorAngle_Range) ){
        if(errorAngle0 > 0)         offset0 = SERVO_OFFSET;
        else if(errorAngle0 < 0)    offset0 = -SERVO_OFFSET;
        else                        offset0 = 0;    

        speed0 = errorAngle0*6.5*3.14/360;
        continueCheck = true;

        car.servo0.set_factor(factor);
        car.servo0.set_speed(speed0 + offset0); 
    }
    else{
        car.servo0.set_factor(factor);
        car.servo0.set_speed(speed0 + offset0); 
    }

    if( abs(errorAngle1)>(int)(errorAngle_Range) ){
        if(errorAngle1 > 0)         offset1 = SERVO_OFFSET;
        else if(errorAngle1 < 0)    offset1 = -SERVO_OFFSET;
        else                        offset1 = 0;    

        speed1 = errorAngle1*6.5*3.14/360;
        continueCheck = true;
        
        car.servo1.set_factor(factor);
        car.servo1.set_speed(speed1 + offset1);
    }
    else{  
        car.servo1.set_factor(factor);
        car.servo1.set_speed(speed1 + offset1); 
    }


    if(continueCheck) return 1;  
    else return 0;
}  
 
void SINGLEspin360(int angle, int servoNum=1){      
    if(servoNum==0) car.servo0.targetAngle = CAR_ANGLE_2_SINGLE_WHEEL_ANGLE*angle + car.servo0.angle;
    else            car.servo1.targetAngle = CAR_ANGLE_2_SINGLE_WHEEL_ANGLE*angle + car.servo1.angle;  
}

int SINGLEcheckspin360(int errorAngle_Range, int servoNum=1){
    int offset0=0, offset1=0, factor=1;  
    int speed0=0, speed1=1;
    bool continueCheck = false;

    if(servoNum==0){
        int errorAngle0 = car.servo0.targetAngle - car.servo0.angle;
        if( abs(errorAngle0)>(int)(errorAngle_Range) ){
            if(errorAngle0 > 0)         offset0 = SERVO_OFFSET;
            else if(errorAngle0 < 0)    offset0 = -SERVO_OFFSET;
            else                        offset0 = 0;    

            speed0 = errorAngle0*6.5*3.14/360;
            car.servo0.set_factor(factor);
            car.servo0.set_speed(speed0 + offset0);
            continueCheck = true;
        }
    }
    else{
        int errorAngle1 = car.servo1.targetAngle - car.servo1.angle;
        if( abs(errorAngle1)>(int)(errorAngle_Range) ){
            if(errorAngle1 > 0)         offset1 = SERVO_OFFSET;
            else if(errorAngle1 < 0)    offset1 = -SERVO_OFFSET;
            else                        offset1 = 0;    

            speed1 = errorAngle1*6.5*3.14/360;
            car.servo1.set_factor(factor);
            car.servo1.set_speed(speed1 + offset1); 
            continueCheck = true;
        }
    }
    if(continueCheck) return 1;  
    else return 0;
}  

int main() {
    parallax_laserping  ping(pin8);
    vector<int> Spin = {-40, 45, 40, -45};
    queue<float> Pop;
    float Filter=0;
    int Sample=0;
    ThisThread::sleep_for(500ms); // Wating for BBcar constructor finish enqueue

    for(int i=0; i<FILTER_SIZE; i++){
        Pop.push(ping.laserping_cm());
        Filter += Pop.back();
    }; 


    for(int i=0; i<500; i++){
        Pop.push(ping.laserping_cm());
        Filter = Filter + Pop.back() - Pop.front();
        Pop.pop();
        curDistance = (Sample==0) ? (Filter / FILTER_SIZE) : curDistance;
        Sample = (Sample==FILTER_SIZE-1) ? 0 : Sample + 1;

        minVerticalDistance = (curDistance<minVerticalDistance) ? curDistance : minVerticalDistance;
    } 
    cout << "=========================\n";
    cout << "minVerticalDistance: " << minVerticalDistance << "\n";
    cout << "=========================\n";


    int originAngle = car.servo0.angle;
    int leftMostAngle = 0, rightMostAngle = 0; 
    float leftMostCarAngle, rightMostCarAngle;
    float slitWidth;
    SINGLEspin360(Spin[0], 0);
    while(SINGLEcheckspin360(1, 0)){};
    car.stop();
    ThisThread::sleep_for(500ms);


    SINGLEspin360(Spin[1], 0);
    while(SINGLEcheckspin360(1, 0)){
        Pop.push(ping.laserping_cm());
        Filter = Filter + Pop.back() - Pop.front();
        Pop.pop();
        preDistance = (Sample==0) ? curDistance : preDistance;
        curDistance = (Sample==0) ? Filter / FILTER_SIZE : curDistance;
        
        Sample = (Sample==FILTER_SIZE-1) ? 0 : Sample + 1;
        

        if(curDistance-preDistance>BOUNDARY_THRESHOLD){
            preAngle = curAngle;
            curAngle = car.servo0.angle;
            int deltaAngle = curAngle - originAngle;
            if(curAngle-preAngle>SLIT_ANGLE && deltaAngle>-180){
                if(deltaAngle<-70) pattern[0] = 0;
                else pattern[1] = 0;
                leftMostAngle = (deltaAngle<leftMostAngle) ? deltaAngle : leftMostAngle;
                cout << "==========Detected==========\n";
            }
        }
        ThisThread::sleep_for(1ms);
    }
    car.stop();
    ThisThread::sleep_for(500ms);

    originAngle = car.servo1.angle;       
    preAngle=-0x3f3f3f3f;

    SINGLEspin360(Spin[2], 1);
    while(SINGLEcheckspin360(1, 1)){};
    car.stop();
    ThisThread::sleep_for(500ms);

    
    SINGLEspin360(Spin[3], 1);
    while(SINGLEcheckspin360(1, 1)){
        Pop.push(ping.laserping_cm());
        Filter = Filter + Pop.back() - Pop.front();
        Pop.pop();
        preDistance = (Sample==0) ? curDistance : preDistance;
        curDistance = (Sample==0) ? Filter / FILTER_SIZE : curDistance;
        
        Sample = (Sample==FILTER_SIZE-1) ? 0 : Sample + 1;
        

        if(curDistance-preDistance>BOUNDARY_THRESHOLD){
            preAngle = curAngle;
            curAngle = car.servo1.angle;
            int deltaAngle = curAngle - originAngle;
            if(preAngle-curAngle>SLIT_ANGLE && deltaAngle<180){ 
                if(deltaAngle>60) pattern[3] = 0;
                else pattern[2] = 0;
                cout << "==========Detected==========\n";
                rightMostAngle = (deltaAngle>rightMostAngle) ? deltaAngle : rightMostAngle;
            }
            
        }
        ThisThread::sleep_for(1ms);
    }
    car.stop();
    leftMostCarAngle = ((float)(-leftMostAngle))/factor;  rightMostCarAngle = ((float)(rightMostAngle))/factor;
    slitWidth = (minVerticalDistance+2.7) *(fabs(tanf(leftMostCarAngle*(PI/180.0))) + fabs(tanf(rightMostCarAngle*(PI/180.0))));


    cout << "||";
    for(int i=0; i<4; i++){
        cout << "  " << pattern[i] << "  ||";
    }
    cout << "\n";  
    cout << "slitWidth: " << slitWidth << "\n";   

    
     

}                                             