/* 
 * rosserial Futaba T14SG messege publisher
 * 
 * This example is calibrated for the Sharp GP2D120XJ00F.
 */

#include <ros.h>
#include <sbus.h>             // For receiving the data from Futaba controller

#include <ros/time.h>
#include <sensor_msgs/Joy.h>

#define INPUT_MIN 1100
#define INPUT_MAX 1940
#define INPUT_MED 1520
#define OUTPUT_MIN -1
#define OUTPUT_MAX 1
#define N_CHANNEL 14          // 12 ch custum setting + 2 fixed digital switch
#define LAUNCH_RANGE 0.06f    // if abs(input)<LAUNCH_RANGE: then output=0


// used pins
#define SBUS_PIN 3            // Set D3 pin as 'sbus data pin'
#define ESTOP_PIN 4
#define HEAD_LIGHT_PIN 5
int relay_pins[] = {4, 5, 6, 7};



enum{
    LIGHT_OFF = 0,
    LIGHT_ON = 1,
};

enum{
    CONN_ON = 0,
    CONN_OFF = -1,
    CONN_UNKNOWN = 9
};

ros::NodeHandle    nh;
sensor_msgs::Joy joy_msg;
ros::Publisher pub_joy( "joy", &joy_msg);
char frameid[] = "futaba";


SBUS sbus;

float axes[] = { 0, 0, 0, 0, 0, 0, 0, 0};
int32_t btns[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float rf_data[14] = {0};
char headlight_state;
char connection_state;
char watchdog_cnt;

void setup()
{
    nh.initNode();
    
    nh.advertise(pub_joy);

    joy_msg.header.frame_id =    frameid;
    
    joy_msg.axes_length = sizeof(axes) / sizeof(axes[0]);
    joy_msg.axes = axes;
    joy_msg.buttons_length = sizeof(btns) / sizeof(btns[0]);
    joy_msg.buttons = btns;

    Serial.begin(115200);             // Setup baud rate for ROSserial;
    
    sbus.begin(SBUS_PIN, sbusBlocking); 

    for(int i; i<sizeof(relay_pins)/sizeof(relay_pins[0]); i++)
        pinMode(relay_pins[i], OUTPUT);

    watchdog_cnt = 0;
    headlight_state = 0;
    connection_state = CONN_UNKNOWN;
}


void loop()
{
    watchdog_cnt++;

    if (!sbus.waitFrame()){
        // Hardware connection ERROR: Timeout! 
        for(int i = 0; i < joy_msg.axes_length;i++)
            axes[i] = 0.0;
        for(int i = 0; i < joy_msg.buttons_length;i++)
            btns[i] = 0.0;
        connection_state = CONN_OFF;
        digitalWrite(ESTOP_PIN, LOW);
    }
    else{
        if (sbus.signalLossActive() || sbus.failsafeActive()){
            // Do some protection by sending zero-value joystick msg
            // for(int i = 0; i < joy_msg.axes_length;i++)
            //     axes[i] = 0.0;
            // for(int i = 0; i < joy_msg.buttons_length;i++)
            //     btns[i] = 0.0;
                
            if(connection_state != CONN_OFF){
                // Once lost connection
                connection_state = CONN_OFF;
                digitalWrite(ESTOP_PIN, LOW);

                // joy_msg.header.stamp = nh.now();
                // pub_joy.publish(&joy_msg);
            }
        }
        else{
            watchdog_cnt = 0;    // clear watchdog counter
            for(int i=0; i< sizeof(rf_data)/sizeof(rf_data[0]); i++){
                int ch_data = sbus.getChannel(i+1);
                if(ch_data < INPUT_MIN) rf_data[i] = -1.00;
                else if(ch_data > INPUT_MAX) rf_data[i] = 1.00;
                else{
                    rf_data[i] = (float)(ch_data - INPUT_MIN) *(OUTPUT_MIN - OUTPUT_MAX) / (INPUT_MAX - INPUT_MIN) + OUTPUT_MAX;
                    // Set a fixed range that keep output=0
                    if(abs(rf_data[i]) < LAUNCH_RANGE)
                        rf_data[i] = 0;
                }
            }

            // Joystick setting mapping
            int SA = (int)rf_data[12] > 0;  // SA --> rf_data[12]
                                            // SB --> rf_data[10]
            int SC = (int)rf_data[11];      // SC --> rf_data[11]     
            int SD = (int)rf_data[13] > 0;  // SD --> rf_data[13]

            axes[1] = rf_data[1];           // forward:1, back:-1
            axes[0] = rf_data[3];           // left:1, right:-1
            btns[0] = (SC > 0);             // low speed mode SC:up
            btns[2] = (SC < 0);             // high speed mode SC: down
            
            headlight_state = (SA)? LIGHT_ON : LIGHT_OFF;
            if(SD > 0){
                if(connection_state != CONN_ON){
                    connection_state = CONN_ON;
                    digitalWrite(ESTOP_PIN, HIGH);
                }
                joy_msg.header.stamp = nh.now();
                pub_joy.publish(&joy_msg);
            }
            else{
                digitalWrite(ESTOP_PIN, LOW);
                connection_state = CONN_UNKNOWN;
            }
        }  
    }

    /*switch(headlight_state)
    {
        case LIGHT_ON:
            // for(int i; i<sizeof(relay_pins)/sizeof(relay_pins[0]); i++)
            //    digitalWrite(relay_pins[i], HIGH);
            break;
        case LIGHT_OFF:
            //for(int i; i<sizeof(relay_pins)/sizeof(relay_pins[0]); i++)
            //    digitalWrite(relay_pins[i], LOW);
            break;
        case LOSE_CONN:
            if(watchdog_cnt == 11)
            {
                for(int i; i<sizeof(relay_pins)/sizeof(relay_pins[0]); i++)
                    digitalWrite(relay_pins[i], HIGH);
                //Serial.println("fo");
            }
            else if(watchdog_cnt == 41)
            {
                for(int i; i<sizeof(relay_pins)/sizeof(relay_pins[0]); i++)
                    digitalWrite(relay_pins[i], LOW);
                //Serial.println("lo");
            }
            
            if(watchdog_cnt == 60) watchdog_cnt = 0;
            break;
    }*/

//    Serial.println(watchdog_cnt);
    nh.spinOnce();
    delay(100);
}


