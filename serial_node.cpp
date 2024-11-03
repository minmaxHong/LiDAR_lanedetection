#include <ros/ros.h>
#include <serial/serial.h>
#include <macaron_06/erp_read.h>
#include <macaron_06/erp_write.h>

#define PI acos(-1)
#define MAX 18

serial::Serial ser;

double t_desire = 3;
bool E_stop;
uint8_t steer1,steer2,speed,speed1,brake,gear;
uint8_t steer1_save = 0x00;
uint8_t steer2_save = 0x00;
uint8_t speed1_save = 0x00;
double ENC_saver[4];
double vel_saver[4];
double dt = 0;
double wheel_base = 1.040, tread = 0.985, width = 1.160;
int steer = 0;
int inputbool = 0;

ros::Time now_time;


void writeCallback(const macaron_06::erp_write::ConstPtr& write)
{
    inputbool = 1;
    E_stop = write->write_E_stop;
    gear = write->write_gear;
    steer= write->write_steer;
    steer1 = (steer/256);
    steer2 = (steer%256);
    if(steer<0) steer1=steer1-1;
    speed1 = write->write_speed;
    brake = write->write_brake;
}

int main(int argc, char **argv)
{
    //ROS setting
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::Publisher  serial_pub = nh.advertise<macaron_06::erp_read>("erp_read", 1);
    ros::Subscriber serial_sub = nh.subscribe("erp_write", 1, writeCallback);
    ros::Rate loop_rate(50);
    macaron_06::erp_read erp42_state;
    //serial setting
    try
    {
        ser.setPort("/dev/ttyUSB0");
        //####################################################
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch(const serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    //
    else
    {
        return -1;
    }
    //지역 변수 선언
    uint8_t a;
    uint8_t PCU_to_UPPER[18] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    //                           S  | T |  X  |A/M|Estop|gear|  speed |  steer  |brake|       ENC        |Alive|ETX0|ETX1 
    uint8_t UPPER_to_PCU[14] = {0x53,0x54,0x58,0x01,0x00,gear,0x00,speed1_save,steer1_save,steer2_save,brake,a,0x0D,0x0A};
    //                           S  | T  | X  |A/M|Estop|gear|   speed        |        steer          |brake| |ETX0|ETX1 
    uint8_t answer_tester[1]={0x00};
    uint8_t answer_quere[MAX]={0,};
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    //초기화
    if(ser.available())
    {
        while(answer_tester[0]!=0x53){
            ser.read(answer_tester,1);
            for(int i=0; i<MAX;i++)
            {
                printf("%x ",answer_quere[i]);
            } 
        }
        answer_tester[0]={0x00};
        ser.read(PCU_to_UPPER,17);
        if(PCU_to_UPPER[0]==0x54 && PCU_to_UPPER[1]==0x58 && PCU_to_UPPER[15]==0x0D && PCU_to_UPPER[16]==0x0A)     
        {
            erp42_state.read_AorM=bool(PCU_to_UPPER[2]);
            erp42_state.read_E_stop=bool(PCU_to_UPPER[3]);
            erp42_state.read_gear=PCU_to_UPPER[4];
            erp42_state.read_speed=int(PCU_to_UPPER[5]);
            erp42_state.read_brake=int(PCU_to_UPPER[9]);
            erp42_state.read_ENC=int(PCU_to_UPPER[13])*256*256*256+int(PCU_to_UPPER[12])*256*256+int(PCU_to_UPPER[11])*256+int(PCU_to_UPPER[10]);
            ENC_saver[0]=erp42_state.read_ENC;
            ENC_saver[1]=erp42_state.read_ENC;
            ENC_saver[2]=erp42_state.read_ENC;
            ENC_saver[3]=erp42_state.read_ENC;
        }
    }

    uint8_t answer_quere_delayed[MAX]={0,};
    uint8_t answer_quere_last[MAX]={0,};
    int delayed = -1;
    while(ros::ok())
    {
        //dt 구하기
        last_time = current_time;
        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();
        
        ros::spinOnce();



        //읽은 값 프린트
        if(ser.available()){
            ser.read(answer_quere,18);

            //
            if(answer_quere[0]==0x53 && answer_quere[1]==0x54 && answer_quere[2]==0x58 && answer_quere[16]==0x0D && answer_quere[17]==0x0A)
                for(int i = 0;i < 18; i++)
                    answer_quere_delayed[i] = answer_quere[i];
            else{

                delayed = -1;
                for(int i = 0; i < 18; i++){

                    /*if(answer_quere[i] == 0x53){
                        if(answer_quere[i + 1 / 18] == 0x54 
                            || answer_quere[i + 2 / 18] == 0x58
                            || answer_quere[i + 16 / 18] == 0x0D
                            || answer_quere[i + 17 / 18] == 0x0A)
                            delayed = i;
                    }*/
                    if(answer_quere[i] == 0x53)
                        delayed = i;
                    if(delayed == i)
                        break;
                }
                delayed = 18 - delayed;
                for(int i = 18 - delayed; i < 18; i++)
                    answer_quere_delayed[i - 18 + delayed] = answer_quere_last[i];
                for(int i = 0; i <= 18 - delayed; i++)
                    answer_quere_delayed[i + delayed] = answer_quere[i];

                for(int i = 0; i < 18; i++)
                    answer_quere_last[i] = answer_quere[i];
            }

            for(int i=0; i<MAX;i++)
            {
                printf("%x ",answer_quere_delayed[i]);
            }
            printf("%f \n",dt);
        }

        //throw trash value
        if(answer_quere_delayed[0]!=0x53 || answer_quere_delayed[1]!=0x54 || answer_quere_delayed[2]!=0x58 || answer_quere_delayed[16]!=0x0D || answer_quere_delayed[17]!=0x0A)
        {
            ser.flushOutput();
            for(int i=0; i<MAX;i++)
            {
                answer_quere_delayed[i]={0x00};
            }
            while(answer_tester[0]!=0x0A)
            {
                ser.read(answer_tester,1);
                ROS_INFO("DORMAMU %x",answer_tester[0]);
            }
        }

        //Read to ERP-42!
        else
        {   
            erp42_state.read_AorM=bool(answer_quere_delayed[3]);
            erp42_state.read_E_stop=bool(answer_quere_delayed[4]);
            erp42_state.read_gear=answer_quere_delayed[5];
            erp42_state.read_speed=int(answer_quere_delayed[6]);
            erp42_state.read_brake=int(answer_quere_delayed[10]);
            erp42_state.read_ENC=int(answer_quere_delayed[14])*256*256*256+int(answer_quere_delayed[13])*256*256+int(answer_quere_delayed[12])*256+int(answer_quere_delayed[11]);
            erp42_state.read_steer=int(answer_quere_delayed[9])*256+int(answer_quere_delayed[8]);
            if(erp42_state.read_steer>32768) erp42_state.read_steer=erp42_state.read_steer-65536+1;
            now_time = ros::Time::now();
            printf("read: erp_speed: %d, sec: %d, nsec: %d\n",erp42_state.read_speed,now_time.sec, now_time.nsec);
            // read steer calibration (-)
            erp42_state.read_steer = -erp42_state.read_steer;
            serial_pub.publish(erp42_state);
            ser.flush();
        }

        //Control to ERP-42!
        if(inputbool) 
        {
            a++;
            uint8_t UPPER_to_PCU[14] = {0x53,0x54,0x58,0x01,0x00,gear,0x00,speed1,steer1,steer2,brake,a,0x0D,0x0A};
             //                          S    T    X   A/M Estop gear   speed        steer      brake  ETX0 ETX1 
            ser.write(UPPER_to_PCU,14);
            printf("write: erp_speed: %d, sec: %d, nsec: %d\n", speed1, now_time.sec, now_time.nsec);
            loop_rate.sleep();
        }
        else 
        {
            a++;
            uint8_t UPPER_to_PCU[14] = {0x53,0x54,0x58,0x01,0x00,0x01,0x00,0x00,steer1_save,steer2_save ,0x80,a,0x0D,0x0A};
            //                           S    T    X   A/M Estop gear   speed             steer          brake  ETX0 ETX1 
            ser.write(UPPER_to_PCU,14);
            loop_rate.sleep();
        }
        
    }
    return 0;
}
