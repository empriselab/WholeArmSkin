  #include <ros.h>
 #include <ros/time.h>
 #include <std_msgs/UInt16.h>
 #include <wholearm_skin_ros/TaxelData.h>

 // ROS taxel interface
 ros::NodeHandle nh;
 
 wholearm_skin_ros::TaxelData msg;
 ros::Publisher pub("skin/taxels", &msg);

#define NUM_TAXELS 28

//Port definitions
int ctr_A = 4 ; //mannual ctr_A to pin 2
int ctr_B = 3 ; //mannual ctr_B to pin 3
int ctr_C = 2 ; //mannual ctr_C to pin 4

int sig_Ctr_A = 7 ; //manual pin 5
int sig_Ctr_B = 6 ;
int sig_Ctr_C = 5 ; //manual pin 7

int muxValues[8][8] = {{0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0}};

void saveData(int Chn, int Aout){
  muxValues[Chn][Aout] = 1024-analogRead(A0);
}

// #define APP_RECV_BUFFER_SIZE      100

// int App_RecvLen = 0;
// bool App_RcvdFirstFlag = false;
// bool App_RecvFinishFlag = false;
// char App_AckCommand = 0;

void setup() {
  //put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pub);
  Serial.begin(115200);
  while(!Serial);
  delay(10);

  //Initialize control ports
  pinMode(ctr_A, OUTPUT);
  pinMode(ctr_B, OUTPUT);
  pinMode(ctr_C, OUTPUT);
  pinMode(sig_Ctr_A, OUTPUT);
  pinMode(sig_Ctr_B, OUTPUT);
  pinMode(sig_Ctr_C, OUTPUT);

  digitalWrite(ctr_A, HIGH);
  digitalWrite(ctr_B, HIGH);
  digitalWrite(ctr_C, HIGH);
  digitalWrite(sig_Ctr_A, HIGH);
  digitalWrite(sig_Ctr_B, HIGH);
  digitalWrite(sig_Ctr_C, HIGH);

  //Adc ports initialize
  AdcInit();
}

void loop() {
  uint16_t c[NUM_TAXELS];
  // put your main code here, to run repeatedly:
   for (int i = 0; i < 4; i++)
   {
    SetChn(i);
    for (int j = 0; j < 8; j++){
      SetAout(j); // choose an input pin on the 74HC4067
      saveData(i,j);
      c[i*8+j] = muxValues[i][j];
    }
  }
//  delay(300);
//  displayData();
  msg.cdc = c;
  msg.cdc_length = NUM_TAXELS;
  msg.header.stamp = nh.now();
  pub.publish(&msg);
  nh.spinOnce();
}

boolean SetChn(int chn)
{
  switch (chn)
  {
    case 0:
      digitalWrite ( ctr_A , HIGH ) ;
      digitalWrite ( ctr_B , HIGH ) ;
      digitalWrite ( ctr_C , HIGH ) ;
      break;
    case 1:
      digitalWrite ( ctr_A , HIGH ) ;
      digitalWrite ( ctr_B , HIGH ) ;
      digitalWrite ( ctr_C , LOW ) ;
      break;
    case 2:
      digitalWrite ( ctr_A , HIGH ) ;
      digitalWrite ( ctr_B , LOW ) ;
      digitalWrite ( ctr_C , HIGH ) ;
      break;
    case 3:
      digitalWrite ( ctr_A , HIGH ) ;
      digitalWrite ( ctr_B , LOW ) ;
      digitalWrite ( ctr_C , LOW ) ;
      break;
    case 4:
      digitalWrite ( ctr_A , LOW ) ;
      digitalWrite ( ctr_B , HIGH ) ;
      digitalWrite ( ctr_C , HIGH ) ;
      break;
    case 5:
      digitalWrite ( ctr_A , LOW ) ;
      digitalWrite ( ctr_B , HIGH ) ;
      digitalWrite ( ctr_C , LOW ) ;
      break;
    case 6:
      digitalWrite ( ctr_A , LOW ) ;
      digitalWrite ( ctr_B , LOW ) ;
      digitalWrite ( ctr_C , HIGH ) ;
      break;
    case 7:
      digitalWrite ( ctr_A , LOW ) ;
      digitalWrite ( ctr_B , LOW ) ;
      digitalWrite ( ctr_C , LOW ) ;
      break;
    default:
      return false;
      break;
  }
  return true;
}

boolean SetAout(int outBit)
{
  switch (outBit)
  {
    case 0:
      digitalWrite ( sig_Ctr_A , HIGH ) ;
      digitalWrite ( sig_Ctr_B , HIGH ) ;
      digitalWrite ( sig_Ctr_C , HIGH ) ;
      break;
    case 1:
      digitalWrite ( sig_Ctr_A , HIGH ) ;
      digitalWrite ( sig_Ctr_B , HIGH ) ;
      digitalWrite ( sig_Ctr_C , LOW ) ;
      break;
    case 2:
      digitalWrite ( sig_Ctr_A , HIGH ) ;
      digitalWrite ( sig_Ctr_B , LOW ) ;
      digitalWrite ( sig_Ctr_C , HIGH ) ;
      break;
    case 3:
      digitalWrite ( sig_Ctr_A , HIGH ) ;
      digitalWrite ( sig_Ctr_B , LOW ) ;
      digitalWrite ( sig_Ctr_C , LOW ) ;
      break;
    case 4:
      digitalWrite ( sig_Ctr_A , LOW ) ;
      digitalWrite ( sig_Ctr_B , HIGH ) ;
      digitalWrite ( sig_Ctr_C , HIGH ) ;
      break;
    case 5:
      digitalWrite ( sig_Ctr_A , LOW ) ;
      digitalWrite ( sig_Ctr_B , HIGH ) ;
      digitalWrite ( sig_Ctr_C , LOW ) ;
      break;
    case 6:
      digitalWrite ( sig_Ctr_A , LOW ) ;
      digitalWrite ( sig_Ctr_B , LOW ) ;
      digitalWrite ( sig_Ctr_C , HIGH ) ;
      break;
    case 7:
      digitalWrite ( sig_Ctr_A , LOW ) ;
      digitalWrite ( sig_Ctr_B , LOW ) ;
      digitalWrite ( sig_Ctr_C , LOW ) ;
      break;
    default:
      return false;
      break;
  }  
  return true;
}

void AdcInit()
{
  for(int i = 0; i < 8; i++)
  {
    pinMode(A0 + i, INPUT);
    digitalWrite(A0 + i, LOW);
  }
}

void displayData()
// dumps captured data from array to serial monitor
{
  Serial.println();
  //Serial.println("Values from multiplexer:");
   Serial.println("========================");
   for (int i = 0; i < 8; i++)
   {
    for (int j = 0; j < 8; j++){
//       Serial.print("I"); 
//       Serial.print(i); 
//       Serial.print(j);
//       Serial.print(" = "); 
      Serial.print(muxValues[i][j]);
      Serial.print(" ");
    }
    Serial.println();
   }
   Serial.println("========================");  
}

void ReadAdc(int adcChn)
{
  int adcVal = analogRead(A0 + adcChn);
  Serial.println(adcVal);
}
