 #include <Wire.h>

#define DEVICE (0x53)    //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)

byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the device//char str[512]; //string buffer to transform data before sending it to the serial port
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define L3G4200D_Address 105

int xg, yg, zg ;
double x_real , y_real , z_real , yaw ,dt,theta,timer,output ;
int motor1[]={3,5};      // controlling motor 1
int motor2[]={9,6};      // controlling motor 2
int en[]={10,11};        // controlling speed
                    
void setup()
{  Serial.print("0");
  Serial.begin(9600);


  Wire.begin();        // join i2c bus (address optional for master)
    // start serial for output
  Serial.print("0");
  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
    Serial.print("0");

  initialize(2000);   // range of gyro , can be chaqnged to 200 or 500 or 2000.
  //timer=millis();
  Serial.print("0");
  
   for(int i=0;i<=1;i++){
  pinMode(motor1[i],OUTPUT);
  pinMode(motor2[i],OUTPUT);     // declaration of pins
  pinMode(en[i],OUTPUT);
  }
  Serial.print("0");
}

void loop()
{ float sum=0;
int x_out, y_out, z_out, x_real, y_real, z_real;
float x,y,z,alpha;

  for(int k=0;k<=19;k++){
  int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
  
  
  
  readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
  
   //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
   //thus we are converting both bytes in to one int
  x_out = (((int)buff[1]) << 8) | buff[0];   
  y_out = (((int)buff[3])<< 8) | buff[2];
  z_out = (((int)buff[5]) << 8) | buff[4];
  
  
  x=x_out/256.0;
  y=y_out/256.0;
  z=z_out/256.0;
  alpha = atan2((sqrt(pow (x,2)+pow(y,2))),z);
  alpha = (alpha*180)/3.14;
  alpha=180-alpha; // angle from acclerometer
  
 xg = getX();
 x_real = xg*(.07); // multiplying by senstivity of gyro which depends on the scale choosen see datasheet page no 10.
  
 yg = getY();
 y_real = yg*(.07);

 zg = getZ();
 z_real = zg*(.07);
 
//yaw = yaw + y_real*dt ; // angle from gyro

float ang[50];

ang[k]=Complementary(alpha,y_real,timer);
 sum=sum+ang[k];}
theta=sum/20;

if (x >0) {theta = -theta ;}
else {theta = theta;}   
output=Compute(theta);
if(output<0){output=-output;}
if(output>=255){output=255;}// calling the PID controller function

int speed;
if (theta >-4 && theta< 4){speed=2;}
else {speed = output;}


 // if motor falling forward
  //if(((theta>0) && (y_real<0)) || ((theta<0 && theta>-5) && (y_real<0))){
    if (theta>0){
    digitalWrite(motor2[0],HIGH);
    digitalWrite(motor2[1],LOW);
    digitalWrite(motor1[0],HIGH);
    digitalWrite(motor1[1],LOW);
    analogWrite(en[0],speed);
    analogWrite(en[1],speed);
    }
    
  //   if(((theta < 0) && (y_real>0)) || ((theta>0 && theta<5) && (y_real>0)) ){
    if (theta<0){
    digitalWrite(motor2[0],LOW);
    digitalWrite(motor2[1],HIGH);
    digitalWrite(motor1[0],LOW);
    digitalWrite(motor1[1],HIGH);
    analogWrite(en[0],speed);
    analogWrite(en[1],speed);
    }
}

//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}
int  getX(){
  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  return ((xMSB << 8) | xLSB);
}

int  getY(){
  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  return ((yMSB << 8) | yLSB);
}

int  getZ(){
  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  return ((zMSB << 8) | zLSB);
}

void  initialize(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void  writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int  readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}

// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Complementary(float newAngle, float newRate,int looptime) {
float dtC = float(looptime)/1000.0;
float x_angleC=0;
float tau=0.075;
float a=0.0;
timer=millis(); 
a=tau/(tau+dtC);
x_angleC= a* (x_angleC + newRate * dtC) + (1-a) * (newAngle);
return x_angleC;
}
// Calculates the PID output
double Compute(double theta)
{
  // PID variables

float lasttheta = 0;
double ITerm =0;

// PID constants
// You can change this values to adjust the control
float kp = 17;  // Proportional value
float ki = 8;           // Integral value
float kd = 12;           // Derivative value

float Setpoint = -2.2;     // Initial setpoint is 0
      float error = theta-Setpoint;
      if(error<5 && error >5){kp=30;}
      Serial.print(" error: ");
      Serial.print(error);
      ITerm+= (ki * error);
      if(ITerm > 255) ITerm= 255;
      else if(ITerm < 0) ITerm= -ITerm;
      float dtheta = (theta - lasttheta);
 
      // Compute PID Output
      float output = kp * error + ITerm + kd * dtheta;

	  
      // Remember some variables for next time
      lasttheta = theta;
      return output;
      
   
}
