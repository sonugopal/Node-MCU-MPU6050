

#include <Wire.h>
#include<SoftwareSerial.h>
#include<ESP8266WiFi.h>
#include<ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <TinyGPS++.h> // library for GPS module

TinyGPSPlus gps;  // The TinyGPS++ object
SoftwareSerial ss(4,5);
String latitude , longitude;
int year , month , date, hour , minute , second;
String date_str , time_str , lat_str , lng_str;
int pm;
// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;
WiFiServer server(80);
//ESP8266WebServer server;
// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D6;
const uint8_t sda = D7;


SoftwareSerial ESP8266WiFi(D1,D2);
char* ssid="Redmi";
char* password="ggggggggg";
// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

void setup() {
  Serial.begin(115200);
   ss.begin(9600);
  Wire.begin(sda,scl);
  MPU6050_Init();
  ESP8266WiFi.begin(4800);
  WiFi.begin(ssid,password);
while(WiFi.status()!=WL_CONNECTED)
{
  Serial.print(".");
  delay(500);
}
Serial.print("");
Serial.print("IP Address:");
Serial.print(WiFi.localIP());
server.begin();
  Serial.println("Server started");
}

void loop() {
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+16.5; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;

  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);
  //=======================================================================
  Serial.println("gps");
  Serial.println(gps.satellites.value());
   latitude = gps.location.lat();
if (ss.available() > 0) 
{//while data is available
  
    if (gps.encode(ss.read())) //read gps data
    {
      if (gps.location.isValid()) //check whether gps location is valid
      {
        latitude = gps.location.lat();
        //lat_str = String(latitude , 6); // latitude location is stored in a string
        longitude = gps.location.lng();
        //lng_str = String(longitude , 6);
        //longitude location is stored in a string
        Serial.print("Latitude:");
        Serial.println(latitude);
      }
        
    }
    else{
      Serial.print("No Gps");
    }
}
 
  //=======================================================================
  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
 
HTTPClient http;  //Declare an object of class HTTPClient
String url="http://192.168.43.107:3000/sensorValues?";
latitude="12.934788";
longitude="77.605884";
String Car="Audi R8 - Red";
url.concat("aX=");
url.concat(Ax);
url.concat("&");
url.concat("aY=");
url.concat(Ay);
url.concat("&");
url.concat("aZ=");
url.concat(Az);

url.concat("&");
url.concat("gX=");
url.concat(Gx);
url.concat("&");
url.concat("gY=");
url.concat(Gy);
url.concat("&");
url.concat("gZ=");
url.concat(Gz);
url.concat("&id=1000123");
url.concat("&lat=");
url.concat(latitude);
url.concat("&lon=");
url.concat(longitude);
url.concat("&temp=");
url.concat(T);
url.concat("&Car=Audi_R8_Red");

//Serial.print(url);
http.begin(url);  //Specify request destination
int httpCode = http.GET();                                                                  //Send the request
 
if (httpCode > 0) { //Check the returning code
 
String payload = http.getString();   //Get the request response payload
//Serial.println(payload);                     //Print the response payload
 
}
/* url="http://192.168.43.107:3000/location?"; 
 http.begin(url);  //Specify request destination
  httpCode = http.GET();                                                                  //Send the request
 
if (httpCode > 0) { //Check the returning code
 
//payload = http.getString();   //Get the request response payload
//Serial.println(payload);                     //Print the response payload
 
}
*/
http.end();  

  delay(100);
}
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
