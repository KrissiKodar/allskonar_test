#include <Arduino.h>
#include <Wire.h>
#include <LSMDS1_Registers.h>
#include <math.h>


#define slave_address 0x6B
#define I2C_SUCCESS 0
#define I2C_ERROR 1


#define accel_scale 2 //+- accel_scale (00: 2, 01: 16, 10: 4, 11: 8)
#define g_scale (32767.0f/accel_scale) // if needed

#define gyro_scale 245 // +- dps_scale (00: 245 dps; 01: 500 dps; 10: Not Available; 11: 2000 dps)
#define dps_scale (32767.0f/gyro_scale)


#define SAMPLERATE_MS 100


#define RAD_TO_DEG 57.3f


//#define SEND_RAW 0

void I2C_WriteRegByte(byte deviceAddress, byte registerAddress, byte value) {
  Wire.beginTransmission(deviceAddress);  // Start communication with the device
  Wire.write(registerAddress);  // Select the register address
  Wire.write(value);  // Write the value to the register
  Wire.endTransmission();  // End communication with the device
}

uint8_t read_outputs_LSM9DS1(uint8_t dev_addr, uint8_t reg_addr, int* Xr, int* Yr, int* Zr) {
  Wire.beginTransmission(dev_addr);  // Start communication with the device
  Wire.write(reg_addr);  // Select the register address

  if (Wire.endTransmission(false) != 0) {  // Send repeated start condition
    return I2C_ERROR;
  }

  if (Wire.requestFrom(dev_addr, 6) != 6) {  // Request 6 bytes from the device
    return I2C_ERROR;
  }

  uint8_t c1 = Wire.read();
  uint8_t c2 = Wire.read();
  *Xr = ((c1 << 8) | c2);

  c1 = Wire.read();
  c2 = Wire.read();
  *Yr = ((c1 << 8) | c2);

  c1 = Wire.read();
  c2 = Wire.read();
  *Zr = ((c1 << 8) | c2);

  return I2C_SUCCESS;
}


int Xg_corr, Yg_corr, Zg_corr;
int Xa_corr, Ya_corr, Za_corr;
int Xg_av = 0, Yg_av = 0, Zg_av = 0;
int Xa_av = 0, Ya_av = 0, Za_av = 0;

unsigned char check     = 1;
unsigned char n_samples = 20;

float theta_a, phi_a;
float theta_g = 0, phi_g = 0;

float theta = 0; //pitch
float phi   = 0; //roll

float alpha = 0.95f;
float beta  = 1 - alpha;


float dt;
unsigned long millisOld;


void setup() {
  Wire.begin();  // Initialize the I2C bus
  Serial.begin(115200);  // Initialize serial communication
  // Gyro settings
  // Acceleration settings
  uint8_t data_rate_scale = 0b01100000;
  if (gyro_scale == 245){
    data_rate_scale |= 0b01100000;
  }
  else if (gyro_scale == 500){
    data_rate_scale |= 0b01101000;
  }
  else if (gyro_scale == 2000){
    data_rate_scale |= 0b01111000;
  }
  /* I2C_WriteRegByte(slave_address, CTRL_REG1_G, data_rate_scale); // data rate and scale
  I2C_WriteRegByte(slave_address, CTRL_REG2_G, 0b00000000); // interrupts, out selection
  I2C_WriteRegByte(slave_address, CTRL_REG3_G, 0b00000000); // power mode and high pass filters
  I2C_WriteRegByte(slave_address, CTRL_REG4, 0b00111000); // angular rate sensor sign and orientation
 */
  //default
  I2C_WriteRegByte(slave_address, CTRL_REG1_G, 0b00000000); // data rate and scale
  I2C_WriteRegByte(slave_address, CTRL_REG2_G, 0b00000000); // interrupts, out selection
  I2C_WriteRegByte(slave_address, CTRL_REG3_G, 0b00000000); // power mode and high pass filters
  I2C_WriteRegByte(slave_address, CTRL_REG4, 0b00111000); // angular rate sensor sign and orientation

  // Acceleration settings
  data_rate_scale = 0b01100000;
  if (accel_scale == 2){
    data_rate_scale |= 0b01100000;
  }
  else if (accel_scale == 16){
    data_rate_scale |= 0b01101000;
  }
  else if (accel_scale == 4){
    data_rate_scale |= 0b01110000;
  }
  else if (accel_scale == 8){
    data_rate_scale |= 0b01111000;
  }
  /* I2C_WriteRegByte(slave_address, CTRL_REG5_XL, 0b00111000); // decimation of output, and accelerometer output
  I2C_WriteRegByte(slave_address, CTRL_REG6_XL, data_rate_scale); // output data rate (119 Hz), scale (+-2g)
  I2C_WriteRegByte(slave_address, CTRL_REG8, 0b00000110); // auto increment for multiple reads, and set to */
  //default
  I2C_WriteRegByte(slave_address, CTRL_REG5_XL, 0b00111000); // decimation of output, and accelerometer output
  I2C_WriteRegByte(slave_address, CTRL_REG6_XL, 0b00000000); // output data rate (119 Hz), scale (+-2g)
  I2C_WriteRegByte(slave_address, CTRL_REG8, 0b00000100); // auto increment for multiple reads, and set to


  // Magnetometer settings
  delay(250);
  millisOld = millis();
}

void send_data_raw(uint8_t status_g, int Xg, int Yg, int Zg, uint8_t status_a, int Xa, int Ya, int Za)
{
  if (status_g == I2C_SUCCESS) {
    Serial.print("g ");
    Serial.print(Xg/dps_scale);
    Serial.print(" ");
    Serial.print(Yg/dps_scale);
    Serial.print(" ");
    Serial.println(Zg/dps_scale);
  } else {
    Serial.println("I2C communication error!");
  }

  if (status_a == I2C_SUCCESS) {
    Serial.print("a ");
    Serial.print((Xa/g_scale)*9.8);
    Serial.print(" ");
    Serial.print((Ya/g_scale)*9.8);
    Serial.print(" ");
    Serial.println((Za/g_scale)*9.8);
  } else {
    Serial.println("I2C communication error!");
  }
}


void send_data_angles(float theta_a, float phi_a, float theta_g, float phi_g, float theta, float phi)
{
  /* Serial.print("a ");
  Serial.print(theta_a);
  Serial.print(" ");
  Serial.println(phi_a);
  Serial.print("g ");
  Serial.print(theta_g);
  Serial.print(" ");
  Serial.println(phi_g);
  Serial.print("f ");
  Serial.print(theta);
  Serial.print(" ");
  Serial.println(phi); */
  Serial.print(theta_a);
  Serial.print(" ");
  Serial.print(phi_a);
  Serial.print(" ");
  Serial.print(theta_g);
  Serial.print(" ");
  Serial.print(phi_g);
  Serial.print(" ");
  Serial.print(theta);
  Serial.print(" ");
  Serial.println(phi);
}

void loop() {
  int Xg, Yg, Zg;
  int Xa, Ya, Za;
  uint8_t status_g = read_outputs_LSM9DS1(0x6B, 0x18, &Xg, &Yg, &Zg);
  uint8_t status_a = read_outputs_LSM9DS1(0x6B, 0x28, &Xa, &Ya, &Za);

  if (check < n_samples)
  {
    Xg_av  += Xg;
    Yg_av  += Yg;
    Zg_av  += Zg;

    Xa_av  += Xa;
    Ya_av  += Ya;
    Za_av  += Za-g_scale;//16384 if full scale is +-2g

    Xg_corr = Xg_av/n_samples;
    Yg_corr = Yg_av/n_samples;
    Zg_corr = Zg_av/n_samples;

    Xa_corr = Xa_av/n_samples;
    Ya_corr = Ya_av/n_samples;
    Za_corr = Za_av/n_samples;

    check += 1;
    delay(100);
    /* if (check == n_samples)
    {
      Serial.print("Xa_av: ");
      Serial.print(Xa_av);
      Serial.print(", Ya_av: ");
      Serial.print(Ya_av);
      Serial.print(", Za_av: ");
      Serial.println(Za_av);
      Serial.println("\n");
      Serial.print("Xg_av: ");
      Serial.print(Xg_av);
      Serial.print(", Yg_av: ");
      Serial.print(Yg_av);
      Serial.print(", Zg_av: ");
      Serial.println(Zg_av);
      Serial.println("\n");
    } */
  }
  else{
    // gyro corrections
    Xg -= Xg_corr;
    Yg -= Yg_corr;
    Zg -= Zg_corr;

    // accelerometer corrections
    Xa -= Xa_corr;
    Ya -= Ya_corr;
    Za -= Za_corr;
    #ifdef SEND_RAW
    send_data_raw(status_g, Xg, Yg, Zg, status_a, Xa, Ya, Za);
    #endif

    #ifndef SEND_RAW
    // acceletometer pitch/roll
    theta_a = atan2(Xa, Za)*RAD_TO_DEG;
    phi_a   = atan2(Ya, Za)*RAD_TO_DEG;

    // gyro pitch/roll
    dt = (millis() - millisOld)/1000.0f;
    millisOld = millis();
    theta_g = theta_g + ((float)Yg/dps_scale)*dt;
    phi_g   = phi_g   - ((float)Xg/dps_scale)*dt;

    theta = (theta + ((float)Yg/dps_scale) * dt) * alpha + theta_a * beta;
    phi =   (phi   - ((float)Xg/dps_scale) * dt) * alpha + phi_a   * beta;
    send_data_angles(theta_a, phi_a, theta_g, phi_g, theta, phi);
    #endif
  }
  delay(SAMPLERATE_MS);  // Wait for a second before repeating the process
}