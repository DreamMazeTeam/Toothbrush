#include <Wire.h>
#include <Arduino.h>

#include "LiquidCrystal_I2C.h"

LiquidCrystal_I2C Lcd(0x27, 16, 2);


int gyro_x, gyro_y, gyro_z;
long gyro_x_offset, gyro_y_offset, gyro_z_offset;
bool set_gyro_angles;

long acc_x, acc_y, acc_z, acc_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

long loop_timer;
int temp;


void readMPU6050Data();
void setupMPU6050Registers();


void setup() {
    Wire.begin();
    Lcd.begin();
    Serial.begin(9600);

    setupMPU6050Registers();

    // Создаем оффсеты
    for (int i = 0; i < 1000; i++){
        readMPU6050Data();

        gyro_x_offset += gyro_x;
        gyro_y_offset += gyro_y;
        gyro_z_offset += gyro_z;

        delay(3); // В нете написанно что так этот луп будет 250 Hz
    }

    gyro_x_offset /= 1000;
    gyro_y_offset /= 1000;
    gyro_z_offset /= 1000;

    loop_timer = micros();
}

float mult = 0.0000611;
float multPi = 0.000001066;

void loop() {
    readMPU6050Data();

    gyro_x -= gyro_x_offset;
    gyro_y -= gyro_y_offset;
    gyro_z -= gyro_z_offset;

    // В нете нашел -> 0.0000611 = 1 / (250 Hz * 65.5)

    angle_pitch += gyro_x * mult;
    angle_roll += gyro_y * mult;
    
    // У ардуины функции sin и cos в радианах, поэтому:
    // 0.000001066 = 0.0000611 * (3.142(Число пи) * 180(градусов))

    angle_pitch += angle_roll * sin(gyro_z * multPi);
    angle_roll -= angle_pitch * sin(gyro_z * multPi);

    // Теперь рассчеты акслерометра
    acc_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));  

    // Функция asin тоже в радианах
    // 57.296 = 1 (3.142 / 180)

    angle_pitch_acc = asin((float)acc_y/acc_vector) * 57.296;
    angle_roll_acc = asin((float)acc_x/acc_vector) * -57.296;

    // Без понятия зачем, но так в интернете написано:
    angle_pitch_acc -= 0.0;
    angle_roll_acc -= 0.0;

    if (set_gyro_angles)
    {
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
    }
    else
    {
        angle_pitch = angle_pitch_acc;
        angle_roll = angle_roll_acc;
        set_gyro_angles = true;
    }

    angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
    angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
    
    // Для 250 Hz
    while (micros() - loop_timer < 4000);
    loop_timer = micros();
}   

void setupMPU6050Registers() {
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();
}

void readMPU6050Data() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68 ,14);
    while(Wire.available() < 14);

    // Читаем данные с гироскопа,
    // Но они идут в 8 битном виде, и мы делаем 16 битные числа
    acc_x = Wire.read() << 8 | Wire.read();
    acc_y = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
}