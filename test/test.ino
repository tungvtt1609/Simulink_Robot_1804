// Libraries
#include <LiquidCrystal.h>
#include <Wire.h>
#include "TimerOne.h"

const int RS = 8;
const int EN = 7;
const int D4 = 6;
const int D5 = 5;
const int D6 = 4;
const int D7 = 3;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// Icons
uint8_t Battery[8] = {0x0E, 0x1B, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F};
uint8_t Panel[8] = {0x1F, 0x15, 0x1F, 0x15, 0x1F, 0x15, 0x1F, 0x00};
uint8_t Pwm[8] = {0x1D, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x17};
uint8_t Flash[8] = {0x01, 0x02, 0x04, 0x1F, 0x1F, 0x02, 0x04, 0x08};

// Constants
#define bulk_voltage_max 14.5 // dien ap max buck
#define bulk_voltage_min 13   // dien ap min buck

#define battery_min_voltage 10 // dien ap min bat
#define solar_min_voltage 17.8 // dien ap min sol
#define charging_current 2.0   // sac dong

#define absorption_max_current 2.0 // dong max tuyet doi
#define absorption_min_current 0.1 // dong min tuyet doi
#define absorption_voltage 14.7    // dien ap tuyet doi

#define float_voltage_max 13.6   // dien ap max noi
#define float_voltage_min 13.2 // dien ap min noi
#define float_voltage 13.4     // dien ap noi
#define float_max_current 0.12 // dong max noi
#define LCD_refresh_rate 1000

// Chế độ làm việc
byte BULK = 0; // Give values to each mode
byte ABSORPTION = 1;
byte FLOAT = 2;
byte mode = 0; // We start with mode 0 BULK

// Inputs
#define solar_voltage_in A1
#define solar_current_in A0
#define battery_voltage_in A2

// Outputs
#define PWM_out 10
#define load_enable 2

// Variables
float bat_voltage = 0;
int pwm_value = 0;
float solar_current = 0;
float current_factor = 0.185; // Value defined by manufacturer ACS712 5A
float solar_voltage = 0;
float solar_power = 0;
float solar_power_last = 0;
String load_status = "OFF";
int pwm_percentage = 0;
unsigned long before_millis = 0;
unsigned long now_millis = 0;
String mode_str = "SAC DONG";

void setup()
{
    pinMode(solar_voltage_in, INPUT); // Set pins as inputs
    pinMode(solar_current_in, INPUT);
    pinMode(battery_voltage_in, INPUT);

    pinMode(PWM_out, OUTPUT);   // Set pins as OUTPUTS
    digitalWrite(PWM_out, LOW); // Set PWM to LOW so MSOFET is off

    pinMode(load_enable, OUTPUT);
    digitalWrite(load_enable, LOW); // Start with the relay turned off

    Timer1.initialize(1000);

    TCCR1B = TCCR1B & B11111000 | B00000001; // timer 1 PWM frequency of 31372.55 Hz
    Serial.begin(9600);
    lcd.clear();
    lcd.begin(20, 4);
    lcd.createChar(0, Battery);
    lcd.createChar(1, Panel);
    lcd.createChar(2, Pwm);
    lcd.createChar(3, Flash);
    before_millis = millis(); // Used for LCD refresh rate
}

void loop()
{
    solar_voltage = get_solar_voltage(15);       // doc het 1.5ms đây là hàm lấy giá trị của điện áp solar
    bat_voltage = get_battery_voltage(15);       // đây là hàm lấy giá trị của điện áp acquy
    solar_current = get_solar_current(15);       //  đây là hàm lấy giá trị của dòng của solar
    solar_power = solar_voltage * solar_current; //  đây là hàm lấy giá trị của năng lượng của solar
    solar_power_last = solar_power;
    pwm_percentage = map(pwm_value, 0, 255, 0, 100); // chuyen gia tri pwm tu 0-255 thanh 0-100

    now_millis = millis();                             // lấy thời gian hiện tại
    if (now_millis - before_millis > LCD_refresh_rate) // nếu thời gian hiện tại - thời gian trước > thời gian cập nhật LCD
    {
        before_millis = now_millis; // cập nhật thời gian trước = thời gian hiện tại
        lcd.clear();
        lcd.setCursor(0, 0);         // chuyen den dong thu 0 cot thu 0
        lcd.write((byte)1);          // in ra ky tu 1
        lcd.print(" ");              // in ra khoang trang
        lcd.print(solar_voltage, 2); // in ra gia tri cua dien ap solar
        lcd.print("V");              // in ra ky tu V
        lcd.print("    ");           // in ra khoang trang
        lcd.write((byte)0);          // in ra acquy icon
        lcd.print(" ");              // in ra khoang trang
        lcd.print(bat_voltage, 2);   // in ra gia tri cua dien ap acquy
        lcd.print("V");              // in ra ky tu V

        lcd.setCursor(0, 1);         // chuyen den dong thu 1 cot thu 0
        lcd.print("  ");             //  in ra khoang trang
        lcd.print(solar_current, 2); // in ra gia tri cua dong solar
        lcd.print("A");              // in ra ky tu A
        lcd.print("     TAI ");      // in ra khoang trang và tên tải
        lcd.print(load_status);      // in ra trạng thái của tải

        lcd.setCursor(0, 2);       // chuyen den dong thu 2 cot thu 0
        lcd.print("  ");           // in ra khoang trang
        lcd.print(solar_power, 2); // in ra gia tri cua nang luong solar
        lcd.print("W");            // in ra ky tu W
        lcd.print("     PWM ");    // in ra khoang trang và tên PWM
        lcd.print(pwm_percentage); //  in ra gia tri cua PWM
        lcd.print("%");            // in ra ky tu %

        lcd.setCursor(0, 3); // chuyen den dong thu 3 cot thu 0
        lcd.print(mode_str); // in ra trạng thái của chế độ
    }

    ///////////////////////////FLOAT-che do tha noi///////////////////////////
    switch (mode)
    {
    case 2:
        /* code */
        if (bat_voltage < float_voltage_min) // nếu điện áp acquy < điện áp thả nổi tối thiểu
        {
            mode = BULK;           // chuyển chế độ thành ổn dòng
            mode_str = "SAC DONG"; // chuyển trạng thái chế độ thành SAC DONG
        }
        else
        {
            if (solar_current > float_max_current) // nếu dòng của solar > dòng thả nổi tối đa
            {                                      // If we exceed max current value, we change mode
                mode = BULK;                       // chuyển chế độ thành ổn dòng
                mode_str = "SAC DONG";             // chuyển trạng thái chế độ thành SAC DONG
            }                                      // End if >

            else // nếu dòng của solar < dòng thả nổi tối đa
            {

                if (bat_voltage > float_voltage) // nếu điện áp acquy > điện áp thả nổi
                {
                    pwm_value--;                              // giảm gia tri pwm
                    pwm_value = constrain(pwm_value, 0, 254); // giới hạn gia tri pwm từ 0-254
                }
                else // nếu điện áp acquy < điện áp thả nổi
                {
                    pwm_value++;                              // tăng gia tri pwm
                    pwm_value = constrain(pwm_value, 0, 254); // giới hạn gia tri pwm từ 0-254
                }
            }
            // End else > float_max_current
            analogWrite(PWM_out, pwm_value); // gửi giá trị pwm đến pin PWM_out
        }
        break;
    
    case 0:
        /* code */
        if (solar_current > charging_current) // nếu dòng của solar > dòng sạc tối đa
        {
            pwm_value--;                              // giảm gia tri pwm
            pwm_value = constrain(pwm_value, 0, 254); // giới hạn gia tri pwm từ 0-254
        }
        else // nếu dòng của solar < dòng sạc tối đa
        {
            // MPPT P&O
            if (solar_power > solar_power_last)
            {
                pwm_value++;                              // tăng gia tri pwm
                pwm_value = constrain(pwm_value, 0, 254); // giới hạn gia tri pwm từ 0-254
            }
            else
            {
                pwm_value--;                              // giảm gia tri pwm
                pwm_value = constrain(pwm_value, 0, 254); // giới hạn gia tri pwm từ 0-254
            }
        }
        analogWrite(PWM_out, pwm_value); // gửi giá trị pwm đến pin PWM_out
        break;

    case 1: // tức là acquy đang ở chế độ ổn áp khi mà điện áp nó lớn hơn 14.5V
        /* code */
        if (solar_current > absorption_max_current)   // nếu dòng của solar > dòng ổn áp tối đa
        {                                             // If we exceed max current value, we reduce duty cycle
            pwm_value--;                              // giảm gia tri pwm
            pwm_value = constrain(pwm_value, 0, 254); // giới hạn gia tri pwm từ 0-254
        }                                             // End if > absorption_max_current

        else // nếu dòng của solar < dòng ổn áp tối đa
        {
            if (bat_voltage > absorption_voltage) // nếu điện áp acquy > điện áp ổn áp
            {
                pwm_value++;                              // tăng gia tri pwm
                pwm_value = constrain(pwm_value, 0, 254); // giới hạn gia tri pwm từ 0-254
            }

            else // nếu điện áp acquy < điện áp ổn áp
            {
                /////////////// MPPT P&O ////////////////////
                if (solar_power > solar_power_last)
                {
                    pwm_value--;                              // giảm gia tri pwm
                    pwm_value = constrain(pwm_value, 0, 254); // giới hạn gia tri pwm từ 0-254
                }
                else
                {
                    pwm_value++;                              // giảm gia tri pwm
                    pwm_value = constrain(pwm_value, 0, 254); // giới hạn gia tri pwm từ 0-254
                }
            }

            if (solar_current < absorption_min_current) // nếu dòng của solar < dòng ổn áp tối thiểu
            {
                mode = FLOAT;         // chuyển chế độ thành thả nổi
                mode_str = "THA NOI"; // chuyển trạng thái chế độ thành THA NOI
            }
        }                                // End else > absorption_max_current
        analogWrite(PWM_out, pwm_value); // gửi giá trị pwm đến pin PWM_out
        break;

    default:
        break;
    }
    
   

    ////////////////////////////////////////////////////////////////////////

    if (bat_voltage < bulk_voltage_min) // nếu điện áp acquy < điện áp ổn dòng tối thiểu
    {
        mode = BULK;           // chuyển chế độ thành ổn dòng
        mode_str = "SAC DONG"; // chuyển trạng thái chế độ thành SAC DONG
    }
    if (bat_voltage >= bulk_voltage_max) // nếu điện áp acquy > điện áp ổn dòng tối đa
    {
        mode_str = "SAC AP"; // chuyển trạng thái chế độ thành SAC AP
        mode = ABSORPTION;   // chuyển chế độ thành ổn áp
    }

    ////////////////////////////////////////////////////////////////////////
    

    //////////////////TẢI//////////////////////////////

    if (bat_voltage < battery_min_voltage) // nếu điện áp acquy < điện áp acquy tối thiểu
    {
        digitalWrite(load_enable, LOW); // tắt tải
        load_status = "OFF";            // chuyển trạng thái tải thành OFF
    }
    else
    {
        digitalWrite(load_enable, HIGH); // bật tải
        load_status = "ON";              // chuyển trạng thái tải thành ON
    }
}

/////////////////////////FUNCTIONS/////////////////////////

float get_solar_voltage(int n_samples) // hàm lấy điện áp của solar
{
    float voltage = 0;                  // khởi tạo giá trị điện áp của solar
    for (int i = 0; i < n_samples; i++) // lặp n_samples lần
    {
        voltage += (analogRead(solar_voltage_in) * (5.0 / 1023.0) * 8.3); // tính điện áp của solar
    }
    voltage = voltage / n_samples; // tính điện áp của solar
    if (voltage < 0)               // nếu điện áp của solar < 0
    {
        voltage = 0; // điện áp của solar = 0
    }
    return (voltage); // trả về điện áp của solar
}

float get_battery_voltage(int n_samples) // hàm lấy điện áp của battery
{
    float voltage = 0;                  // khởi tạo giá trị điện áp của battery
    for (int i = 0; i < n_samples; i++) // lặp n_samples lần
    {
        voltage += (analogRead(battery_voltage_in) * (5.0 / 1023.0) * 8.3); // tính điện áp của battery
    }
    voltage = voltage / n_samples; // tính điện áp của battery
    if (voltage < 0)               // nếu điện áp của battery < 0
    {
        voltage = 0; // điện áp của battery = 0
    }
    return (voltage); // trả về điện áp của battery
}

float get_solar_current(int n_samples) // hàm lấy dòng của solar
{
    float Sensor_voltage;               // khởi tạo giá trị điện áp của sensor
    float current = 0;                  // khởi tạo giá trị dòng của solar
    for (int i = 0; i < n_samples; i++) // lặp n_samples lần
    {
        Sensor_voltage = analogRead(solar_current_in) * (5.0 / 1023.0); // tính điện áp của sensor
        current = current + (Sensor_voltage - 2.5) / current_factor;    // tính dòng của solar
    }
    current = current / n_samples; // tính dòng của solar
    if (current < 0)               // nếu dòng của solar < 0
    {
        current = 0; // dòng của solar = 0
    }
    return (current); // trả về dòng của solar
}
