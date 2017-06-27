#include <Wire.h>
#include <SimpleBLE.h>

SimpleBLE ble;

#define SHT21_SLAVE_ADDR 0x40 //7bit address
#define SHT21_T_NOHOLD_CMD 0xF3
#define SHT21_RH_NOHOLD_CMD 0xF5

#define BH1715FVC_SLAVE_ADDR 0x23

#define BH1715FVC_H_resolution_mode 0x20 // 1回測定
#define BH1715FVC_Power_on 0x01

int i2c_read_sht21(byte command);
int i2c_read_BH1715(byte Power_on,byte MODE); 

float temperature_conversion();
float relative_humidity_conversion();
float illuminance_conversion();
void ble_send(String name,float value);

void setup() {
  float tmp=0.0;
  float rh=0.0;
  float illuminance=0.0;
  Wire.begin(21,22);//sda,scl
  Serial.begin(115200);
  tmp=temperature_conversion();
  Serial.print("温度:"); 
  Serial.print((int)(tmp));
  Serial.print("\n");
  ble_send("温度",tmp);
  
  rh=relative_humidity_conversion();
  Serial.print("相対湿度:"); 
  Serial.print((int)(rh));
  Serial.print("\n");
  ble_send("湿度",rh);
  
  illuminance=illuminance_conversion();
  Serial.print("照度:"); 
  Serial.print((int)(illuminance));
  Serial.print("\n");
  ble_send("照度",illuminance);

  
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);

  
}

void loop() {
  
 Serial.println("good night!");
  delay(500);

  esp_deep_sleep_enable_timer_wakeup(10 * 1000 * 1000);  // wakeup(restart) after 10secs
  esp_deep_sleep_start();
  delay(500);
  Serial.println("now sleeping");  // ここは実行されない
  
}

void ble_send(String name,float value){

  String out = name+ ":" + String(value,DEC);
  ble.begin(out);
  delay(50);
  ble.end();
  delay(100);
  
  }
/*
i2c_read_sht21
sht21の温度か相対湿度をi2c通信で読み出しを行い、
そのデータを返り値として返す。
引数　command: sht21のコマンド送信のコード　詳細はデータシートで
返り値　読みだしたデータからステータスビットを'0'にセットしたもの(int型)
*/
int i2c_read_sht21(byte command){
  int samp_TmporRH =0;
  byte cheak_sum=0;
  Wire.beginTransmission(SHT21_SLAVE_ADDR);
  Wire.write(command);
  Wire.endTransmission();
  delay(100);

  Wire.requestFrom(SHT21_SLAVE_ADDR, 3);
  while(Wire.available()<3){
    delay(1);
  }
  samp_TmporRH = ((Wire.read()) << 8);
  samp_TmporRH += Wire.read();
  cheak_sum = Wire.read();

  samp_TmporRH  &= ~0x0003;
  return samp_TmporRH;
}

int i2c_read_BH1715(byte Power_on,byte MODE){
  int samp_illuminance;
  Wire.beginTransmission(BH1715FVC_SLAVE_ADDR);
  Wire.write(Power_on);
  Wire.endTransmission();
  delay(1);

  Wire.beginTransmission(BH1715FVC_SLAVE_ADDR);
  Wire.write(MODE);
  Wire.endTransmission();
  delay(180);
  
  Wire.requestFrom(BH1715FVC_SLAVE_ADDR, 2);
  while(Wire.available()<2){
    delay(1);
  }
  samp_illuminance=((Wire.read()) << 8);
  samp_illuminance += Wire.read();
  return samp_illuminance;
}
/*temperature_conversion
関数i2c_read_sht21で読みだしたデータを
sht21のデータシートの計算式に従い変換した温度のデータを
返り値として返す関数
引数　void
返り値　温度[℃](float型)
*/
float temperature_conversion(){
  int samp_tmp;
  float tmp;
  samp_tmp=i2c_read_sht21(SHT21_T_NOHOLD_CMD);
  tmp = -46.85+175.72*((float)(samp_tmp)/65536.0);
  return tmp;
}

/*relative_humidity_conversion
関数i2c_read_sht21で読みだしたデータを
sht21のデータシートの計算式に従い変換した相対湿度のデータを
返り値として返す関数
引数　void
返り値　相対湿度[%RH](float型)
*/
float relative_humidity_conversion(){
  int samp_rh;
  float rh;
  samp_rh=i2c_read_sht21(SHT21_RH_NOHOLD_CMD);
  rh = -6+125*((float)(samp_rh)/65536.0);
  return rh;
}

float illuminance_conversion(){
  int samp_illuminance;
  float illuminance;
  samp_illuminance= i2c_read_BH1715(BH1715FVC_Power_on,BH1715FVC_H_resolution_mode);
  illuminance = (float)(samp_illuminance)/1.2;
  return illuminance;
}

