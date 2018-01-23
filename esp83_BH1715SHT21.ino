#include <Wire.h>
#include <SimpleBLE.h>

SimpleBLE ble;

#define SHT21_SLAVE_ADDR 0x40 //7bit address
#define SHT21_T_NOHOLD_CMD 0xF3
#define SHT21_RH_NOHOLD_CMD 0xF5


#define BH1715FVC_SLAVE_ADDR 0x23

#define BH1715FVC_H_resolution_mode 0x20 // 1回測定
#define BH1715FVC_Power_on 0x01

#define max_data_num 20
#define tmp_err_range 5
#define illuminance_err_range 250
#define rh_err_range 5

int i2c_read_sht21(byte command);
int i2c_read_BH1715(byte Power_on,byte MODE); 

float temperature_conversion();
float relative_humidity_conversion();
float illuminance_conversion();

float get_SensorData(char sensor);
float avr_SensorData_clalulation(char sensor, int err_range,int data_num,float*sen_data);

void ble_send(String id,int sensor_data);

void setup() {
  float tmp=0.0;
  float rh=0.0;
  float illuminance=0.0;
  int up_data = 0;
  
  Wire.begin(21,22);//sda,scl
  Serial.begin(115200);
  
  tmp=get_SensorData('T');
  Serial.print("温度:"); 
  Serial.print((int)(tmp));
  Serial.print("\n");
   
  rh=get_SensorData('H');
  Serial.print("相対湿度:"); 
  Serial.print((int)(rh));
  Serial.print("\n");

  illuminance=get_SensorData('I');
  Serial.print("照度:"); 
  Serial.print((int)(illuminance));
  Serial.print("\n");
   
  for(up_data = 0;up_data<10;up_data++){
     ble_send("AAA",(int)(tmp));
     ble_send("BBB",(int)rh);
     ble_send("CCC",(int)illuminance);
     delay(10);
  }
 
  
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);

  
}

void loop() {
  
 Serial.println("good night!");
  delay(500);

  esp_deep_sleep_enable_timer_wakeup(10 *60* 1000 * 1000);  // wakeup(restart) after 5 min
  esp_deep_sleep_start();
  delay(500);
  Serial.println("now sleeping");  // ここは実行されない
  
}

void ble_send(String id,int sensor_data){
  int out_len;
  String out = id+  String(sensor_data,DEC);
  out_len =out.length();
  while(out_len != 12){
    out = out +'Z';
    out_len = out.length();
  }
  ble.begin(out);
  delay(50);
  ble.end();
  delay(100);
  
  }
/*------------------------------------------------------------------------
i2c_read_sht21
sht21の温度か相対湿度をi2c通信で読み出しを行い、
そのデータを返り値として返す。
引数　command: sht21のコマンド送信のコード　詳細はデータシートで
返り値　読みだしたデータからステータスビットを'0'にセットしたもの(int型)
-------------------------------------------------------------------------*/
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
/*----------------------------------------------------------------------------
i2c_read_BH1715
データシートに従いi2c通信を用いて照度センサーのデータを読み出し
そのデータを返り値として返す
引数　Power_on: BH1715の内部回路が測定命令待ち状態になる。Opecode
　　　MODE:測定のモードをきめる　今回は、H-Resolution Modeの1回測定を使用する
返り値　読みだしたデータを返す(int型)
------------------------------------------------------------------------------*/
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
/*-----------------------------------------------------------
 temperature_conversion
関数i2c_read_sht21で読みだしたデータを
sht21のデータシートの計算式に従い変換した温度のデータを
返り値として返す関数
引数　void
返り値　温度[℃](float型)
--------------------------------------------------------------*/
float temperature_conversion(){
  int samp_tmp;
  float tmp;
  samp_tmp=i2c_read_sht21(SHT21_T_NOHOLD_CMD);
  tmp = -46.85+175.72*((float)(samp_tmp)/65536.0);
  return tmp;
}

/*------------------------------------------------------------
 relative_humidity_conversion
関数i2c_read_sht21で読みだしたデータを
sht21のデータシートの計算式に従い変換した相対湿度のデータを
返り値として返す関数
引数　void
返り値　相対湿度[%RH](float型)
--------------------------------------------------------------*/
float relative_humidity_conversion(){
  int samp_rh;
  float rh;
  samp_rh=i2c_read_sht21(SHT21_RH_NOHOLD_CMD);
  rh = -6+125*((float)(samp_rh)/65536.0);
  return rh;
}
/*------------------------------------------------------------
 illuminance_conversion()
 関数i2c_read_BH1715で読みだしたデータを
 BH1715FVCのデータシートの計算式に従い照度のデータを算出する
 関数
 引数void
 返り値　照度[lx](float型)
 ------------------------------------------------------------*/
float illuminance_conversion(){
  int samp_illuminance;
  float illuminance;
  samp_illuminance= i2c_read_BH1715(BH1715FVC_Power_on,BH1715FVC_H_resolution_mode);
  illuminance = (float)(samp_illuminance)/1.2;
  return illuminance;
}
/*--------------------------------------------------------------------
 get_SensorData
 関数avr_SensorData_clalulationと連動している
 avr_SensorData_clalulationから返ってきた値を
 センサーの正確なデータとして返す。
 またavr_SensorData_clalulationにセンサー毎の誤差範囲と
 センサーデータ数：data_numを０として用意し,一つ一つのデータを格納するsensor_data
 を用意しておく
 引数：sensor 取りたいセンサーを指定する今回は、T:温度I:照度 H:湿度
 返り値：センサーのデータ(float型)
 -------------------------------------------------------------------*/
float get_SensorData(char sensor){      //sensor(T:温度I:照度 H:湿度)
  int err_range;
  int data_num=0;
  float sensordata;
  float sensor_data[max_data_num];
  
  switch(sensor){   //センサー毎の誤差範囲を取得
    case 'T':
     err_range = tmp_err_range; //10
    break;
    case 'I':
     err_range = illuminance_err_range; //100
    break;
    case 'H':
     err_range = rh_err_range; //10
    break;
    default:
           printf("NoSensor \n");
     return 0;
  }

  sensordata = avr_SensorData_clalulation(sensor, err_range,data_num,sensor_data);

  return sensordata;
}
/*
 avr_SensorData_clalulation
 関数get_SensorDataと連動している
 指定したセンサーの値をデータ数の最大までとり、
 その合計のアベレージから誤差範囲を超えた値を除外してゆく。
 そしてデータ数が最大になった時のアベレージを正確なデータとして返す関数
 引数　  sensor :データを取りたいセンサーを指定
         err_range:センサー毎の誤差範囲の値
         data_num:sensor_dataに格納されているデータ数
         sensor_data:誤差範囲を超えないデータを保持するために
 返り値　センサーのデータのアベレージ(float型)
 */
float avr_SensorData_clalulation(char sensor, int err_range,int data_num,float*sensor_data){
  float avr_sensor_data=0;
  int num;
  int chk_range,over_range;             
  
  switch(sensor){      //センサー毎にデータを取得する
    case 'T':
     for(;data_num<max_data_num;data_num++){
      sensor_data[data_num] =temperature_conversion();
     }
     break;
    case 'I':
     for(;data_num<max_data_num;data_num++){
      sensor_data[data_num] =illuminance_conversion();
     }
     break;
    case 'H':
     for(;data_num<max_data_num;data_num++){
      sensor_data[data_num] =relative_humidity_conversion();
     }
     break;
  }
  for(num=0;num<max_data_num;num++){
    avr_sensor_data+=sensor_data[num];
  }
    avr_sensor_data/=max_data_num;
    
  for(chk_range=(max_data_num-1);chk_range>=0;chk_range--){ //誤差範囲を超える値を後ろにやり、その分のdata_numを減らす
    if((sensor_data[chk_range]<=(avr_sensor_data - err_range))||(sensor_data[chk_range]>=(avr_sensor_data + err_range))){
    for(over_range= chk_range;over_range<(max_data_num-1);over_range++){
        sensor_data[over_range]=sensor_data[over_range+1];
        sensor_data[over_range+1]=0;
      }
      data_num--;
    }
  }
  if(data_num < max_data_num){      //データ数がマックスに到達していないときマックスに到達するまでこの関数をやり直す
    avr_sensor_data=avr_SensorData_clalulation(sensor, err_range,data_num,sensor_data); //再帰関数
  }
  return avr_sensor_data;           //データ数がマックスに到達したときの値を返す
}
