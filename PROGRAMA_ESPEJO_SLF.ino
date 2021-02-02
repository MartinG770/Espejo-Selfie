#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory
#include "BluetoothSerial.h"   //Libreria de comunicacion BT
BluetoothSerial ESP_BT;
#include "Base64.h"            //Libreria de Base 64
#include <Wire.h>
#include "SPI.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//VARIABLES QUE SE USAN EN EL PROGRAMA//
int incoming;
char character;
int LED = 13;
int bos = 14;
int j = 0;
int k = 0;
int l = 0;
int m = 0;
int g = 0;
int h = 0;
int a = 0;
int b = 0;
int y = 0;
int d = 0;
int q = 0;
int inicio = 0;
//.............................................//

//ARCHIVOS PARA GUARDAR FOTO, GUARDAR ARCHIVO BASE 64 Y FOTO A ENVIAR//
File b64;
File miarchivo;
File myfile;
//...........................................................//
// DEFINE EL NUMERO DE BYTES A ACCEDER//
#define EEPROM_SIZE 1
//.........................................................//
// DEFINICION DE PINES PARA CAMERA_MODEL_AI_THINKER //
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
//...........................................//
//VARIABLE DE NUMERO DE FOTO//
unsigned int pictureNumber = 0;
//...........................................//

//ESTABLECE LOS DATOS DE CONFIGURACION DE LA CAMARA//
camera_fb_t * fb = NULL;
camera_config_t config;
//...........................................//
void setup() {
  //ABRE LA CONFIGURACION DE LA CAMARA//
  configInitCamera();
  //............................................//

  //INICIA LA COMUNICACION PARA EL MONITOR SERIE//
  Serial.begin(115200);//SE COLOCA EN COMENTARIO PARA EVITAR PROBLEMAS DE BOTONES//
  //.............................................//

  //ESTABLECE EL NOMBRE DE BLUETOOTH//
  ESP_BT.begin("ESPEJO2.0"); //Name of your Bluetooth Signal
  Serial.println("Bluetooth Device is Ready to Pair");
  //...........................................//

  //ESTABLECE LOS PINES DE LOS RELEVADORES//
 /* pinMode (LED, OUTPUT);//Specify that LED pin is output
  pinMode (bos, OUTPUT);
  pinMode(4, OUTPUT);*/
  //...........................................//

 // INICIALIZA LA SD CARD//
  /* if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return;
  }*/
  //.........................//
}
void loop() {
  //INICIA LA TOMA DE DATOS DEL DISPOSITIVO BLUETOOTH//
  if (ESP_BT.available()) //VERIFICA SI HAY ALGO EN EL PUERTO BT
  {
    incoming = ESP_BT.read(); //LEE QUE RECIVE DEL BT
    Serial.print("Received:"); Serial.println(incoming);//IMPRIME POR EL PUERTO SERIE QUE RECIVIO

    //ESTABLECE EL PIN4(FLASH) EN APAGADO//
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);
    //..................................//

    //CUANDO RECIVE UNA Y POR EL BT ABRE EL ARCHIVO BASE 64 Y LO ENVIA POR BT//
    if (incoming == 'Y') {
      if (!SD_MMC.begin()) {
        Serial.println("SD Card Mount Failed");
        return;
      }
      q = 1;
      if (h == 0) {
        SDwriteb64();
        myfile = SD_MMC.open("/ESPblc.txt"); //Abrimos el fichero
        if (myfile) {
          while (myfile.available()) {
            ESP_BT.write(myfile.read());
          }
          ESP_BT.print("*");
          myfile.close(); //Es muy importante cerrar el archivo al final
        }
        else {
          Serial.println("Error al abrir el archivo.");
        }
        h = 1;
        //SD_MMC.end();
      }
    }
    //...............................................//

    //ACTIVACION DE COMPONENTES SEGUN LO QUE RECIVA DEL DISPOSITIVO BT//
    //LEDS//
  /*  if (incoming == 'a') {
      j++;
      if (j == 1) {
        digitalWrite(LED, LOW);
        delay(200);
      }
      if (j == 2) {
        digitalWrite(LED, HIGH);
        delay(200);
        j = 0;
      }
    }
    //BOCINAS//
    if (incoming == 'c') {
      k++;
      if (k == 1) {
        digitalWrite(bos, LOW);
        delay(200);
      }
      if (k == 2) {
        digitalWrite(bos, HIGH);
        delay(200);
        k = 0;
      }
    }
    //TOMA DE FOTO//
    if (incoming == 'd') {
      m++;
      if (m == 1) {
        digitalWrite(LED, LOW);
        delay(750);
        digitalWrite(LED, HIGH);
        delay(750);
        digitalWrite(LED, LOW);
        delay(750);
        digitalWrite(LED, HIGH);
        delay(750);
        digitalWrite(LED, LOW);
        delay(2000);
        takephoto();
        m = 0;
      }
    }
    delay(100);
    //..............................//
  */}
  //................................//
/*
  //ESTABLECE LOS BOTONES Y TOMA LA LECTURA
  int btna = 3;
  int btnb = 16;
  int btnd = 1;

  pinMode(btna, INPUT);//leds
  pinMode(btnb, INPUT);//bocinas
  pinMode(btnd, INPUT);//Camara
  //toma lectura de botones
  a = digitalRead(btna);
  b = digitalRead(btnb);
  d = digitalRead(btnd);
  //..................................//

  //ENCIENDE O APAGA LOS COMPONENTES SEGUN LA SEÑAL DEL BOTON//
  if (inicio == 0) {
    digitalWrite(LED, HIGH);
    digitalWrite(bos, HIGH);
    inicio = 1;
  }
  //LEDS//
  if (a == 1) {
    j++;
    if ( j == 1) {
      digitalWrite(LED, LOW);
      delay(200);
    }
    if ( j == 2) {
      digitalWrite(LED, HIGH);
      delay(200);
      j = 0;
    }
  }
  //BOCINAS
  if (b == 1) {
    k++;
    if (k == 1 ) {
      digitalWrite(bos, LOW);
      delay(200);
    }
    if (k == 2) {
      digitalWrite(bos, HIGH);
      delay(200);
      k = 0;
    }
  }
  //TOMA DE FOTO//
  if (d == 1) {
    m++;
    //INICIA UNA SEUENCIA DE PARPADEO DE 3S//
    if (m == 1) {
      digitalWrite(LED, LOW);
      delay(750);
      digitalWrite(LED, HIGH);
      delay(750);
      digitalWrite(LED, LOW);
      delay(750);
      digitalWrite(LED, HIGH);
      delay(750);
      digitalWrite(LED, LOW);
      delay(2000);
      takephoto();//VA A LA SUBRUTINA DE TOMA DE FOTO
      m = 0;
    }
  }
  //.............................//
  delay(200);
  //ESTABLECE LAS ENTRADAS COMO SALIDAS PARA NO TENER PROBLEMAS DE COMUNICACION//
  if (q == 1) {
    int btna = 12;
    int btnb = 16;
    int btnd = 2;

    pinMode(btna, OUTPUT);//leds
    pinMode(btnb, OUTPUT);//bocinas
    pinMode(btnd, OUTPUT);//Camara
    q = 0;
  }
  //.............................................//
*/}

void configInitCamera() {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; //YUV422,GRAYSCALE,RGB565,JPEG

  // Select lower framesize if the camera doesn't support PSRAM
  config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  config.jpeg_quality = 10; //10-63 lower number means higher quality
  config.fb_count = 2;

  // Initialize the Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 2);       // -2 to 2
  s->set_saturation(s, 2);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 300);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
}

void takephoto() {
  //SD_MMC.begin();
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return;
  }
  // Take Picture with Camera
  fb = esp_camera_fb_get();
  digitalWrite(LED, LOW);
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  //initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;

  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) + ".jpg";
  fs::FS &fs = SD_MMC;
  Serial.printf("Picture file name: %s\n", path.c_str());

  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
  }
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();
  esp_camera_fb_return(fb);

  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);    //make sure flash is held LOW in sleep
  delay(2000);
  //Serial.println("Going to sleep now");
  //  ESP.restart();
  SD_MMC.end();
}

void SDwriteb64() {
  String data;
  int t = 0;
  int em;
  int lim;
  String buffere;
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0);
  String pic = ("/picture" + String(pictureNumber) + ".jpg");
  //Serial.print(pic);
  miarchivo = SD_MMC.open(pic, FILE_READ);
  int sz = miarchivo.size();
  int q = sz / 3;
  b64 = SD_MMC.open("/ESPblc.txt", FILE_WRITE);
  if (sz % 3 == 2) {
    em = 2;
  }
  if (sz % 3 == 1) {
    em = 1;
  }
  if (b64)
  {
    if (miarchivo) {
      while (miarchivo.available()) {
        if (q >= t) {
          lim = 3;
        }
        else {
          lim = em;
        }
        byte input[lim];
        for (int i = 0; i < lim; i++)
        {
          input[i] = miarchivo.read();
          buffere += input[i];
        }
        int inputLen = sizeof(buffere);
        int encodedLen = base64_enc_len(inputLen);
        char encoded[encodedLen];
        base64_encode(encoded, (char *)input, inputLen);
        char sub[5];//5
        strncpy(sub, encoded, lim + 1); //4
        if (lim == 1) {
          sub[2] = '=';
          sub[3] = '=';
        }
        if (lim == 2) {
          sub[3] = '=';
        }
        sub[4] = '\0';
        // Serial.println(sub);
        data = sub;
        // Serial.print(data);
        b64.print(data);
        t++;
      }
      miarchivo.close();
    }
    else {
      Serial.println("Error Opening for reading myFile");
    }

    b64.close();
  }
  else {
    Serial.println("Error Opening for writing b64");
  }

}
