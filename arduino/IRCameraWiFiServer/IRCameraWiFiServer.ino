/*
 MLX90640 thermal camera connected to a SparkFun Thing Plus - ESP32 WROOM

 Created by: Christopher Black
 */
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Wire.h>  // Used for I2C communication
//#include <SFE_Micro//oled.h>  // Include the SFE_Micro//oled library
#include <WebSocketsServer.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
//#include "env.h"
#include "webpage.h"

// Micro//oled variables
#define PIN_RESET 9  
#define DC_JUMPER 1 
// Micro//oled //oled(PIN_RESET, DC_JUMPER);    // I2C declaration


#define wifi_term_ssid "ThermalCam AP"
#define wifi_term_pw "88888888"


#define wifi_AP_ssid "ThermalCam"
#define wifi_AP_pw "88888888"


bool Term_or_AP=false;
// WiFi variables
WiFiServer server(80);


//declare socket related variables
WebSocketsServer webSocket = WebSocketsServer(81);

// MLX90640 variables
#define TA_SHIFT -64; // Default shift for MLX90640 in open air is 8
static float mlx90640To[768];

// Used to compress data to the client
char positive[27] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
char negative[27] = "abcdefghijklmnopqrstuvwxyz";

TaskHandle_t TaskA;
/* this variable hold queue handle */
xQueueHandle xQueue;

int total = 0;

int LED_CH=2;
int Servo_CH=3;

void setup()
{
    ledcSetup(LED_CH, 5000, 10);
    ledcAttachPin(2, LED_CH);

    
//    ledcSetup(Servo_CH, 25, 12);
//    ledcAttachPin(3, Servo_CH);

    
    Serial.begin(115200);
    delay(1000);

    // Connect to the WiFi network

    
    ledcWrite(LED_CH, 0);
    WiFi.setHostname("esp32thing1");
    if(Term_or_AP)
    {
      Serial.println();
      Serial.println();
      Serial.print("Connecting to ");
      Serial.println(wifi_term_ssid);

      int retry = 0;
      while (WiFi.status() != WL_CONNECTED) {
          delay(1000);
          retry += 1;
          Serial.print(".");
          if (retry > 4 ) {
            // Retry after 5 seconds
            Serial.println("");
            WiFi.begin(wifi_term_ssid, wifi_term_pw);
            retry = 0;
          }
      }
      
    }
    else
    {
      WiFi.softAP(wifi_AP_ssid, wifi_AP_pw); //as AP
      
      IPAddress local_ip(192,168,123,1);
      IPAddress gateway(192,168,123,1);
      IPAddress subnet(255,255,255,0);
      
      WiFi.softAPConfig(local_ip, gateway, subnet);
    }





    ledcWrite(LED_CH, 50);
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    if (!MDNS.begin("thermal")) {
        Serial.println("Error setting up MDNS responder!");
    } else {
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("ws", "tcp", 81);
        Serial.println("mDNS responder started");
    }
    
    server.begin();
    
    xQueue = xQueueCreate(1, sizeof(mlx90640To));
    xTaskCreatePinnedToCore(
      Task1,                  /* pvTaskCode */
      "Workload1",            /* pcName */
      100000,                   /* usStackDepth */
      NULL,                   /* pvParameters */
      1,                      /* uxPriority */
      &TaskA,                 /* pxCreatedTask */
      0);                     /* xCoreID */
    xTaskCreate(
      receiveTask,           /* Task function. */
      "receiveTask",        /* name of task. */
      10000,                    /* Stack size of task */
      NULL,                     /* parameter of the task */
      1,                        /* priority of the task */
      NULL);                    /* Task handle to keep track of created task */

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

int value = 0;
int dataVal = 0;

void loop(){
  webSocket.loop();
  
//  ledcWrite(LED_CH, 100);
  WiFiClient client = server.available();   // listen for incoming clients
  
  if (client) {
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected()) {
      
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == '\n') {

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print("<script>const ipAddress = '"); 
            client.print(WiFi.localIP());
            client.print("'</script>");
            client.println();
            // the content of the HTTP response follows the header:
            client.print(canvas_htm);
            
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}

int speedIdx2FPS(int idx)
{
  return 1<<(idx-1);
}

int frameSpeedIdx=4;
// Capture thermal image on a different thread
void Task1( void * parameter )
{
    int tick = 0;
    const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
    
//    Micro//oled //oled(PIN_RESET, DC_JUMPER);    // I2C declaration
    Wire.setClock(400000L);
    Wire.begin();
    
    //oled.begin();    // Initialize the //oled
    //oled.clear(ALL); // Clear the display's internal memory
    //oled.display();  // Display what's in the buffer (splashscreen)
    delay(50);
    //oled.clear(PAGE); // Clear the buffer
    //oled.print("Welcome!");
    delay(400);
    //oled.display(); // Draw on the screen
    paramsMLX90640 mlx90640;
    Wire.beginTransmission((uint8_t)MLX90640_address);
    if (Wire.endTransmission() != 0) {
        Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
        while (1);
    }
    Serial.println("MLX90640 online!");

    //Get device parameters - We only have to do this once
    int status;
    uint16_t eeMLX90640[832];
    status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);

    if (status != 0) {
        Serial.println("Failed to load system parameters");
    }
    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    if (status != 0) {
        Serial.println("Parameter extraction failed");
    }
    int cur_frameSpeedIdx=frameSpeedIdx;
    MLX90640_SetRefreshRate(MLX90640_address,cur_frameSpeedIdx);
    Wire.setClock(1000000L);
    float mlx90640Background[768];
    for( ;; )
    {
      bool skipFrame=false;
      if(cur_frameSpeedIdx!=frameSpeedIdx)//thread danger but whatever
      {
        cur_frameSpeedIdx=frameSpeedIdx;
        MLX90640_SetRefreshRate(MLX90640_address,cur_frameSpeedIdx);
        
        vTaskDelay(20/ portTICK_PERIOD_MS);
        skipFrame=true;
      }
//      String startMessage = "Capturing thermal image on core ";
//      startMessage.concat(xPortGetCoreID());
//      Serial.println( startMessage );
//      long startTime = millis();
      for (byte x = 0 ; x < 2 ; x++) //Read both subpages
      {
        uint16_t mlx90640Frame[834];
        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
        if (status < 0)
        {
          Serial.print("GetFrame Error: ");
          Serial.println(status);
        }
    
        float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
  
        float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
        float emissivity = 0.95;
  
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640Background);
      }
//      long stopReadTime = millis();
//      Serial.print("Read rate: ");
//      Serial.print( 1000.0 / (stopReadTime - startTime), 2);
//      Serial.println(" Hz");
      tick += 1;
      if (false && tick > 10 ) {
        float maxReading = mlx90640To[0];
        float minReading = mlx90640To[0];
        float avgReading = mlx90640To[0];
        for (int x = 0 ; x < 768 ; x++)
        {
          if (isnan(mlx90640To[x])) {
            continue;
          }
          avgReading = (avgReading + mlx90640To[x]) / 2;
          if ( mlx90640To[x] > maxReading) {
            maxReading = mlx90640To[x];
          }
          if ( mlx90640To[x] < minReading) {
            minReading = mlx90640To[x];
          }
        }
        avgReading = avgReading * 1.8 + 32;
        maxReading = maxReading * 1.8 + 32;
        minReading = minReading * 1.8 + 32;
        String output = "Max:";
        output.concat(maxReading);
        String minOutput = "Min:";
        minOutput.concat(minReading);
        String avgOutput = "Avg:";
        avgOutput.concat(avgReading);
        //oled.setCursor(0, 0);
        //oled.clear(PAGE); // Clear the buffer
        //oled.print(output);
        //oled.setCursor(0, 10);
        //oled.print(minOutput);
        //oled.setCursor(0, 20);
        //oled.print(avgOutput);
        //oled.display(); // Draw on the screen
        tick = 0;
      }
      /* time to block the task until the queue has free space */
      const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
      if(skipFrame==false)
        xQueueSendToFront( xQueue, &mlx90640Background, xTicksToWait );
      
      const TickType_t xDelay = 20/ portTICK_PERIOD_MS; // 8 Hz is 1/8 second
      vTaskDelay(xDelay);
  }
}

void receiveTask( void * parameter )
{
  /* keep the status of receiving data */
  BaseType_t xStatus;
  /* time to block the task until data is available */
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  for(;;){
    /* receive data from the queue */
    xStatus = xQueueReceive( xQueue, &mlx90640To, xTicksToWait );
    /* check whether receiving is ok or not */
    if(xStatus == pdPASS){
      compressAndSend();
      total += 1;
    }
  }
  vTaskDelete( NULL );
}

char* strJump(char* str,char* pattern)
{
  char * searchRes=strstr(str,pattern);
  if(searchRes==NULL)return NULL;

  return searchRes+strlen(pattern);
}


char* strOriginJump(char* str,char* pattern)
{
  if(str==NULL || pattern==NULL)return NULL;
  while(*str!='\0')
  {
    
    if(*pattern=='\0')
    {
      return str;
    }
    
    if(*pattern!=*str)
    {
      break;
    }
    
    str++;
    pattern++;
  }
  if(*pattern=='\0')
  {
    return str;
  }

  return NULL;
}



int clidentCount=0;
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    
    switch(type) {
        case WStype_DISCONNECTED:
            clidentCount--;
            Serial.println("Socket Disconnected.");
            break;
        case WStype_CONNECTED:
            {
                clidentCount++;
                IPAddress ip = webSocket.remoteIP(num);
                Serial.println("Socket Connected.");
                // send message to client
                webSocket.sendTXT(num, "Connected");
            }
            break;
        case WStype_TEXT:
            // send message to client
            {
              Serial.println((char*)payload);

              
              const char *curs=strJump((char*)payload,"\"speedIdx\":");
              Serial.println((char*)curs);
              if(curs!=NULL)
              {
                int offset=12;
                const char* nPayLoad=curs;
                if(nPayLoad[0]>='0' && nPayLoad[0]<='5')
                {
                  frameSpeedIdx=nPayLoad[0]-'0';
                }
                
              }
            }
            break;
        case WStype_BIN:
        case WStype_ERROR:      
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
            break;
    }
}


bool TXLedToggle=false;
// Some precision is lost during compression but data transfer speeds are
// much faster. We're able to get a higher frame rate by compressing data.




void compressAndSend() 
{
    ledcWrite(LED_CH, TXLedToggle?200:20);
    if(clidentCount==0){
      TXLedToggle=false;
      return;
    }
    TXLedToggle=!TXLedToggle;
    int16_t sendBin[768];
    
    for (int x = 0 ; x < 768; x += 1)
    {
        sendBin[x]= round(mlx90640To[x] *100);
    }
    webSocket.broadcastBIN((uint8_t* )sendBin,sizeof(sendBin));
}
