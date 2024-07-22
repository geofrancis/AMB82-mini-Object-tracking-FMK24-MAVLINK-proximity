/*

 Example guide:
 https://www.amebaiot.com/en/amebapro2-arduino-neuralnework-object-detection/

 NN Model Selection
 Select Neural Network(NN) task and models using .modelSelect(nntask, objdetmodel, facedetmodel, facerecogmodel).
 Replace with NA_MODEL if they are not necessary for your selected NN Task.

 NN task
 =======
 OBJECT_DETECTION/ FACE_DETECTION/ FACE_RECOGNITION

 Models
 =======
 YOLOv3 model         DEFAULT_YOLOV3TINY   / CUSTOMIZED_YOLOV3TINY
 YOLOv4 model         DEFAULT_YOLOV4TINY   / CUSTOMIZED_YOLOV4TINY
 YOLOv7 model         DEFAULT_YOLOV7TINY   / CUSTOMIZED_YOLOV7TINY
 SCRFD model          DEFAULT_SCRFD        / CUSTOMIZED_SCRFD
 MobileFaceNet model  DEFAULT_MOBILEFACENET/ CUSTOMIZED_MOBILEFACENET
 No model             NA_MODEL
 */


#include <Arduino.h>
#include <floatToString.h>
#include "mavlink/common/mavlink.h"

unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 3000;        // interval at which to blink (milliseconds)

int base = 0;
int custom = 0;
int mode = 0;
unsigned long previousMillis2 = 0;
const long interval2 = 200;

int volt = 0;
int current = 0.9;

float roll = 0;
float pitch = 0;

char messages[50];



unsigned long previousMil = 0; 
const long inter = 3000;


int steering = 0;
int range = 0;

uint16_t distances[72];

uint16_t targets[5];
int i;
int target = 0;
int oldtarget = 0;

int minimum = 50;
int maximum = 2000;

#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "AudioStream.h"
#include "AudioEncoder.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"

#define CHANNEL 0
#define CHANNELNN 3

// Lower resolution for NN processing
#define NNWIDTH 640
#define NNHEIGHT 480
VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNObjectDetection ObjDet;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO videoStreamerNN(1, 1);

char ssid[] = "2.4GN";       // your network SSID (name)
char pass[] = "passw0rded";  // your network password
int status = WL_IDLE_STATUS;

IPAddress ip;
int rtsp_portnum;

int R;

#include <ArduinoSort.h>

const int send = 11;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);  //RXTX from Pixhawk
  Serial2.begin(57600);   //RADAR

 pinMode(send, OUTPUT);
 digitalWrite(send, LOW);


  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 2 seconds for connection:
    delay(2000);
  }
  ip = WiFi.localIP();

  // Configure camera video channels with video format information
  // Adjust the bitrate based on your WiFi network quality
  config.setBitrate(2 * 1024 * 1024);  // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
  Camera.configVideoChannel(CHANNEL, config);
  Camera.configVideoChannel(CHANNELNN, configNN);
  Camera.videoInit();

  // Configure RTSP with corresponding video format information
  rtsp.configVideo(config);
  rtsp.begin();
  rtsp_portnum = rtsp.getPort();

  // Configure object detection with corresponding video format information
  // Select Neural Network(NN) task and models
  ObjDet.configVideo(configNN);
  ObjDet.setResultCallback(ODPostProcess);
  ObjDet.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV4TINY, NA_MODEL, NA_MODEL);
  ObjDet.begin();

  // Configure StreamIO object to stream data from video channel to RTSP
  videoStreamer.registerInput(Camera.getStream(CHANNEL));
  videoStreamer.registerOutput(rtsp);
  if (videoStreamer.begin() != 0) {
    Serial.println("StreamIO link start failed");
  }

  // Start data stream from video channel
  Camera.channelBegin(CHANNEL);

  // Configure StreamIO object to stream data from RGB video channel to object detection
  videoStreamerNN.registerInput(Camera.getStream(CHANNELNN));
  videoStreamerNN.setStackSize();
  videoStreamerNN.setTaskPriority();
  videoStreamerNN.registerOutput(ObjDet);
  if (videoStreamerNN.begin() != 0) {
    Serial.println("StreamIO link start failed");
  }

  // Start video channel for NN
  Camera.channelBegin(CHANNELNN);

  // Start OSD drawing on RTSP video channel
  OSD.configVideo(CHANNEL, config);
  OSD.begin();


  memset(distances, UINT16_MAX, sizeof(distances));  // Filling the distances array with UINT16_MAX

 

  uint8_t system_id = 1;
  uint8_t component_id = 2;
  uint8_t severity = 1;
  uint16_t id = 0;
  uint8_t chunk_seq = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_statustext_pack(system_id, component_id, &msg, 0, "CAMERA ONLINE", id, chunk_seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial3.write(buf, len);


}

void loop() {
  RADAR();
  HEARTBEAT();
}

void HEARTBEAT() {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
    uint8_t system_mode = MAV_MODE_PREFLIGHT;  ///< Booting up
    uint32_t custom_mode = 30;                 ///< Custom mode, can be defined by user/adopter
    uint8_t system_state = MAV_STATE_STANDBY;  ///< System ready for flight
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlink_msg_heartbeat_pack(1, 197, &msg, 0, autopilot_type, system_mode, custom_mode, system_state);
    len = mavlink_msg_to_send_buffer(buf, &msg);

    digitalWrite(send, HIGH);
    delay(10);
    Serial3.write(buf, len);
    Serial3.flush();
    digitalWrite(send, LOW);
  }
}

void RADAR() {
  target = Serial2.parseInt();  //dataIn now holds 0
  range = Serial2.parseInt();   //dataIn now holds 0
  if (target == 1) {
    targets[0] = range;
  }
  if (target == 2) {
    targets[1] = range;
  }
  if (target == 3) {
    targets[2] = range;
  }
  if (target == 4) {
    targets[3] = range;
  }
  if (target == 5) {
    targets[4] = range;
  }


  //Serial.print("radar");
  //Serial.println(targets[1]);
}





// User callback function for post processing of object detection results
void ODPostProcess(std::vector<ObjectDetectionResult> results) {
  uint16_t im_h = config.height();
  uint16_t im_w = config.width();

  Serial.print("Network URL for RTSP Streaming: ");
  Serial.print("rtsp://");
  Serial.print(ip);
  Serial.print(":");
  Serial.println(rtsp_portnum);
  Serial.println(" ");

  printf("Total number of objects detected = %d\r\n", ObjDet.getResultCount());
  OSD.createBitmap(CHANNEL);

  if (ObjDet.getResultCount() > 0) {
    for (i = 0; i < ObjDet.getResultCount(); i++) {
      int obj_type = results[i].type();
      if (itemList[obj_type].filter) {  // check if item should be ignored

        ObjectDetectionResult item = results[i];
        // Result coordinates are floats ranging from 0.00 to 1.00
        // Multiply with RTSP resolution to get coordinates in pixels
        int xmin = (int)(item.xMin() * im_w);
        int xmax = (int)(item.xMax() * im_w);
        int ymin = (int)(item.yMin() * im_h);
        int ymax = (int)(item.yMax() * im_h);

        // Draw boundary box
        printf("Item %d %s:\t%d %d %d %d\n\r", i, itemList[obj_type].objectName, xmin, xmax, ymin, ymax);
        OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE);


        // Print identification text
        char text_str[20];
        snprintf(text_str, sizeof(text_str), "%s %d", itemList[obj_type].objectName, targets[i] );
        OSD.drawText(CHANNEL, xmin, ymin - OSD.getTextHeight(CHANNEL), text_str, OSD_COLOR_CYAN);
        uint16_t distancey[20];
        uint16_t x[20];

        //send mavlink message
        distancey[i] = ymin;

        Serial.print("xmax ");
        Serial.println(xmax);
        Serial.print("xmin ");
        Serial.println(xmin);

        x[i] = (xmin + ((xmax - xmin) / 2));

        //sortArray(distancey, 20);


        Serial.print("distancey ");
        Serial.println(distancey[i]);
        steering = map(x[i], 0, 1920, 0, 60);

unsigned long currentMillis = millis();

  if (currentMillis - previousMil >= inter) {
    // save the last time you blinked the LED
    previousMil = currentMillis;

       

  uint8_t system_id = 1;
  uint8_t component_id = 197;
  uint8_t severity = 1;
  uint16_t id = 0;
  uint8_t chunk_seq = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_statustext_pack(system_id, component_id, &msg, 0, text_str, id, chunk_seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial3.write(buf, len);
    }
      
        
      
    


    // Initialize the required buffers
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // Pack the message
    Serial.println("target sent");
    int sysid = 1;
    //< The component sending the message.
    int compid = 197;
    uint64_t time_usec = 0;
    uint8_t sensor_type = 0;
    distances[steering] = targets[i];  //UINT16_MAX gets updated with actual distance values
    uint8_t increment = 2;
    uint16_t min_distance = 200;
    uint16_t max_distance = 20000;
    float increment_f = 0;
    float angle_offset = -60;
    uint8_t frame = 12;
    mavlink_msg_obstacle_distance_pack(sysid, compid, &msg, time_usec, sensor_type, distances, increment, min_distance, max_distance, increment_f, angle_offset, frame);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    digitalWrite(send, HIGH);
    delay(10);
    Serial3.write(buf, len);
    Serial3.flush();
    digitalWrite(send, LOW);


    Serial.print("Steering ");
    Serial.println(steering);
    Serial.print("targets");
    Serial.println(targets[i]);

    Serial.println();


    memset(distances, UINT16_MAX, sizeof(distances));
    targets[1] = 0;
    targets[2] = 0;
    targets[3] = 0;
    targets[4] = 0;
    targets[5] = 0;
    OSD.update(CHANNEL);
  }
}
  }
}
