#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "Proj_Setup.h"

// need to register to UIUC network
// const char * ssid = "The Retreat";
const char * ssid = "IllinoisNet_Guest";
const char * pass = NULL;

AsyncWebServer server(80); // use port 80, http

binState current_state = STOP;

void handleRoot(AsyncWebServerRequest *);
void handleStopMotors(AsyncWebServerRequest *);
void handleSetSpeed(AsyncWebServerRequest *);
void handleLid(AsyncWebServerRequest *);
void moveForward(AsyncWebServerRequest *);
void moveBackward(AsyncWebServerRequest *);
void LeftTurn(AsyncWebServerRequest *);
void RightTurn(AsyncWebServerRequest *);


void setup(void) {
	Serial.begin(9600);
    // Serial.println(WiFi.macAddress()); // MAC addr: 34:85:18:50:4B:54

	// wifi connection
	WiFi.begin(ssid, pass);
	Serial.printf("Connecting to %s", ssid);
	while(!WiFi.isConnected()){ // attempt to connect
	  Serial.print(".");
	  delay(1000);
	}
	Serial.println();

	// output IP address for user connection
	Serial.println("WiFi IP: ");
	Serial.println(WiFi.localIP());

	// async server handling
	server.on("/", HTTP_GET, handleRoot);
	server.on("/stop_motors", HTTP_POST, handleStopMotors);
	server.on("/set_speed", HTTP_POST, handleSetSpeed);
    server.on("/set_lid", HTTP_POST, handleLid);

    // more services goes here
	server.begin();

    // // motors setups
	// stopDCMotor();
	// setupMotorInterrupt();
    // setupLinearActuator();
    // QuickPID_Init();
	// // for debug
	// setTargetTicksPerFrame(0,0);

}

void loop(void) {
	// // plotData();
	// // parseCmd();
	// // checkLid();

    if(current_state == LID){
        checkLid();
    }
}

// web service handlers
void handleRoot(AsyncWebServerRequest * request){ // use this for debugging I guess
	Serial.println("Request root");
	request->send(200, "html", "<h1>Welcome to ESP32-S3</h1>");
}

// // force stop
void handleStopMotors(AsyncWebServerRequest * request){ // maybe set a flag, rather than this
	Serial.println("Request stop");
	stopDCMotor();
	request->send(200);
}

void handleSetSpeed(AsyncWebServerRequest * request){
	Serial.println("Request set speed");

    JsonDocument doc;
    String response;
    doc["state"] = current_state;

    // LID state, blocking commands
    if(current_state == LID){
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    }

	float speedLeft = 0;
	float speedRight = 0;
	if(request->hasParam("speedLeft", true)){
		speedLeft = request->getParam("speedLeft", true)->value().toFloat();
	}
	if(request->hasParam("speedRight",true)){
		speedRight = request->getParam("speedRight",true)->value().toFloat();
	}

    Serial.printf("Left: %f, Right: %f\n", speedLeft, speedRight);
	setTargetSpeed(speedLeft, speedRight);

    // return different states
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

// need to do some adjustment
void handleLid(AsyncWebServerRequest * request){
    Serial.println("Request lid operation");
    
    stopDCMotor();
    current_state = LID;
    setLid();

    request->send(200);
}

// more service handlers go here


