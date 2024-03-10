#include <Arduino.h>
// #include <WiFi.h>
// #include <ESPAsyncWebServer.h>
#include "Proj_Setup.h"
#define DEBUG 0

#if (DEBUG)
#else

#endif

// const char * ssid = "The Retreat";
// const char * pass = NULL;

// AsyncWebServer server(80); // use port 80, http

// void handleRoot(AsyncWebServerRequest *);
// void handleStopMotors(AsyncWebServerRequest *);
// void handleSetSpeed(AsyncWebServerRequest *);
// void handleSpeedUp(AsyncWebServerRequest *);
// void handleSlowDown(AsyncWebServerRequest *);

void setup(void) {
	Serial.begin(9600);
	
	// wifi connection
	// WiFi.begin(ssid, pass);
	// Serial.printf("Connecting to %s", ssid);
	// while(!WiFi.isConnected()){ // attempt to connect
	//   Serial.print(".");
	//   delay(1000);
	// }
	// Serial.println();

	// // output IP address for user connection
	// Serial.println("WiFi IP: ");
	// Serial.println(WiFi.localIP());

	// // async server handling
	// server.on("/", HTTP_GET, handleRoot);
	// server.on("/stop_motors", HTTP_GET, handleStopMotors);
	// server.on("/set_speed", HTTP_GET, handleSetSpeed);
	// server.on("/speedup", HTTP_POST, handleSpeedUp);
	// server.on("/slowdown", HTTP_POST, handleSlowDown);
	// server.begin();

	// setupAllMotors();
	// setupMotorOne();

	setupMotorInterrupt();
	setTargetTicksPerFrame(0,0);

}

void loop(void) {
  	// PID_compute();
	plotData();
}

// web service handlers
// void handleRoot(AsyncWebServerRequest * request){ // use this for debugging I guess
// 	Serial.println("Request root");
// 	request->send(200, "html", "<h1>Welcome to ESP32-S3</h1>");
// }

// // force stop
// void handleStopMotors(AsyncWebServerRequest * request){ // maybe set a flag, rather than this
// 	Serial.println("Request stop");
// 	stopDCMotor();
// 	request->send(200, "application/json", "");
// }

// void handleSetSpeed(AsyncWebServerRequest * request){
// 	Serial.println("Request set speed");
// 	double speedOne = 0;
// 	double speedTwo = 0;
// 	if(request->hasParam("speedOne")){
// 		speedOne = request->getParam("speedOne")->value().toDouble();
// 	}
// 	if(request->hasParam("speedTwo")){
// 		speedTwo = request->getParam("speedTwo")->value().toDouble();
// 	}
// 	setSpeed(speedOne, speedTwo);
// 	request->send(200, "application/json", "");
// }

// void handleSpeedUp(AsyncWebServerRequest * request){
//   Serial.println("Request speed up");
//   request->send(200, "application/json", "");
// }

// void handleSlowDown(AsyncWebServerRequest * request){
//   Serial.println("Request slow down");
//   request->send(200, "application/json", "");
// }


