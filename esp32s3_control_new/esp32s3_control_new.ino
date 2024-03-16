#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <unordered_map>
#include "Proj_Setup.h"

// need to register to UIUC network
// const char * ssid = "The Retreat";
const char * ssid = "IllinoisNet_Guest";
const char * pass = NULL;

AsyncWebServer server(80); // use port 80, http

binState current_state = STOP;
long currentDuration = 0;
long recordTime = 0;

// web service handlers
void handleRoot(AsyncWebServerRequest *);
void handleStopMotors(AsyncWebServerRequest *);
void handleSetSpeed(AsyncWebServerRequest *);
void handleLid(AsyncWebServerRequest *);
void moveForwardBackward(AsyncWebServerRequest *);
void turnLeftRight(AsyncWebServerRequest *);
std::string stateToString(binState state);

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
	server.on("/move", HTTP_POST, moveForwardBackward);
	server.on("/turn", HTTP_POST, turnLeftRight);

    // more services goes here
	server.begin();
	
	current_state = STOP;

    // // motors setups
	// stopDCMotor();
	// setupMotorInterrupt();
    // setupLinearActuator();
    // QuickPID_Init();
	// // for debug
	// setTargetTicksPerFrame(0,0);

}

// main loop, cmd executing
void loop(void) {
	// // plotData();
	// // parseCmd();
	// // checkLid();

	switch(current_state){
		case LID: 
			// this event reject all the commands until this event finishes
			if(checkLid() == CLOSE){
				current_state = STOP;
			}
			break;
		case STOP:
			stopDCMotor(); // stop all DC motors
			break;

		case BACKWARD:
		case FORWARD:
		case LEFT:
		case RIGHT:
			if(millis() - recordTime > currentDuration){
				Serial.print(stateToString(current_state));
				Serial.println(" Done");
				current_state = STOP;
				stopDCMotor();
			}
			break;
		default: break;
	}
}

/** web service handlers section **/

void handleRoot(AsyncWebServerRequest * request){ // use this for debugging I guess
	Serial.println("Request root");
	request->send(200, "html", "<h1>Welcome to ESP32-S3</h1>");
}

// force stop
void handleStopMotors(AsyncWebServerRequest * request){ // need to think about this...
	Serial.println("Request stop");
	// current_state = STOP;
	stopDCMotor();
	request->send(200);

	/* the problem with this event, need to store the state info of the lid */
	/* should it continue execute with the given info ? */
}

// this function is not finished yet
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

	// for other moving state
	float speedLeft = 0;
	float speedRight = 0;
	long duration = 0; // need to be miliseconds

	// get the control duration
	if(request->hasParam("duration", true)){
		duration = request->getParam("duration", true)->value().toInt();
		currentDuration = max(duration, (long)(0)); // guard negative values
	}

	// get the setting speed 
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

	recordTime = millis(); // recording end time
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
void turnLeftRight(AsyncWebServerRequest * request){ // turn the bin left or right
	Serial.println("Request turn left or right");
	JsonDocument doc;
	String response;

	// LID state, blocking commands
	if(current_state == LID){
		doc["state"] = stateToString(LID);
		serializeJson(doc, response);
		request->send(200, "application/json", response);
	}

	float speed = 0;
    long duration = 0;
	currentDuration = 5000;

	// get the control duration
	if(request->hasParam("duration", true)){
		duration = request->getParam("duration", true)->value().toInt();
		currentDuration = max(duration, (long)(0)); // guard negative values
	}

	// get the setting speed 
	if(request->hasParam("speed", true)){
		speed = request->getParam("speed", true)->value().toFloat();
	}

	// set the speed
	if(request->hasParam("direction", true)){
		String direction = request->getParam("direction", true)->value();
		if(direction == "left"){
			setTargetSpeed(-speed, speed);
			current_state = LEFT;
		}
		else if(direction == "right"){
			setTargetSpeed(speed, -speed);
			current_state = RIGHT;
		}
	}

	doc["state"] = stateToString(current_state);;

	// return different states
	serializeJson(doc, response);
	request->send(200, "application/json", response);

	recordTime = millis();
}

void moveForwardBackward(AsyncWebServerRequest * request){ // move the bin forward or backward
	Serial.println("Request move forward or backward");
	JsonDocument doc;
	String response;

	// LID state, blocking commands
	if(current_state == LID){
		doc["state"] = stateToString(LID);
		serializeJson(doc, response);
		request->send(200, "application/json", response);
	}

	float speed = 0;
    long duration = 0;
	currentDuration = 5000;

	// get the control duration
	if(request->hasParam("duration", true)){
		duration = request->getParam("duration", true)->value().toInt();
		currentDuration = max(duration, (long)(0)); // guard negative values
	}

	// get the setting speed 
	if(request->hasParam("speed", true)){
		speed = request->getParam("speed", true)->value().toFloat();
	}

	// set the speed
	if(request->hasParam("direction", true)){
		String direction = request->getParam("direction", true)->value();
		if(direction == "forward"){
			setTargetSpeed(speed, speed);
			current_state = FORWARD;
		}
		else if(direction == "backward"){
			setTargetSpeed(-speed, -speed);
			current_state = BACKWARD;
		}
	}

	doc["state"] = stateToString(current_state);

	// return different states
	serializeJson(doc, response);
	request->send(200, "application/json", response);
	
	recordTime = millis();
}

std::string stateToString(binState state) {
    static const std::unordered_map<binState, std::string> stateMap = {
        {binState::LID, "LID"},
        {binState::STOP, "STOP"},
        {binState::BACKWARD, "BACKWARD"},
        {binState::FORWARD, "FORWARD"},
        {binState::LEFT, "LEFT"},
        {binState::RIGHT, "RIGHT"}
    };

    auto it = stateMap.find(state);
    if (it != stateMap.end()) {
        return it->second;
    } else {
        return "UNKNOWN";
    }
}