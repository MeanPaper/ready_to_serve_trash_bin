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
    // long temp = millis();
    // while(millis() - temp < 2000);

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

    // motors setups
	stopDCMotor();
	setupMotorInterrupt();
    setupLinearActuator();
    QuickPID_Init();
	
	// for debug
	#if (DEBUG)
	setTargetTicksPerFrame(0,0);
	#endif
}

// main loop, cmd executing
void loop(void) {

	/* testing, debugging begin */
	#if (DEBUG)
	// plotData();
    // Serial.println("Hello");
	// checkLid();
    // Serial.println("Hello World");
	#endif
	/* testing, debugging end */

	switch(current_state){
		case LID: 
			// this event reject all the commands until this event finishes
			if(checkLid() == CLOSE){
				#if (DEBUG)
                Serial.println("Lid operation done");
				#endif
				current_state = STOP;
			}

			if(millis() - recordTime > 2000){ // turn off h-bridges for dc motors when stop for 2000ms
				motorsOff();
			}

			break;
		case STOP:
			stopDCMotor(); // stop all DC motors

			if(millis() - recordTime > 5000){ // turn off h-bridges for dc motors when stop for 2000ms
				motorsOff();
			}
            else{
                QuickPID_Compute();
            }

			break;

		case BACKWARD:
		case FORWARD:
		case LEFT:
		case RIGHT:
		case YOLO:
			QuickPID_Compute();
			// speedAutoAdjust();
			if(millis() - recordTime > currentDuration){

				#if (DEBUG)
                Serial.printf("Current Time: %d\n", millis());
                Serial.printf("Recorded Time: %d\n", recordTime);
				Serial.print(stateToString(current_state).c_str());
				Serial.println(" Done");
				#endif

				current_state = STOP;
				stopDCMotor();
                recordTime = millis();
			}
			break;
		default: break;
	}
}

/** web service handlers section **/
void handleRoot(AsyncWebServerRequest * request){ // use this for debugging I guess

	#if (DEBUG)
	Serial.println("Request root");
	#endif

	request->send(200, "html", "<h1>Welcome to ESP32-S3</h1>");
}

// force stop
void handleStopMotors(AsyncWebServerRequest * request){ // need to think about this...

	#if (DEBUG)
	Serial.println("Request stop");
	#endif

	JsonDocument doc;
	String response;	

	if(current_state == LID){

		#if (DEBUG)
		Serial.println("In LID state, motors have stop already");
		#endif

		doc["state"] = stateToString(LID);
		serializeJson(doc, response);
		request->send(200, "application/json", response);
		return;
	}

	stopDCMotor();

	// data to the client
	doc["state"] = stateToString(STOP);
	current_state = STOP;
	serializeJson(doc, response);

	recordTime = millis();
	request->send(200, "application/json", response);
}

// this function is not finished yet
void handleSetSpeed(AsyncWebServerRequest * request){
	
	#if (DEBUG)
	Serial.println("Request set speed");
	#endif

    JsonDocument doc;
    String response;

    // LID state, blocking commands
    if(current_state == LID){
		
		#if (DEBUG)
        Serial.println("BLOCK: lid is operating");
		#endif

    	doc["state"] = stateToString(LID);
        serializeJson(doc, response);
        request->send(200, "application/json", response);
        return;
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

	#if (DEBUG)
    Serial.printf("Left: %f, Right: %f, duration: %ld\n", speedLeft, speedRight, duration);
	#endif

	setTargetSpeed(speedLeft, speedRight);
	
	current_state = YOLO;
	doc["state"] = stateToString(current_state);
    serializeJson(doc, response);

	recordTime = millis(); // recording end time

    request->send(200, "application/json", response);
}

// need to do some adjustment
void handleLid(AsyncWebServerRequest * request){
	
	#if (DEBUG)
    Serial.println("Request lid operation");
	#endif

    JsonDocument doc;
	String response;
	
	// start operations
    stopDCMotor();
    setLid();

	// response data to the client
    current_state = LID;
	doc["state"] = stateToString(current_state);
	serializeJson(doc, response);

	recordTime = millis();
    request->send(200, "application/json", response);
}

// more service handlers go here
void turnLeftRight(AsyncWebServerRequest * request){ // turn the bin left or right
	#if (DEBUG)
	Serial.println("Request turn left or right");
	#endif

	JsonDocument doc;
	String response;

	// LID state, blocking commands
	if(current_state == LID){
		#if (DEBUG)
        Serial.println("BLOCK: lid is operating");
		#endif

		doc["state"] = stateToString(LID);
		serializeJson(doc, response);
		request->send(200, "application/json", response);
        return;
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

	doc["state"] = stateToString(current_state);
    
    #if (DEBUG)
    Serial.printf("duration: %ld, speed: %f, direction: ", duration, speed);
    Serial.print(stateToString(current_state).c_str());
    Serial.println();
	#endif

	// return different states
	serializeJson(doc, response);
	recordTime = millis();
	request->send(200, "application/json", response);
}

void moveForwardBackward(AsyncWebServerRequest * request){ // move the bin forward or backward
	#if (DEBUG)
	Serial.println("Request move forward or backward");
	#endif

	JsonDocument doc;
	String response;

	// LID state, blocking commands
	if(current_state == LID){

        #if (DEBUG)
        Serial.println("BLOCK: lid is operating");
        #endif
		
		doc["state"] = stateToString(LID);
		serializeJson(doc, response);
		request->send(200, "application/json", response);
        return;
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

	#if (DEBUG)
    Serial.printf("duration: %ld, speed: %f, direction: ", duration, speed);
    Serial.print(stateToString(current_state).c_str());
    Serial.println();
	#endif

	// return different states
	serializeJson(doc, response);
	recordTime = millis();
	request->send(200, "application/json", response);
}

std::string stateToString(binState state) {
    static const std::unordered_map<binState, std::string> stateMap = {
        {binState::LID, "LID"},
        {binState::STOP, "STOP"},
        {binState::BACKWARD, "BACKWARD"},
        {binState::FORWARD, "FORWARD"},
        {binState::LEFT, "LEFT"},
        {binState::RIGHT, "RIGHT"},
		{binState::YOLO, "YOLO"}
    };

    auto it = stateMap.find(state);
    if (it != stateMap.end()) {
        return it->second;
    } else {
        return "UNKNOWN";
    }
}
