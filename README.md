# Ready-to-serve Trash Bin
By collaboration of [Dongming Liu](https://github.com/MeanPaper), [Owen Xu](https://github.com/jinyuxu2), [Josh Litao](https://github.com/jlitao2) <br>
UIUC SP24 Senior Project Repository (Team 19) | January 2024 - May 2024

## About the Project
Throwing away trash is a simple task that many people take for granted. However, those with little to no mobility as a result of a disability, hospitalization, natural aging, or other health conditions struggle to carry out this necessary task. According to the CDC, 12.1 percent of U.S. adults have a mobility disability with serious difficulty walking or climbing stairs. As a result, these people either require an assistant to dispose of their trash for them, which may not always be feasible, or they are forced to hold their trash and let it accumulate by their side. Letting trash accumulate is a sanitary concern that could escalate into further problems. A trash bin could be placed next to the person, but this solution has various problems. An open trash bin would allow the odor of the trash to spread throughout the room, and a bin with a lid could pose difficulty for users whose conditions make them unable to open the lid directly with their hands or use their foot to press the pedal.

In order to eliminate the problems with existing trash bins for people with limited mobility, we propose a trash bin that would be ready to take a user’s trash once they perform a particular hand gesture to call it. A camera will be part of the motion and object detection system. This system to detect the hand gesture would be placed somewhere in the room where it would be able to monitor whether the user needs the trash bin to pick up their trash. The trash bin would be attached to a set of wheels to allow it to move. The lid of the bin would also be controlled to open and close. Once the camera detects that the user wishes to dispose of trash, the camera system would wirelessly communicate with the bin to prompt the bin to move toward the user. Upon arriving at the user, the lid would open, ready to collect the user’s trash. Once the user has disposed of trash into the bin, the lid would close, and the bin would return to its resting position. This solution simplifies the process of throwing trash by only requiring the users to call it and drop their trash into the bin.

## Project Video and Presentation
[Video](https://www.youtube.com/embed/DfU_nMM2fV8)<br>
[Presentation](https://courses.engr.illinois.edu/ece445/getfile.asp?id=23341)

## Usage 
### Hand Gesture Detection
For hand gesture detection, we recommend [Raspberry Pi 4B with 4GB RAM](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) with [64-Bit Raspberry Pi OS](https://www.raspberrypi.com/software/operating-systems/). The project uses this [Pi Camera](https://www.amazon.com/Arducam-Megapixels-Sensor-OV5647-Raspberry/dp/B012V1HEP4?th=1) in our project but other Pi Camera or USB camera should work as well. 

The required packages for the hand gesture detection can be found [here](https://github.com/MeanPaper/ready_to_serve_trash_bin/blob/main/hand-gesture-recognition-mediapipe/README.md#requirements). To make package installation easier, we recommend setting up a virtual python environment using [`conda`](https://conda.io/projects/conda/en/latest/user-guide/getting-started.html) or [`venv`](https://docs.python.org/3/tutorial/venv.html). 

### Running Hand Gesture Detection and Training
More details can be found in the [hand-gesture-recognition-mediapipe](https://github.com/MeanPaper/ready_to_serve_trash_bin/blob/main/hand-gesture-recognition-mediapipe/README.md).

## Acknowledgements
- [Hand guesture recognition mediapipe](https://github.com/kinivi/hand-gesture-recognition-mediapipe/tree/main): computer vision for hand guesture recognition.
- [COMTool](https://github.com/Neutree/COMTool): easy-to-use debugging tool, for gathering data from serial port.

## References
- [QuickPID](https://github.com/Dlloydev/QuickPID/tree/master)
- [ESPAsyncWebServer](https://github.com/esphome/ESPAsyncWebServer)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/index.html)
- [ESP32-S3-WROOM-1 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf)
- [L298](https://www.st.com/content/ccc/resource/technical/document/datasheet/82/cc/3f/39/0a/29/4d/f0/CD00000240.pdf/files/CD00000240.pdf/jcr:content/translations/en.CD00000240.pdf)
- [TPS56220x 4.5V to 17V Input, 2A Synchronous Step-Down Voltage Regulator in SOT-23](https://www.ti.com/lit/ds/symlink/tps562201.pdf)
- [TLV1117LV 1-A, Positive Fixed-Voltage, Low-Dropout Regulator](https://www.ti.com/lit/ds/symlink/tlv1117lv.pdf)
- [Low Dropout Linear Regulator](https://www.diodes.com/assets/Datasheets/AZ1117C.pdf)

## Academic Integrity
Please review the University of Illinois Student Code, particularly all subsections of [Article 1, Part 4 - Academic Integrity Policy and Procedure](https://studentcode.illinois.edu/article1/part4/1-401/).
