# STR_Practice1_2021-2022
Project from Real Time Systems subject which It consists of building an electronic system of a wagon connected to a computer from an external server. This system must perform some operations within the specified period of time. 


## Structure

We have two systems involved in the control of the wagon:

 -  **Microcontroller (Arduino)**: This part will be in charge of obtaining information from the sensors and sending to the main controller

 -  **Main Controller:** This part collects the information from the microcontroller and execute the logical operations 

## Sensors in the Microcontroller

 1. **Velocimeter:** The Arduino will compute the speed at a certain period. This value will be send to the main controller. The speed is represented with a Led that has a variable brightness. The led will bright in a range between 40 and 70 km/h.  

       ![image](https://user-images.githubusercontent.com/79408013/152409765-43ef992c-0648-4b8a-907a-7faa2999fdff.png)

 3. **Light Sensor:** The wagon has a sensor to measure the light outside. When the value of the sensor is too low, it means that the wagon reaches a tunnel. So, this value will send to the maincontroller for setting the lights on
 
       ![image](https://user-images.githubusercontent.com/79408013/152410175-f1f99bb4-b52b-4b02-b7b8-2aed846ced87.png)        ![image](https://user-images.githubusercontent.com/79408013/152410281-aad1f97c-d1f8-4af3-a05f-a18c6e244184.png)
 
 4. **Read the Slope:** This sensor is going to detect a change on the slope of the wagon. This info will be received by the maincontroller to know if the wagon needs to brake or accelerate

       ![image](https://user-images.githubusercontent.com/79408013/152411129-f6781398-f9cc-4a59-9c25-2445cc0eba5c.png)

 5. **Activation of Mixer:** The brake system must be implemented using a LED that turns on when the brake is activated and turns off otherwise.

       ![image](https://user-images.githubusercontent.com/79408013/152412165-a001bd4f-b5bf-4fb9-91a1-f1d03a54f0f8.png)
       
 6. **Brake and Acceleration System:** The acceleration system must be implemented using a LED that turns on when the accelerator is activated and turns off otherwise. The brake system must be implemented using a LED that turns on when the brake is activated and turns off otherwise.

       ![image](https://user-images.githubusercontent.com/79408013/152412535-a2aca09a-5b0d-4fc1-ba29-a86a621b5c4e.png)  ![image](https://user-images.githubusercontent.com/79408013/152412608-61b132d9-ccb2-416a-84e9-f3c22d59635c.png)

All tasks should check periodically the information about those sensors and sending to the Maincontroller.
