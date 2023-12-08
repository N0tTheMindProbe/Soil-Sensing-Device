**Overview** 

Welcome to Twin Cities Engineering’s soil gas sensing project. Here, you will find full documentation for both the hardware and software components of our newly developed soil gas sensing device. 

Soil health is a key part within the realm of agriculture and environmental sustainability. As such, understanding the composition of gases within soil void spaces is a powerful means to monitor soil health, offering crucial insights into nutrient content and microorganism activity. Unfortunately, existing commercial soil gas measurement devices are prohibitively expensive, limiting access for farmers and soil scientists alike. To bridge this gap, our team of student engineers is crafting an affordable DIY soil gas sensing device designed to measure carbon dioxide concentrations at three distinct soil depths every 2-3 hours. Our goal is to create a prototype that is not only economical but also portable and user-friendly, increasing accessibility to those wishing to measure and track soil health. 

Our prototype works by collecting gas samples from soil void spaces. This is accomplished with three soil probes of varying lengths (10cm, 20cm, and 50cm) positioned at each of the aforementioned depths. A pump and solenoid valves are used to collect gas samples sequentially from each depth, so they can be passed over a carbon dioxide sensor. A Mayfly microcontroller is used to control each of these functions and record the gas data collected from the sensor. This dynamic setup allows for precise monitoring and data collection, providing a cost-effective and efficient solution for soil health analysis. 

The development of the programming for this device was made possible through the utilization of resources provided by EnviroDIY's ModularSensors repository. The extensive collection of tutorials, libraries, and code documentation available in the ModularSensor repository allowed our team to streamline the programming process and ensure consistent functionality of the soil gas sensing prototype.  

**Quick Start Guide** 

The following is a quick start guide for those wishing to assemble their own soil sensing device. 

Prerequisites: 

Creating a replicated version of our soil sensing prototype does not require extensive programming expertise, making it accessible to those with minimal prior knowledge. However, having the right Integrated Development Environment (IDE) is crucial for compiling and uploading the necessary code to the microcontroller. We highly recommend using VScode with the PlatformIO extension for this purpose. The code for this device relies on numerous libraries from EnviroDIY’s ModularSensor repository, Adafruit, as well as locally sourced ones. The PlatformIO extension conveniently manages and sources all these libraries, saving individuals from having to manage all these libraries themselves. For this reason, we strongly advise using this configuration over simpler compilers like the Arduino IDE. 

Both VSCode and PlatformIO are free to install and run on both Windows and Mac. A detailed tutorial by EnviroDIY on how to download and configure these tools for our project can be found here. 

**Hardware requirements:** 

We selected the Mayfly Data Logger as the microcontroller for this project. Designed by the EnviroDIY community, the Mayfly microcontroller was specifically tailored with environmental data monitoring in mind. For additional hardware specifications and purchasing links, please refer to the EnviroDIY website. 

**Software dependencies/Installation instructions:** 

To install the software for the Mayfly controller, download the ZIP file for the project and open it using PlatformIO. With the project opened, the workstation in VSCode should look similar to this: 

![image](https://github.com/bellahenkel/Soil-Sensing-Device/assets/45954420/55ec0795-b579-4bbb-8566-ed68927000d7)


In the examples folder, there are a variety of individual files for each of the main mechanisms of the sensor. These can be ignored, unless individual testing/calibration of components is required.  

To upload the code to the Mayfly device, ensure the connection between the Mayfly and the computer is secure, and that VSCode recognizes the device. Click the checkmark (PlatformIO: Build) located at the bottom left of the screen. VSCode may take several seconds to install and compile each of the libraries required for the prototype to run. Afterwards, the terminal should show a “success” message.  

Clicking the arrow button to the right of the compile checkmark will upload the code to the Mayfly device. Again, this can take up to a minute to complete. Once the code has been successfully uploaded to the Mayfly device, further testing in the Serial Monitor can be done to ensure that the prototype is working as expected. Otherwise, the Mayfly can be unplugged from the computer and installed into the device.   

**Hardware Components** 

Detailed documentation regarding complete hardware assembly (such as component lists, electrical schematics, and pin configurations for the Mayfly) will be located in the docs folder as it becomes available.  

**Software Architecture** 

The general process for the prototype’s programming is shown in the following diagram.  

<img width="579" alt="Screenshot 2023-12-08 at 5 53 21 AM" src="https://github.com/bellahenkel/Soil-Sensing-Device/assets/45954420/ea4bf004-71f4-4ced-b8ab-583f6c3b25ec">


There are numerous libraries referenced within the main code file that are required to operate the soil sensing device. Several libraries, such as the sensor libraries required for the gas and temperature sensors, are sourced from EnviroDIY’s ModularSensors. There are also a few local libraries, such as the “CollectSample.h”, which is required for operating the pump and solenoids to collect gas samples from the correct depths. All of these libraries work together in the main programming code to effectively operate the prototype.  

**Calibration Procedures for the Alphasense IRC-A1 Sensor** 

The Alphasense IRC-A1 sensor is the sensor utilized in this prototype for collecting carbon dioxide readings from the soil. This sensor comes pre-calibrated by the manufacturer and are generally ready for use immediately after purchase.  

The manufacturers recommend recalibrating the sensor no less than every 12 months to maintain the highest level of accuracy. Further information on sensor calibration procedures can be found on Alphasense’s website.   

**Troubleshooting** 

Output is “nullnullnull...” 

-Adjust the monitor_speed set in the platform.io file 

-Ensure the connection between the Mayfly and computer is secure (no faults in the cable, etc.) 

Output is coming at –9999 or Device only outputs –1250ppm or Readings are very low and/or negative: 

-Ensure that connection between Mayfly and sensor is correct 

-Ensure enough power is being fed to the CO2 sensor (It needs a minimum of 12 volts) 

-If the above solutions do not amend the issue, unplug the CO2 sensor from power for at least 30 seconds, and then plug it back in.  

“Avrdude” connection error: 

-Restart your computer/VSCode workstation.  

-Ensure the USB-C cord is undamaged.   

**License**  

Our soil sensing device project is released under the BSD 3-Clause License, a permissive open-source license that allows for the redistribution and modification of the source and binary forms of the software. This license provides flexibility for users to adapt and incorporate our technology into various applications, both commercial and non-commercial, while still maintaining certain conditions such as attribution and the inclusion of the original license terms. The BSD 3-Clause License aligns with our commitment to fostering collaboration and innovation in the field of soil sensing. 

**Third-Party Licenses** 

In the development of our project, we have incorporated specific third-party libraries, each governed by its respective license. These third-party components contribute essential functionality to our soil sensing device. We adhere to the terms and conditions outlined in the licenses of these libraries or components, and users are encouraged to review the individual licenses for detailed information. This transparent approach ensures that our project not only benefits from external contributions but also upholds the principles of open collaboration and proper attribution within the broader open-source community. 

**Acknowledgments** 

We would like to extend gratitude to the EnviroDIY community for their invaluable resources that have been instrumental to our team's success throughout the semester. The utilization of the ModularSensors repository proved to be vital for creating the programming for this project, serving as a vital resource that significantly contributed to our accomplishments. 

We would also like to thank Anthony Aufdenkampe for his guidance and the contributions he provided for this project. 

EnviroDIY’s ModularSensors libraries are provided “as-is” under the BSD-3 license. Full licensing/terms of use details can be found in the ModularSensors repository here. 
