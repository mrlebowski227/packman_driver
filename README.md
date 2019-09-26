# packman_driver
ROS interface for the Packman Sigmatek driver.

- Het beste om Ubuntu te installeren is om een Linux kernel via de windows store te installeren. Dat is ook gewoon een terminal van Ubuntu dus je kunt daar alles op als dat je met een VM zou werken. Als je het anders wilt moet je zelf een VM opzetten of een dualboot opzetten.

https://www.microsoft.com/en-us/p/ubuntu/9nblggh4msv6?activetab=pivot:overviewtab

In de description van de app staat welke acties je moet ondernemen voor het installeren van Ubuntu.


- Nadat je klaar bent moet je nog updaten en upgraden om de meest recente dependancies in je systeem te krijgen.


- Dan kun je deze pagina volgen:

http://wiki.ros.org/melodic/Installation/Ubuntu

Ik raad je sterk aan om voor de ros-base variant te gaan gezien je geen RVIZ nodig hebt of andere componenten. Bespaard tijd en geheugen.


- Nadat je ROS hebt geïnstalleerd en je catkin_ws hebt aangemaakt heb je nog het volgende nodig:

sudo apt install ros-melodic-rosbash

pip install opcua


- Hierna kun je de packman_driver package clonen in je src directory van de catkin_ws en kun je compilen (~/catkin_ws/src).

dit doe je met 'git clone https://github.com/mrlebowski227/packman_driver'

Vergeet niet om te checken of driver_opcua.py 'executable' is. Je kunt voor de zekerheid naar de src map gaan van de driver package en dan deze commando uit voeren (~/catkin_ws/src/packman_driver/src):

sudo chmod +x driver_opcua.py


- Dan kun je nog eens 'source ~/.bashrc' doen om de package in je catkin_ws ontdekbaar te maken voor bash.

Vergeet niet om het ip adres en port aan te passen naar jouw voorkeur.

En dan kun je 'roslaunch packman_driver packman_driver.launch' doen en moet het gaan werken.
