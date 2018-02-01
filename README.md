# 2018Vision

Prerequisites:
1) Install latest version of raspbian
2) Boot on a MONITOR and execute the following:<br />
	a) Open Terminal<br />
	b) sudo raspi-config:<br />
<pre>    I) Change keyboard settings to U.S
    II) Change local and time zone to central
    III) Locate and enable ssh and camera
    IV) Exit out of configuration and reboot</pre> 
3) Go to a laptop and do the following: <br />
<pre>	a) Download and install Putty and WinSCP
	b) Open Terminal
	c) run "ping raspberrypi.local"
	d) If you get response in packets then that mean you already have a variable to store the IP
	e) Log into Putty using raspberrypi.local 
	f) If warning for data breach shows up, answer yes
	g) Log into the pi with username pi and password raspberry </pre>

Steps taken to install OpenCV using g++ and Cmake:

Start with the following commands:
