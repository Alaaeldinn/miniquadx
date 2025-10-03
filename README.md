# miniquadx

MiniQuadX is a lightweight and compact mini quadcopter project based on the ESP32-CAM Board 

not the standard methodology , you can use motor drivers , esc , different types of motors ... etc 

![MiniQuadX](/media/miniquad.jpeg)

**Hardware components:**
(thinking out loud , what you need)
  1. Esp32-cam board
  2. Lipo Battery
  3. 4x MOSFET , Diodes (alternate :  motor drivers)
  5. 4x Mini Brushed DC Motor

# SETUP 
 
 make sure that ROS, arduino ide installed ..

```bash
chmod +x esp32quad.py
```

```bash
roscore
```

```bash
rosrun esp32quad.py
```

# TODO
-[ ] Optimization
-[ ] Better readme
-[ ] Unit Tests
-[ ] Control Metrics
-[ ] AUX features 
-[ ] Compact design
