TODO: /cmd_grip command shoudl be boolean for maximum efficiency
TODO: in stm code before taking action check current gripper state

# setup:
1. export the assets/models path to gz sim 
`export GZ_SIM_RESOURCE_PATH=~/.gz/sim/models`

0. in the megajaw.xacro.urdf manually switch depending on simulation or real life (todo make it auto switch)
0.1. in index.js change the connection port depending on if real life or simulation (todo change this too so it changes dynamically)
1. run simulation: ros2 launch megajaw_bringup gz.launch.py
2. run the web_interface/index.html
3. start recording: ros2 bag record /camera/image/compressed


for Estimating Depth we rely on Pinhole cam model
Z = (w * Fx) / p
W: real world width
p: measured pixels width
Fx: focal length
    Fx = (P_calib * Z_calib) / W
    or Fx can be acheives from teh calibration matrix
    [
        [fx 0 cx]
        [0 fy cy]
        [0 0 1]
    ]
     recommended to run cv2.undisort and give the disortion coeffs.
     
     Ideal world: Fx = img_width / (2 * tan(HFOV / 2))
