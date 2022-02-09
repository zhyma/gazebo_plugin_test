## A sample of changing visual and physical properties of an object (link) in Gazebo
- It is a Gazebo world plugin that monitors a rostopic to change the appearance of the object in Gazebo simulation environment.
- Go to `models` folder, run `python3 move_model.py` to copy the model to the default Gazebo model folder.
- Compile workspace.
- In one terminal, run `roslaunch env_ctrl test.launch`
- In another terminal, run `rostopic pub -1 /set_rod_properties env_ctrl/CylinderProperties "{x: .4, y: .0, z: 0.3, r: 0.03, l: 0.1}"`. Values are arbitrary.
![](demo.gif)