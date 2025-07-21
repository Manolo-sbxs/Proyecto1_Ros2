# Parte 4: Simulacion en Gazebo

-Para poder visualizar el robot en Gazebo, sera necesario abrir mas terminales que en Rviz, se debe seguir la siguiente rutina:

	1. Se abre el terminal 1, el cual servira para cargar el robot, en donde se deben ejecutar estos dos comandos seguidos:
	
	colcon build --packages-select triciclo_py
	source install/setup.bash
	
	2. Una vez cargados, se abrira el terminal 2, el cual sirve para cargar gazebo, para ello se ejecuta el siguiente comando, Esto abrira la interfaz de Gazebo pero sin el robot.:
	
	gz sim -v 4 ground_plane.sdf
	
	3. Se vuelve a la terminal 1 y se ejecuta el siguiente comando, el cual servira para poder visualizar el robot en Gazebo, en caso de que no cargue, apretar "ctrl+c" y ejecutar nuevamente el comando hasta que cargue el robot:
	
	ros2 launch triciclo_py gazebo.launch.py

	4. Se ejecuta el nodo de movimiento con los siguientes comandos:
	
	source install/setup.bash
	ros2 run triciclo_py mover_robot_cmdvel_rosgz
	
	5. Se puede ejecutar un archivo python de movimiento o utilizar teleoperacion, pero por problemas de un plugin el cual no se puede instalar, creo que por la version de Ros2 Jazzy Harmonic, no encontramos la manera de instalar el plugin necesario de movimiento, lo cual hace que el robot no le lleguen los mensajes "/cmd_vel" haciendo que no se mueva, se intentaron varias formas de instalarlo y varias formas de darle movimiento al robot pero todas tenian conflicto:
	
	source install/setup.bash
	ros2 run triciclo_py demo_movimiento  --> aarchivo python con pequeña secuencia de movimientos
	
	ros2 run teleop_twist_keyboard teleop_twist_keyboard  --> comando para abrir teleoperacion








