# Parte 5: Agregar camara en Gazebo 

-Para poder visualizar el robot en Gazebo, sera necesario abrir mas terminales que en Rviz, se debe seguir la siguiente rutina:

	1. Se abre el terminal 1, el cual servira para cargar el robot, en donde se deben ejecutar estos dos comandos seguidos:
	
	colcon build --packages-select triciclo_py
	source install/setup.bash
	
	2. Una vez cargados, se abrira el terminal 2, el cual sirve para cargar gazebo, para ello se ejecuta el siguiente comando, Esto abrira la interfaz de Gazebo pero sin el robot.:
	
	gz sim -v 4 ground_plane.sdf
	
	3. Se vuelve a la terminal 1 y se ejecuta el siguiente comando, el cual servira para poder visualizar el robot en Gazebo, en caso de que no cargue, apretar "ctrl+c" y ejecutar nuevamente el comando hasta que cargue el robot:
	
	ros2 launch triciclo_py gazebo.launch.py
	
	4. Una vez abierto se puede ejecutar Rviz, en donde se debe agregar un display tipo image, con el siguiente comando se abre Rviz:
	
	ros2 run rviz2 rviz2

- Se logro incluir la camara en gazebo, en la lista de links aparece, pero no se puede comunicar con Rviz, esto puede deverse a los problemas con los plugin de camara al igual que en la parte 4 con los de movimiento, los cuales no nos fue posible instalar. 
	
	








