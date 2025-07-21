# Parte 3: Carga y visualización del robot en Rviz con archivo URDF.

Para ejecutar el robot en Rviz, se puede hacer mediante dos modos.

- Primer modo:

Dentro de la carpeta "Parte 3" se debe abrir un terminal y ejecutar el archivo "launchrviz.sh" con el siguiente comando: ./launchrviz.sh
Este permitira ejecutar la secuencia de comandos necesarios para ejecutar la carga del robot en Rviz automaticamente.

- Segundo modo:

En caso de que el archivo .sh no funcione, se puede hacer de forma manual, para eso, dentro de la carpeta "Parte 3" se abrira un terminal en donde se seguira la siguiente secuencia de comandos para poder cargar y vizualizar el robot en Rviz, son 3 comandos en total.

	1. colcon build  --> Este comando compila el paquete creado 
	2. source install/setup.bash  --> Este comando carga los cambios realizados 
	3. ros2 launch triciclo_py display.launch.py  --> este comando ejecuta el archivo launcher del proyecto el cual hace posible visualizar el robot en "Rviz", tambien "JointStatePublisher" para probar los movimientos de grados de libertad y "robot_state_publisher".
	
- Una vez ejecutado alguna de estas secuencias de comandos, es posbile ver el robot en Rviz, en caso de que el robot no se muestre o no cargue, hay que hacer lo siguiente. Dentro de Rviz, se vera la grilla del plano y se debera cambiar lo siguiente:

	1. En "Global Options" se seleccionara "base_link" el cual es el chasis del robot.
	2. En el boton "add" del lado inferior derecho, se pueden agregar robots al plano, dentro de el se seleccionara la opción "RobotModel".
	3. Ahora aparecera la opción de "RobotModel" en la lista de Display, con la flecha que esta a la derecha del "Robotmodel" se puede desplegar sus opciónes.
	4. La opción que nos intereza es la "Description Topic", al seleccionarla se desplegara una lista y debemos seleccionar la que nos importa, en este caso "robot_descrption"
	5. Una vez seleccionada, el robot sera visible en el plano y junto a "JointStatePublisher" es posible mover las ruedas traseras, la horquilla y la rueda delantera.
