# Parte 2: Modularización con Xacro.

En esta parte del proyecto se refactorizó el archivo "triciclo.urdf" para transformarlo en un archivo "triciclo.xacro", utilizando el formato Xacro con el objetivo de modularizar y simplificar la descripción del robot.

El nuevo archivo "triciclo.xacro" contiene exactamente el mismo modelo del robot triciclo definido en la Parte 1, pero ahora está estructurado de forma más limpia y reutilizable.

Se implementaron las siguientes mejoras:

- Se crearon parámetros "xacro:property" para definir dimensiones y colores que se usan varias veces en el modelo, lo cual facilita futuras modificaciones.
- Se definió una macro "xacro:macro" para generar ruedas, ya que todas tienen la misma estructura geométrica con solo pequeñas variaciones (color y radio).
- Se mantuvieron los mismos enlaces "links" y articulaciones "joints" del modelo original, respetando los tres grados de libertad del robot.

