import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/Escritorio/ejemplos/final/Parte 3/install/triciclo_py'
