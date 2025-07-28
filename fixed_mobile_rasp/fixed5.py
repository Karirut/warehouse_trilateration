import time
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
import boto3  # Importar Boto3
import os     # Para obtener la ubicación de la RPi
import sys    # Para sys.exit() en caso de errores críticos

os.environ['AWS_ACCESS_KEY_ID'] = 'xxx'
os.environ['AWS_SECRET_ACCESS_KEY'] = 'xxx'

# --- Configuración de AWS DynamoDB ---
# Asegúrate de que estas variables de entorno o la configuración de ~/.aws/credentials estén bien
AWS_REGION = 'xxx'  # ej. 'us-east-1', 'sa-east-1'
RFID_TAGS_TABLE_NAME = 'RFID'
ALMACEN_INVENTARIO_TABLE_NAME = 'Almacen' # Nombre de la tabla de almacén

# Inicializar cliente de DynamoDB
try:
    dynamodb = boto3.resource('dynamodb', region_name=AWS_REGION)
    rfid_tags_table = dynamodb.Table(RFID_TAGS_TABLE_NAME)
    almacen_inventario_table = dynamodb.Table(ALMACEN_INVENTARIO_TABLE_NAME)
    print("Conexión a DynamoDB establecida.")
except Exception as e:
    print(f"Error al conectar con DynamoDB: {e}")
    print("Asegúrate de que tus credenciales de AWS y la región estén configuradas correctamente.")
    sys.exit(1) # Salir si no se puede conectar a DynamoDB, es un error crítico

# Inicializar lector RFID
try:
    rfid_reader = SimpleMFRC522()
    print("Lector RFID inicializado.")
except Exception as e:
    print(f"Error al inicializar lector RFID: {e}")
    print("Asegúrate de que el lector RFID esté correctamente conectado y las librerías instaladas.")
    sys.exit(1) # Salir si no se puede inicializar el lector, es un error crítico


def read_rfid():
    """Lee el tag RFID si está cerca del lector"""
    try:
        print("Esperando escaneo de tarjeta RFID...")
        tag_id, text = rfid_reader.read()
        if tag_id:
            # tag_id puede ser un entero, es mejor convertirlo a string para consistencia con DynamoDB
            print(f"Tag detectado: ID {tag_id}, Texto: {text}")
            return str(tag_id) # Solo necesitamos el ID del tag
        return None
    except Exception as e:
        print(f"Error en lectura RFID: {e}")
        return None

# --- Función: Actualizar estado en DynamoDB ---
def update_package_status(rfid_tag_id):
    if not rfid_tag_id:
        print("No se proporcionó un ID de RFID para actualizar el estado.")
        return False

    try:
        # 1. Obtener el ID_Paquete de la tabla de tags RFID usando rfid_tag_id
        # La llave primaria en RFID_Tags es 'rfid_tag_id'
        response_rfid_tags = rfid_tags_table.get_item(
            Key={
                'rfid_tag_id': rfid_tag_id
            }
        )
        rfid_item = response_rfid_tags.get('Item')

        if not rfid_item:
            print(f"Error: RFID tag '{rfid_tag_id}' no encontrado en la tabla '{RFID_TAGS_TABLE_NAME}'.")
            return False

        # El ID que necesitamos para la tabla de almacén es 'ID_Paquete'
        package_id_from_rfid_table = rfid_item.get('ID_Paquete')
        if not package_id_from_rfid_table:
            print(f"Error: El tag RFID '{rfid_tag_id}' no tiene un 'ID_Paquete' asociado en '{RFID_TAGS_TABLE_NAME}'.")
            return False

        print(f"RFID '{rfid_tag_id}' asociado al ID_Paquete: {package_id_from_rfid_table}")

        # 2. Actualizar el estado en la tabla de almacén 'Almacen_Inventario'
        # La llave primaria en Almacen_Inventario es 'device_id'
        # El atributo a actualizar es 'Estado_Ubicacion'
        almacen_inventario_table.update_item(
            Key={
                'device_id': package_id_from_rfid_table # Aquí 'ID_Paquete' de RFID_Tags es el 'device_id' en Almacen_Inventario
            },
            UpdateExpression="SET #eu = :new_estado, #ts = :timestamp_val, #loc = :location_val",
            ExpressionAttributeNames={
                '#eu': 'Estado_Ubicacion',
                '#ts': 'timestamp',        # Usando el nombre del atributo real de la tabla Almacen_Inventario
                '#loc': 'location'         # Este es un nuevo atributo que se añadirá
            },
            ExpressionAttributeValues={
                ':new_estado': 'Estanteria Ropa', # Estado que se va a asignar
                ':timestamp_val': int(time.time()), # Timestamp actualizado
                ':location_val': os.getenv('RASPBERRY_PI_LOCATION', 'RaspberryPi_Desconocida') # Ubicación de la RPi
            },
            ReturnValues="UPDATED_NEW"
        )
        print(f"Estado del paquete con device_id '{package_id_from_rfid_table}' actualizado a 'En Almacén' en la tabla '{ALMACEN_INVENTARIO_TABLE_NAME}'.")
        return True

    except Exception as e:
        print(f"Ocurrió un error al actualizar DynamoDB: {e}")
        return False

# --- Bucle principal de ejecución ---
if _name_ == "_main_":
    # Puedes establecer el nombre de esta Raspberry Pi aquí o como una variable de entorno.
    # Si tienes varias Pi's, es mejor usar variables de entorno (ej. export RASPBERRY_PI_LOCATION="RaspberryPi_Entrada")
    # Para pruebas o una única Pi, puedes descomentar la línea de abajo:
    # os.environ['RASPBERRY_PI_LOCATION'] = 'RaspberryPi_01'
    
    print("Iniciando sistema de escaneo RFID para actualización de inventario.")
    try:
        while True:
            # Leer RFID
            tag_id = read_rfid() # read_rfid ahora solo devuelve el tag_id

            # Si se detecta un RFID, actualizar DynamoDB
            if tag_id:
                print(f"RFID detectado: {tag_id}. Intentando actualizar estado en DynamoDB...")
                update_package_status(tag_id)
                # Opcional: Esperar un poco más después de una lectura exitosa
                # para evitar lecturas duplicadas rápidas del mismo tag
                time.sleep(3) # Espera 3 segundos después de una lectura exitosa
            else:
                # Si no se leyó nada, simplemente esperar un poco y reintentar
                time.sleep(1) # Espera 1 segundo si no hay lectura

    except KeyboardInterrupt:
        print("\nDeteniendo el monitor.")
    except Exception as e:
        print(f"Error inesperado en el bucle principal: {e}")
    finally:
        # Limpieza de GPIO al finalizar (crucial para evitar problemas en futuras ejecuciones)
        print("Realizando limpieza de GPIO...")
        GPIO.cleanup()
        print("Programa finalizado.")
