// public/map_ros2.js

document.addEventListener('DOMContentLoaded', () => {
    const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090' // ¡IMPORTANTE! Cambia esto si rosbridge está en otra IP o puerto
    });

    ros.on('connection', () => {
        console.log('Conectado a rosbridge_server.');
        document.getElementById('statusMessage').innerText = 'Conectado a ROS. Cargando mapa...';
        initializeMap();
    });

    ros.on('error', (error) => {
        console.error('Error al conectar a rosbridge_server:', error);
        document.getElementById('statusMessage').innerText = 'Error de conexión con ROS. Revisa la consola y asegúrate que rosbridge_server esté corriendo.';
    });

    ros.on('close', () => {
        console.log('Desconectado de rosbridge_server.');
        document.getElementById('statusMessage').innerText = 'Desconectado de ROS.';
    });

    let mapCanvas;
    let mapCtx;
    let mapImage = new Image();
    let robotX = 0;
    let robotY = 0;
    let robotYaw = 0;
    let pathPoints = [];
    const PATH_MAX_LENGTH = 500;

    // Configuración del mapa – ACTUALIZADO SEGÚN robocov_map_circuito.yaml
    const MAP_RESOLUTION = 0.05; // metros/píxel (coincide con YAML)
    const MAP_ORIGIN_X = -23.8; // Origen X del YAML
    const MAP_ORIGIN_Y = -31; // Origen Y del YAML
    const MAP_IMAGE_PATH = 'robocov_map_circuito.png'; // Nombre de la imagen del YAML (debe estar en la carpeta public)

    function initializeMap() {
        mapCanvas = document.getElementById('mapCanvas');
        mapCtx = mapCanvas.getContext('2d');

        mapImage.onload = () => {
            // Ajustar el tamaño del canvas al tamaño de la imagen del mapa
            mapCanvas.width = mapImage.width;
            mapCanvas.height = mapImage.height;

            drawMapAndRobot(); // Dibuja el mapa una vez cargado
            document.getElementById('statusMessage').innerText = 'Mapa cargado. Esperando datos del robot...';
        };
        mapImage.src = MAP_IMAGE_PATH; // Carga la imagen del mapa

        subscribeToRobotPose(); // Suscribirse a la posición del robot
    }

    function drawMapAndRobot() {
        if (!mapCtx) return;

        mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height); // Limpia el canvas

        // Dibuja el mapa
        if (mapImage.complete) {
            mapCtx.drawImage(mapImage, 0, 0, mapCanvas.width, mapCanvas.height);
        }

        // Convertir coordenadas del mundo ROS a coordenadas del canvas
        // (0,0) del mapa en ROS está en el origen del mapa en el mundo real.
        // El canvas tiene su (0,0) en la esquina superior izquierda.
        // Para dibujar el mapa ROS en el canvas de manera que el origen (MAP_ORIGIN_X, MAP_ORIGIN_Y)
        // se mapee correctamente y el eje Y se invierta (ROS Y arriba, Canvas Y abajo),
        // necesitamos calcular el desplazamiento.

        // Calcula el origen del mapa en píxeles relativos al canvas (bottom-left corner of ROS map in canvas coords)
        const mapPixelOriginX = -MAP_ORIGIN_X / MAP_RESOLUTION;
        const mapPixelOriginY = mapCanvas.height - (-MAP_ORIGIN_Y / MAP_RESOLUTION);


        // Dibujar el camino del robot
        mapCtx.strokeStyle = 'rgba(255, 0, 0, 0.7)'; // Rojo semitransparente
        mapCtx.lineWidth = 2;
        mapCtx.beginPath();
        if (pathPoints.length > 0) {
            // El primer punto
            const startX = (pathPoints[0].x / MAP_RESOLUTION) + mapPixelOriginX;
            const startY = mapCanvas.height - ((pathPoints[0].y / MAP_RESOLUTION) + mapPixelOriginY);
            mapCtx.moveTo(startX, startY);

            // El resto de los puntos
            pathPoints.forEach(point => {
                const canvasX = (point.x / MAP_RESOLUTION) + mapPixelOriginX;
                const canvasY = mapCanvas.height - ((point.y / MAP_RESOLUTION) + mapPixelOriginY);
                mapCtx.lineTo(canvasX, canvasY);
            });
            mapCtx.stroke();
        }


        // Dibujar el robot
        const robotCanvasX = (robotX / MAP_RESOLUTION) + mapPixelOriginX;
        const robotCanvasY = mapCanvas.height - ((robotY / MAP_RESOLUTION) + mapPixelOriginY);

        const robotRadius = 5; // Radio del robot en píxeles, ajusta según sea necesario

        mapCtx.save();
        mapCtx.translate(robotCanvasX, robotCanvasY);
        mapCtx.rotate(-robotYaw); // Ajustar según orientación de canvas. ROS yaw es CCW desde X, canvas rotate es CW.

        mapCtx.fillStyle = 'blue';
        mapCtx.beginPath();
        mapCtx.arc(0, 0, robotRadius, 0, 2 * Math.PI);
        mapCtx.fill();

        mapCtx.strokeStyle = 'white';
        mapCtx.lineWidth = 3;
        mapCtx.beginPath();
        mapCtx.moveTo(0, 0);
        mapCtx.lineTo(robotRadius, 0); // Línea hacia adelante
        mapCtx.stroke();

        mapCtx.restore();

        requestAnimationFrame(drawMapAndRobot); // Solicitar el siguiente frame para animación
    }


    function subscribeToRobotPose() {
        const odomListener = new ROSLIB.Topic({
            ros: ros,
            name: '/odom',
            messageType: 'nav_msgs/Odometry' // Asegúrate que el tipo de mensaje sea correcto para ROS2
        });

        odomListener.subscribe((message) => {
            robotX = message.pose.pose.position.x;
            robotY = message.pose.pose.position.y;

            const q = message.pose.pose.orientation;
            // Calcular yaw de un Quaternion (copiado de ROS)
            const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            robotYaw = Math.atan2(siny_cosp, cosy_cosp);

            pathPoints.push({ x: robotX, y: robotY });
            if (pathPoints.length > PATH_MAX_LENGTH) {
                pathPoints.shift(); // Eliminar el punto más antiguo
            }
        });
    }

    // Nota: la función `logout` debería estar definida globalmente o en otro script que ya esté cargado
    // para que el botón de cerrar sesión funcione correctamente.
});