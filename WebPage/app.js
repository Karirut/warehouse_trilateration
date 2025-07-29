// app.js (versión completa y actualizada con ES Modules y todos los filtros)
import express from 'express';
import AWS from 'aws-sdk'; // Asegúrate de que es 'aws-sdk'
import path from 'path';
import session from 'express-session';
//const io = require('socket.io')(http);
import { fileURLToPath } from 'url'; // Para __filename
import { dirname } from 'path';     // Para __dirname
import http from 'http'; // Importar módulo HTTP para crear el servidor
import { Server as SocketIOServer } from 'socket.io'; // Importar Server de socket.io

// Obtener __dirname y __filename para ES Modules
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);


const app = express();
const port = 3000;

// Crear el servidor HTTP a partir de la aplicación Express
const server = http.createServer(app);
// Inicializar Socket.IO con el servidor HTTP
const io = new SocketIOServer(server);


// --- Middlewares de Express ---
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Configuración de express-session
app.use(session({
    secret: '', // ¡CAMBIA ESTO por una cadena de texto larga y aleatoria!
    resave: false,
    saveUninitialized: false,
    cookie: { maxAge: 60 * 60 * 1000 } // 1 hora de duración de la sesión (en milisegundos)
}));

// --- Configuración de las credenciales y la región de AWS ---
AWS.config.update({
    accessKeyId: '',
    secretAccessKey: '',
    region: ''
});

const dynamodb = new AWS.DynamoDB.DocumentClient();
const tableName = 'Almacen';
const users = [
    { username: 'user', password: '54321' },
    { username: 'admin', password: '12345' }
];

// --- Middleware para verificar autenticación (protege rutas) ---
function isAuthenticated(req, res, next) {
    if (req.session.isAuthenticated) {
        next();
    } else {
        res.redirect('/'); // No autenticado, redirigir a la página de login
    }
}

// --- Servir archivos estáticos desde la carpeta 'Public' ---
// Asegurarse de que el directorio 'Public' contiene 'map.html' y 'socket.io.js' (servido por Socket.IO)
app.use(express.static(path.join(__dirname, 'Public')));

// --- Rutas de Autenticación ---

// Ruta para la página de login
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'Public', 'login.html'));
});

// Ruta para manejar el POST de login
app.post('/login', (req, res) => {
    const { username, password } = req.body;
    const user = users.find(u => u.username === username && u.password === password);

    if (user) {
        req.session.isAuthenticated = true;
        res.json({ success: true, message: 'Login exitoso' });
    } else {
        req.session.isAuthenticated = false;
        res.status(401).json({ success: false, message: 'Usuario o contraseña incorrectos.' });
    }
});

// Ruta para cerrar sesión
app.post('/logout', (req, res) => {
    req.session.destroy(err => {
        if (err) {
            return res.status(500).json({ message: 'Error al cerrar sesión' });
        }
        res.json({ message: 'Sesión cerrada' });
    });
});

// --- Rutas Protegidas ---

// Ruta para el Panel Principal (dashboard)
app.get('/dashboard', isAuthenticated, (req, res) => {
    res.sendFile(path.join(__dirname, 'Public', 'dashboard.html'));
});

// Ruta para la página de la Tabla de DynamoDB
app.get('/table', isAuthenticated, (req, res) => {
    res.sendFile(path.join(__dirname, 'Public', 'tabla.html'));
});

// Ruta para la página del Mapa (ahora incluye la funcionalidad de Raspberry Pi)
app.get('/map', isAuthenticated, (req, res) => {
    res.sendFile(path.join(__dirname, 'Public', 'map.html'));
});

// Endpoint API para obtener los datos de DynamoDB (con múltiples filtros y paginación)
app.get('/api/items', isAuthenticated, async (req, res) => {
    // Obtener y limpiar todos los parámetros de filtro de la URL usando .trim()
    const categoryFilter = req.query.category ? String(req.query.category).trim() : '';
    const productTypeFilter = req.query.productType ? String(req.query.productType).trim() : '';
    const packageIdFilter = req.query.packageId ? String(req.query.packageId).trim() : '';
    const locationStatusFilter = req.query.locationStatus ? String(req.query.locationStatus).trim() : '';

    console.log("Backend: locationStatusFilter recibido y limpiado:", locationStatusFilter);
    console.log("Backend: Tipo de dato de locationStatusFilter:", typeof locationStatusFilter);

    const limit = parseInt(req.query.limit) || 10;
    let exclusiveStartKey = req.query.exclusiveStartKey;

    let params = {
        TableName: tableName,
        Limit: limit
    };

    if (exclusiveStartKey) {
        try {
            params.ExclusiveStartKey = JSON.parse(exclusiveStartKey);
        } catch (e) {
            console.error("Formato de llave de acceso invalida", e);
            return res.status(400).json({ error: 'exclusiveStartKey invalido (formato incorrecto)' });
        }
    }

    // Construir dinámicamente el FilterExpression y ExpressionAttributeValues
    let filterExpressions = [];
    let expressionAttributeValues = {};

    // Añadir filtros si existen y no están vacíos
    if (categoryFilter !== '') {
        filterExpressions.push("Categoria = :cat");
        expressionAttributeValues[":cat"] = categoryFilter;
    }
    // Usar contains() para Tipo_Producto
    if (productTypeFilter !== '') {
        filterExpressions.push("contains(Tipo_Producto, :prodType)");
        expressionAttributeValues[":prodType"] = productTypeFilter;
    }
    // Usar begins_with() para ID_Paquete
    if (packageIdFilter !== '') {
        filterExpressions.push("begins_with(ID_Paquete, :pkgId)");
        expressionAttributeValues[":pkgId"] = packageIdFilter;
    }
    if (locationStatusFilter !== '') {
        filterExpressions.push("Estado_Ubicacion = :locStatus");
        expressionAttributeValues[":locStatus"] = locationStatusFilter;
        console.log("Backend: Añadiendo filtro de Estado_Ubicacion con valor:", expressionAttributeValues[":locStatus"]);
    }

    // Si hay alguna expresión de filtro, unirlas con " AND "
    if (filterExpressions.length > 0) {
        params.FilterExpression = filterExpressions.join(" AND ");
        params.ExpressionAttributeValues = expressionAttributeValues;
    }

    // --- CONSOLE.LOGS CRÍTICOS PARA DEPURACIÓN ---
    console.log("Backend: Parámetros de scan ENVIADOS A DYNAMODB:", JSON.stringify(params, null, 2));
    try {
        const data = await dynamodb.scan(params).promise();
        console.log("Backend: RESPUESTA COMPLETA RECIBIDA DE DYNAMODB:", JSON.stringify(data, null, 2)); // <-- ESTE ES CLAVE

        res.json({
            items: data.Items,
            lastEvaluatedKey: data.LastEvaluatedKey || null
        });

    } catch (e) {
        console.error("Error al escanear la tabla:", JSON.stringify(e, null, 2));
        res.status(500).json({ error: 'No se pudieron recuperar los datos de AWS por DynamoDB' });
    }
});

// --- Funcionalidad de Socket.IO para Raspberry Pi ---
io.on('connection', (socket) => {
    console.log('Cliente WebSocket conectado');
});

// Ruta POST para actualizar la posición de la Raspberry Pi
app.post('/robot', (req, res) => {
    let { x, y } = req.body; // Coordenadas del robot principal (ya en SMR/mapa ROS)

    // Aquí no se aplica transformación porque se asume que ya vienen en el sistema del mapa ROS
    // Si tus datos del robot principal NO vienen en el sistema del mapa ROS, también necesitarías transformar aquí.

    console.log(`Posición del ROBOT (desde ROS2): X=${x.toFixed(2)}, Y=${y.toFixed(2)}`);

    // Emitir un evento Socket.IO DEDICADO para la posición del robot principal
    io.emit('robot_main_position_update', { x, y });
    res.send('OK');
});

// === RUTA PARA LA POSICIÓN DE LA RASPBERRY PI (proveniente de Multitrilateración) ===
// Esta es la ruta existente donde la Raspberry Pi envía sus coordenadas de trilateración.
app.post('/map', (req, res) => { // Renombré la ruta para claridad
    let { x, y } = req.body; // Coordenadas del sistema de multitrilateración (SMT)

    // --- APLICACIÓN DE LA TRANSFORMACIÓN SMT -> SMR ---
    const offset_x_smt_to_smr = 9.7; // Offset X calculado
    const offset_y_smt_to_smr = 0.7; // Offset Y calculado

    const x_transformed = x + offset_x_smt_to_smr;
    const y_transformed = y + offset_y_smt_to_smr;
    // --- FIN DE LA TRANSFORMACIÓN ---

    console.log(`Posición original RASPBERRY PI (SMT): X=${x.toFixed(2)}, Y=${y.toFixed(2)}`);
    console.log(`Posición transformada RASPBERRY PI (SMR): X=${x_transformed.toFixed(2)}, Y=${y_transformed.toFixed(2)}`);

    // Emitir un evento Socket.IO DEDICADO para la posición de la Raspberry Pi
    io.emit('raspberry_pi_position_update', { x: x_transformed, y: y_transformed });
    res.send('OK');
});

// Iniciar el servidor HTTP (ahora con soporte para Socket.IO)
server.listen(port, () => {
    console.log(`Servidor escuchando en http://localhost:${port}`);
});