// public/script.js
document.addEventListener('DOMContentLoaded', () => {
    const loadDataButton = document.getElementById('loadDataButton');
    const dataContainer = document.getElementById('dataContainer');

    loadDataButton.addEventListener('click', async () => {
        dataContainer.innerHTML = '<p>Cargando datos...</p>'; // Mensaje de carga

        try {
            // Realiza una solicitud al endpoint de tu servidor Node.js
            const response = await fetch('/api/items');
            
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const items = await response.json(); // Parsea la respuesta JSON

            if (items.length > 0) {
                // Limpia el contenedor y añade los datos
                dataContainer.innerHTML = '<h2>Elementos de DynamoDB:</h2>';
                items.forEach(item => {
                    const itemCard = document.createElement('div');
                    itemCard.classList.add('item-card');
                    
                    // --- AQUÍ ES DONDE PERSONALIZAS LA VISUALIZACIÓN ---
                    // Asegúrate de que las propiedades coincidan con los nombres de tus atributos de DynamoDB.
                    itemCard.innerHTML = `
                        <h3>Device ID: ${item.device_id || 'N/A'}</h3>
                        <p><strong>Categoría:</strong> ${item.Categoria || 'N/A'}</p>
                        <p><strong>Estado de Ubicación:</strong> ${item.Estado_Ubicacion || 'N/A'}</p>
                        <p><strong>ID de Paquete:</strong> ${item.ID_Paquete || 'N/A'}</p>
                        <p><strong>Tipo de Producto:</strong> ${item.Tipo_Producto || 'N/A'}</p>
                    `;
                    // --- FIN DE LA PERSONALIZACIÓN ---

                    dataContainer.appendChild(itemCard);
                });
            } else {
                dataContainer.innerHTML = '<p>No se encontraron elementos en la tabla.</p>';
            }

        } catch (e) {
            console.error('Error al cargar los datos:', e);
            dataContainer.innerHTML = `<p>Error al cargar los datos: ${error.message}. Verifica la consola del servidor y del navegador.</p>`;
        }
    });
});
