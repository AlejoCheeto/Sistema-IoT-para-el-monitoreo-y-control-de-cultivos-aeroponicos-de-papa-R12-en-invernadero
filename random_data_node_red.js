// Funcion para nodo de Node-red, para generación
// de datos de forma psuedo-aleatoria dados unos
// rangos especificos
 
var date = new Date()

msg.payload = {
  deviceID:"Aeronodo",
  temperatura: getRandomNumber(20, 30).toString(),
  humedad: getRandomNumber(40, 80).toString(),
  lux: getRandomNumber(5, 20).toString(),
  pH: getRandomNumber(6, 8).toString(),
  fechahora:date.toISOString()
};

function getRandomNumber(min, max) {
  return Math.floor(Math.random() * (max - min + 1)) + min;
}

return msg;