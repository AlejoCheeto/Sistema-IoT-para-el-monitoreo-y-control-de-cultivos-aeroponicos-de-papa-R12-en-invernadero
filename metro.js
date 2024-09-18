var { EventHubClient, EventPosition } = require('@azure/event-hubs');

var connectionString = 'HostName=ingenieriaiothub2.azure-devices.net;SharedAccessKeyName=IntegracionProcesosTI;SharedAccessKey=UGKsyJFl95dVEKWAsdUERQ1bUzrvDgd5QAIoTMgALDg='

//funcion de error
var printError = function (err) {
    console.log(err.message);
  };  

var sql = require("mssql");
var config = {
    user: 'aeroponia',
    password: 'Aeroponicos2024',
    server: 'sqlceaiotserver.database.windows.net',
    database: 'SQLCEAIOTDB',
    encrypt: true
  }

  //send query to sql database
var writesqldb =  function(query) {
    sql.connect(config, function (err) {
        if (err) console.log(err);
        // create Request object
        var request = new sql.Request();
        // query to the database and get the records
        request.query(query, function (err, recordset) {
  
            if (err) console.log(err)
              // send records as a response
            console.log(recordset);
        });
    });
  }

// funcion para conectarse al IoT HUB
var ehClient;
EventHubClient.createFromIotHubConnectionString(connectionString).then(function (client) {
  console.log("Successfully created the EventHub Client from iothub connection string.");
  ehClient = client;
  return ehClient.getPartitionIds();
}).then(function (ids) {
  console.log("The partition ids are: ", ids);
  return ids.map(function (id) {
    return ehClient.receive(id, todowithdatarcvfromiothub, printError, { eventPosition: EventPosition.fromEnqueuedTime(Date.now()) });
  });
}).catch(printError);

var todowithdatarcvfromiothub = function (message) {
    //console.log(message)
    deviceid=JSON.stringify(message._raw_amqp_mesage.message_annotations['iothub-connection-device-id']); //del IoTHub
    console.log(deviceid)
    console.log(message.body)
    humedad = message.body.humedad
    temperatura = message.body.temperatura
    lux = message.body.lux
    pH = message.body.pH
    //deviceID = message.body.deviceID
	deviceID = message.body.device
	
	if(temperatura != '888888')
	{
		var datetime = new Date();
		console.log(datetime)
		fechahora= datetime.toISOString().substr(0,10)+" "+datetime.toISOString().substr(11,8);
		console.log(fechahora)
		query = `INSERT INTO dbo.AEROPONIA_telemetria values('${fechahora}','${temperatura}','${humedad}','${lux}','${pH}','${deviceID}')`
		console.log(query)
		writesqldb(query)
	}
   
}
