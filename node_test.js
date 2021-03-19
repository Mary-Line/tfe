//chargement des modules
const express = require('express');
const socketIO = require("socket.io");
const port = process.env.PORT || 3000;
const index = "/index.html";
//paramètrage serveur
const server = 
express()
.use((req, res) =>{
    res.sendFile(index,{ root: __dirname})
})
.listen(port, () => {
    console.log(`Server started on port: ${port}`);
});
//On setup socketIo, sur base du serveur
const io = socketIO(server);

io.on("connection", (socket) => {
    
	const relay_box1_I2C_address = 33;  // "0x21" conversion hexa en entier;
	const relay_box2_I2C_address = 49;  //"0x31" conversion hexa en entier;
	const sht35_sensor_I2C_address = "0x45";
	const water_sensor_I2C_address_1 = "0x77";
	const water_sensor_I2C_address_2 = "0x78";
	const channel_temp = 1
	const channel_hum = 2

	var breeding_temp = 30.00; //importer données depuis la DB
	var breeding_hum = 45.00; //importer données depuis la DB

	var serialport = require('serialport');

	var {tryParse} = require('./module.js'); 

	var portName="COM3";

	var comPort = new serialport(portName, 9600);

	var Readline = serialport.parsers.Readline;

	var parser = new Readline(); 

	comPort.pipe(parser); 
	comPort.on('open', showPortOpen);
	parser.on('data', readSerialData);

	function showPortOpen(){
		console.log("Port opened");
	}

	function readSerialData(data){
		var humidity;
		var temperature;
		var level;
		
		const sensorData = tryParse(data);
		humidity = sensorData.humidity;
		temperature = sensorData.temperature;
		level = sensorData.level;
		console.log(data);
		console.log(sensorData.humidity);
		console.log(sensorData.temperature); 
		console.log(sensorData.level);
		openrelay(humidity,temperature);
		makelevelalert(level);

		io.emit("hum", sensorData.humidity);
		io.emit("temp", sensorData.temperature);
		io.emit("waterlevel", sensorData.level);
	}

	function writettoserial(i2caddress,channelporttemp,channelporttempstate,channelporthum,channelporthumstate){
		strtoserial = i2caddress + ";" + channelporttemp + ";" + channelporttempstate + ";" + channelporthum + ";" + channelporthumstate + '\n';
		comPort.open(function(err) 
		{
			comPort.write(strtoserial,function(err, res) 
			{
				if (err) 
				{
					console.log(err); 
				}
			});
		});
	}


	function openrelay(hum, temp){
		if(hum < breeding_hum && temp < breeding_temp)
		{
			console.log("Hum and temp to low");
			console.log("open relay brum and heater");
			writettoserial(relay_box1_I2C_address,channel_temp,1, channel_hum, 1);
		}
		else if(hum >= breeding_hum && temp >= breeding_temp)
		{
			console.log(" Hum and temp ok");
			console.log("nothing to do");
			writettoserial(relay_box1_I2C_address,channel_temp,0, channel_hum, 0);
		}
		else if(hum < breeding_hum && temp >= breeding_temp)
		{
			console.log(" Hum to low and temp ok");
			console.log("Just open relay for the brum");	
			writettoserial(relay_box1_I2C_address,channel_temp,0, channel_hum, 1);
		}
		else if(hum >= breeding_hum && temp < breeding_temp)
		{
			console.log(" Hum ok and temp to low");
			console.log("Just open relay for the heater");
			writettoserial(relay_box1_I2C_address,channel_temp,1, channel_hum, 0);
		}
	}

	function makelevelalert(level){
		if (level < 30)
		{
			console.log("Attention le niveau d'eau est trop faible");
		}
	}
})