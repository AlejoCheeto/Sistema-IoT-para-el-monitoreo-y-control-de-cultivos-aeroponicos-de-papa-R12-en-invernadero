[
    {
        "id": "cac3f3dcb2fae203",
        "type": "tab",
        "label": "Flow 1",
        "disabled": false,
        "info": "",
        "env": []
    },
    {
        "id": "48f46a5e.c70864",
        "type": "inject",
        "z": "cac3f3dcb2fae203",
        "name": "Arduino simulation",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "v": "lora",
                "vt": "msg"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": "",
        "topic": "",
        "payload": "\"deviceID\":\"Aeronodo\",\"temperatura\":\"16.00\",\"humedad\":\"41.00\",\"lux\":\"1047.00\",\"pH\":\"4.00\"",
        "payloadType": "str",
        "x": 170,
        "y": 240,
        "wires": [
            [
                "254297f5.99dd78"
            ]
        ]
    },
    {
        "id": "254297f5.99dd78",
        "type": "function",
        "z": "cac3f3dcb2fae203",
        "name": "Add Gateway Time UTC",
        "func": "var date = new Date();\n\nmsg.payload=\"{\"+msg.payload+\",\"+ \"\\\"fechahora\\\":\\\"\"+date.toISOString()+\"\\\"}\";\nreturn {topic:'loraNode',\n        payload:msg.payload\n        };",
        "outputs": 1,
        "timeout": "",
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 380.14990234375,
        "y": 314.79998779296875,
        "wires": [
            [
                "7bcbe7a42c23db52",
                "8a8b1d04.0c96b"
            ]
        ]
    },
    {
        "id": "5a1877dc.86b538",
        "type": "switch",
        "z": "cac3f3dcb2fae203",
        "name": "",
        "property": "payload",
        "propertyType": "msg",
        "rules": [
            {
                "t": "cont",
                "v": "ID",
                "vt": "str"
            },
            {
                "t": "cont",
                "v": "Aeronodo",
                "vt": "str"
            }
        ],
        "checkall": "true",
        "repair": false,
        "outputs": 2,
        "x": 626.4999923706055,
        "y": 370.2501093149185,
        "wires": [
            [
                "5ce32e36.f85c"
            ],
            [
                "e0f0e7cc.3689a8"
            ]
        ]
    },
    {
        "id": "5ce32e36.f85c",
        "type": "debug",
        "z": "cac3f3dcb2fae203",
        "name": "Cultivos",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "payload",
        "targetType": "msg",
        "statusVal": "",
        "statusType": "auto",
        "x": 838.5000267028809,
        "y": 492.8500270843506,
        "wires": []
    },
    {
        "id": "e0f0e7cc.3689a8",
        "type": "debug",
        "z": "cac3f3dcb2fae203",
        "name": "Aeroponia",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "payload",
        "targetType": "msg",
        "statusVal": "",
        "statusType": "auto",
        "x": 824.2000370025635,
        "y": 589.8500289916992,
        "wires": []
    },
    {
        "id": "711c4ced.152bb4",
        "type": "mqtt out",
        "z": "cac3f3dcb2fae203",
        "name": "IoTHUB CEA",
        "topic": "devices/GATEWAYLORATIBAITATA/messages/events/",
        "qos": "1",
        "retain": "false",
        "broker": "583e984b.ed2f58",
        "x": 899,
        "y": 714.6500244140625,
        "wires": []
    },
    {
        "id": "8a8b1d04.0c96b",
        "type": "mqtt out",
        "z": "cac3f3dcb2fae203",
        "name": "LoraINE",
        "topic": "devices/ALEJOESTEBANPC/messages/events/",
        "qos": "1",
        "retain": "false",
        "respTopic": "",
        "contentType": "",
        "userProps": "",
        "correl": "",
        "expiry": "",
        "broker": "583e984b.ed2f58",
        "x": 740,
        "y": 280,
        "wires": []
    },
    {
        "id": "c9aef799e8bad4a6",
        "type": "function",
        "z": "cac3f3dcb2fae203",
        "name": "Variable data + Timestamp",
        "func": "var date = new Date()\n\nmsg.payload = {\n  deviceID:\"Aeronodo\",\n  temperatura: getRandomNumber(20, 30).toString(),\n  humedad: getRandomNumber(40, 80).toString(),\n  lux: getRandomNumber(5, 20).toString(),\n  pH: getRandomNumber(6, 8).toString(),\n  fechahora:date.toISOString()\n};\n\nfunction getRandomNumber(min, max) {\n  return Math.floor(Math.random() * (max - min + 1)) + min;\n}\n\nreturn msg;",
        "outputs": 1,
        "timeout": 0,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 360,
        "y": 500,
        "wires": [
            [
                "5a1b29be9cb39b8f",
                "5a1877dc.86b538"
            ]
        ]
    },
    {
        "id": "c506156cd32adcf1",
        "type": "inject",
        "z": "cac3f3dcb2fae203",
        "name": "inject",
        "props": [
            {
                "p": "topic",
                "vt": "str"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "lora",
        "x": 130,
        "y": 500,
        "wires": [
            [
                "c9aef799e8bad4a6"
            ]
        ]
    },
    {
        "id": "5a1b29be9cb39b8f",
        "type": "debug",
        "z": "cac3f3dcb2fae203",
        "name": "debug 1",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "false",
        "statusVal": "",
        "statusType": "auto",
        "x": 620,
        "y": 600,
        "wires": []
    },
    {
        "id": "047952ca98c3a9c3",
        "type": "inject",
        "z": "cac3f3dcb2fae203",
        "name": "Lora Simulated",
        "props": [
            {
                "p": "payload"
            },
            {
                "p": "topic",
                "v": "lora",
                "vt": "msg"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": "",
        "topic": "",
        "payload": "\"deviceID\":\"Aeronodo\",\"temperatura\":25.87,\"humedad\":60.16, \"lux\":11.1,\"pH\":14",
        "payloadType": "str",
        "x": 160,
        "y": 180,
        "wires": [
            []
        ]
    },
    {
        "id": "7bcbe7a42c23db52",
        "type": "debug",
        "z": "cac3f3dcb2fae203",
        "name": "debug 2",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "false",
        "statusVal": "",
        "statusType": "auto",
        "x": 560,
        "y": 220,
        "wires": []
    },
    {
        "id": "583e984b.ed2f58",
        "type": "mqtt-broker",
        "name": "",
        "broker": "ingenieriaiothub2.azure-devices.net",
        "port": "8883",
        "tls": "",
        "clientid": "ALEJOESTEBANPC",
        "autoConnect": true,
        "usetls": true,
        "compatmode": false,
        "protocolVersion": "4",
        "keepalive": "60",
        "cleansession": true,
        "autoUnsubscribe": true,
        "birthTopic": "",
        "birthQos": "1",
        "birthRetain": "false",
        "birthPayload": "",
        "birthMsg": {},
        "closeTopic": "",
        "closePayload": "",
        "closeMsg": {},
        "willTopic": "",
        "willQos": "0",
        "willPayload": "",
        "willMsg": {},
        "userProps": "",
        "sessionExpiry": ""
    }
]