# Sistema-IoT-para-el-monitoreo-y-control-de-cultivos-aerop-nicos-de-papa-R12-en-invernadero
## Solución de errores con libreria de MCCI_LoRaWAN_LMIC_library
Para poder realizar una conexión, utilizando OTAA o ABP, por medio de LoRaWAN, es necesario utilizar las libreria "MCCI_LoRaWAN_LMIC_library". Sin embargo, esta misma puede generar conflictos de multiple definición de una función de nombre hal_init, que se encuentra tanto en la libreria mencionada como en los archivos de configuración de las tarjetas basadas en ESP32, que se descargar para el IDE de Arduino.

Por lo tanto, es necesario agregar una línea de código a la librería de configuración del paquete de liberías "MCCI_LoRaWAN_LMIC_library". Esta línea permite definir de forma diferente la función que genera el error y poder compilar de forma correcta el código de conexión entre la tarjeta LilyGO LoRa32 v2.1 y el Gateway utilizando OTAA.

La solución para este problema se encuentra en el siguiente hilo en GitHub: https://github.com/lnlp/LMIC-node/issues/41

## Registro de tarjetas LilyGO LoRa32 v2.1 a TTN
Con el fin de poder enlazar las tarjetas con el Gateway Multitech CONDUIT IP67, es necesario realizar el registro de estas tartejas en una plataforma de control para dispositivos tanto "end-devices" como "gateways". Para este caso se está utilizando el Sandbox de The Things Stack (The Things Network), que permite configurar región, frecuencia de trabajo, y normativa de los dispositivos que se van a registrar.

El registro de las tarjetas mencionadas se realiza un cluster NAM1 (North America) bajo autenticación OTAA, teniendo en cuenta que los chips SX1276 soportan la versión de LoRaWAN 1.0.2

OTAA o Over-the-air-activation es la forma preferida y más segura de conectar un dispositivo. Los dispositivos realizan un procedimiento de conexión a la red, durante el cual se les asigna una dirección de dispositivo dinámica y se negocian las claves de seguridad con el dispositivo.

Con respecto a las direcciones APPEUI y DevEUI que son necesarias al momento del registro de los End-devices, corresponden al identificador único extendido (EUI) o proceso EUI-64 modificado. Este proceso utiliza la dirección MAC de Ethernet de 48 bits de un cliente e introduce otros 16 bits en medio de la dirección MAC de 48 bits para crear una ID de interfaz de 64 bits.

Para poder realizar un correcto registro el AppEUI se puede asignar de forma manual, siempre y cuando, al momento de realizar la conexión desde el código cargado a las tarjetas, todas estas direcciones correspondan.

Mayor información sobre el proceso de selección de las direcciones EUI y cómo las asigna TTN, ver los siguientes links: https://www.sapalomera.cat/moodlecf/RS/1/course/module8/8.2.4.5/8.2.4.5.html, https://www.thethingsindustries.com/docs/reference/id-eui-constraints/

**Importante** Las tarjetas, para este proyecto, fueron registradas como dispositivos de Clase A.

## Glosario 
https://www.thethingsindustries.com/docs/reference/glossary/

