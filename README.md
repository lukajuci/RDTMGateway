## Overview

The RDTM Gateway Node collects incoming data from all the Data Nodes and sends them to the end user's device through a MQTT server.

This repository contains the device firmware for the RDTM gateway node using the Meshtastic project as a base.

Changes include writing and implementing the [DataNodeModule](https://github.com/lukajuci/RDTMGateway/blob/main/src/modules/DataNodeModule.cpp) as well as the [rdtmtelemetry](https://github.com/lukajuci/RDTMGateway/blob/main/protobufs/meshtastic/rdtmtelemetry.proto) protobuf for packaging the desired sensor data.

- **[Meshtastic Project Main Github](https://github.com/meshtastic)**


