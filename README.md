# MAVLink v2 Telemetry Recorder (C++)

This project is a small C++ program that reads MAVLink v2 telemetry data, decodes selected messages, and logs them into a CSV file.  
It is built as a learning exercise while exploring UAV telemetry, embedded-style programming, and protocol parsing. The idea is to understand how drones send binary data and how to parse it safely, rather than relying on pre-made libraries.

The parser works byte by byte, checks for packet integrity using CRC-16/X.25, and records only the messages we care about. It is lightweight, deterministic, and avoids dynamic memory, so it behaves like embedded code.

---

## Why This Project

Drones and autonomous vehicles continuously send telemetry as a **stream of bytes**.  
Unlike text or JSON, there are **no natural boundaries between messages**, so if even one byte is lost, the whole stream can go out of sync.  

A proper parser must:

- Detect where each MAVLink packet starts  
- Recover if there is corrupted or missing data  
- Verify the packet with CRC  
- Decode only the payload fields we care about  

This project demonstrates all of this with **MAVLink v2**, without using the official MAVLink parsing utilities. Everything is implemented manually.

---

## What Makes This Special

- Parses **one byte at a time** (FSM-based)  
- No `new` or dynamic memory — everything is fixed buffers  
- Uses **C++17** standards  
- Clear separation between parser and application logic  
- Easy to extend with more MAVLink messages  

---

## High-Level Flow

```
+------------------+
|   Byte Stream    |
|  (Drone / SITL)  |
+--------+---------+
		|
		v
+-------------------------+
|   MAVLink Parser 	      |
|    (FSM-based)  	      |
|  				 		  |
|  - Detects magic byte   |
|  - Reads header  		  |
|  - Reads payload 		  |
|  - Validates CRC  	  |
+--------+----------------+
		|
		v
+------------------+
|  Message Decoder |
|				   |
|  - HEARTBEAT     |
|  - ATTITUDE      |
|  - GLOBAL_POS    |
+--------+---------+
		|
		v
+------------------+
|   CSV Logger     |
|  flight_log.csv  |
+------------------+
```

---

## MAVLink v2 Packet Layout

| Byte(s) | Field              |
|---------|------------------|
| 0       | Magic (0xFD)      |
| 1       | Payload Length    |
| 2-3     | Incompat/Compat Flags |
| 4       | Sequence Number   |
| 5       | System ID         |
| 6       | Component ID      |
| 7-9     | Message ID        |
| N       | Payload           |
| N+1,N+2 | CRC-16/X.25       |

Packets come **back-to-back** without delimiters. Parser uses the magic byte and FSM to stay in sync.

---

## Parser FSM
```
The parser runs a **finite state machine**:
+-----------+
| HUNTING   | ← waits for magic byte (0xFD)
+-----+-----+
		|
		v
+-----------+
| HEADER    | ← reads fixed header bytes
+-----+-----+
		|
		v
+-----------+
| PAYLOAD   | ← reads N bytes based on payload length
+-----+-----+
		|
		v
+-----------+
| CRC       | ← verifies CRC-16/X.25
+-----+-----+
		|
		v
+-----------+
| DECODE    | ← if CRC valid, extract fields
| or DROP   | ← else return to HUNTING
+-----------+
```
If at any stage the data is invalid, it goes back to **HUNTING** to find the next packet.

---

## Supported Messages

Currently, the parser decodes:

- `HEARTBEAT` — Drone system info  
- `ATTITUDE` — Roll, pitch, yaw  
- `GLOBAL_POSITION_INT` — GPS coordinates and altitude  

Adding more messages is straightforward:

1. Add CRC extra value in parser  
2. Extend `decodeMessage()` with the new message ID  
3. Log the decoded fields to CSV  

---

## Project Structure


```
mavlink-logger/
├── include/
│ ├── mavlinkparser.hpp # Parser interface
│ └── crc_x25.hpp # CRC function
├── src/
│ ├── main.cpp # Test harness / simulator
│ ├── mavlinkparser.cpp # Parser implementation
│ └── crc_x25.cpp # CRC-16/X.25 function
├── libs/
│ └── mavlink_cLib_v2/ # MAVLink headers (LGPL)
├── build/ # Out-of-source build
├── CMakeLists.txt
├── LICENSE # MIT License
├── README.md
└── THIRD_PARTY_NOTICES.md
```
---

## Build Instructions

### Requirements

- C++17 compiler (GCC / Clang / MSVC)  
- CMake ≥ 3.16  

### Steps

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

All warnings are treated as errors to keep code clean.

---

## Run the Program

```./mavlink_telemetry_recorder```

This produces the output file:

```flight_log.csv```

---

## Output Format

Example output:

```
TYPE,A,B,C
HEARTBEAT,2,1,4
ATTITUDE,5.72958,11.4592,17.1887
```

- `TYPE` identifies the MAVLink message
- `A`, `B`, `C` represent decoded fields
- Units are converted where appropriate (e.g., radians to degrees)

---

## Notes

- MAVLink headers are only used as message definitions, not parsing utilities
- Everything assumes little-endian
- Works offline; suitable for post-flight analysis
- No external dependencies besides standard C++

---

## Licensing

- Project source code is licensed under the MIT License.
- MAVLink headers: LGPL, included unmodified

## Disclaimer

This is an educational project, written independently.  
No official MAVLink utilities are used. MAVLink is just a reference for message structures.