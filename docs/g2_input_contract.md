# MQTT Payload Contract

To ensure the G2 Ingestion Service can process the Arduino MQTT payload must follow this strict JSON format. 

**MQTT Topic:** `ontime/telemetry/raw`

**JSON Schema Expected:**
```json
{
  "bus_id": "ABC-1234",      // String: Unique identifier for the bus
  "lat": 6.796877,           // Float: GPS Latitude
  "lng": 79.901778,          // Float: GPS Longitude
  "speed_kmh": 45.2,         // Float: Calculated speed in km/h from Arduino node
  "heading": 120.5,          // Float: Direction in degrees (0-360)
  "timestamp": 1713600000    // Integer: Unix epoch timestamp natively from GPS module
}
