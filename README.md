# Valuepipe

## The Priciple
Valuepipe consist of three signals: `data`, `clk` and `latch`, exactly like a shift register

### Initialization
In normal operation `clk` and `latch` appears never at the same time. This is used during the initialisation: When the controller receives a init command, it sets first `clk` and then `latch` on high. This indicates to all clients to clear their shift registers.

Then the controller send  High bits on `data out`, until they appear back on its `data in`. By counting the ticks, the controller knows the byte field size of the connected clients.