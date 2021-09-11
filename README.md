# Valuepipe

Valuepipe is the poor mens solution to concat the multi-IOs of two or more SoCs to one big multi- multi- IO.

These IOs are handled as one array of byte values, where valuepipe allows to read and write these bytes either through the serial connection or programmely by one of the included SoCs.

To not be depend on additional hardware, valuepipe uses the SoC GPIOs to generate a serial protocol, where several SoCs can be chained behind

## The Priciple
Valuepipe consist of three signals: `data`, `clk` and `latch`, exactly like a shift register. Each SoC has these as input signals and also as outputs. The outputs of the last SoC are feed back to the first SoC inputs.

At start, all SoCs are equal, but as soon one of them receives an `init` command on its serial input, it becomes the controller, the other ones are the clients

### Initialization
In normal operation `clk` and `latch` appears never at the same time. This is used during the initialisation: When the controller receives a `init` command, it sets first `clk` and then `latch` on high. This indicates to all clients to clear their shift registers.

Then the controller send  High bits on `data out`, until they appear back on its `data in`. By counting the ticks, the controller knows the byte field size of the connected clients.

### Output values
The values are sent to the serial input of the controller as decimal unsigned 8 bit values, seperated by a blank, finished with \<lf>

After receiving, the controller keeps the first bytes for its own, depending on its internal byte array size, and forwards the remaining bytes to its connected client. That client does the same, until all bytes have been chained through.

### Input Values

to be programmed & descripted