# Ideas for USB Christmas Lights' Controller

I have 4 sets of USB fairy lights. Rated at 5 V and 1 A.

## Proposed components

Item        | Mfr        | PN            | Supplier | Cost   | Notes
------------|------------|---------------|----------|-------:|---
Box         |            |               |          |        | metal/plastic. Not sure what is safest.
Mains In    |            |               |          |        | Power inlet/fuse
PSU         | Tracopower | TXL 035-0512D | RS Comp  | £59-14 | 35 W, 5/12 VDC (for lights/arduino). Check currents.
Controller  | Arduino    | ABX00021      | Amazon   | £37-15 | ARDUINO UNO WiFi REV2. WiFi for clock function.
Switching   | Arduino    | A000110       | Amazon   | £32-99 | ARDUINO 4 RELAYS SHIELD - seems good enough for lights. Check currents
Connect out | Startech   | USBPLATE4     | Amazon   |  £5-65 | 4 Port USB A Female Slot Plate Adapter.

Basically to power lights on and off at set times.

But could add subtle clock functionality.
1. Turn off at the hour and then blink on the hour.
1. Blink off 1, 2 or 3 times at 15, 30 or 45 past the hour.
1. Go crazy at 4pm for tea - blink out the tea for two tune rhythm?
1. Others?

Probably not sophis enough for PWM through the relays! :(

Include indicator LEDs in box so lights don't have to be connected for testing?

Power LED lihgt from traco visible?

Other permanently on UBS charger socket?

a