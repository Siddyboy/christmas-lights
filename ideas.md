# Ideas for USB Christmas Lights' Controller

I have 4 sets of USB fairy lights. Rated at 5 V and 1 A.

## Proposed components

Item            | Mfr        | PN             | Supplier | Cost   | Notes
----------------|------------|----------------|----------|--------|---
Box             |            |                |          |        | metal/plastic. Not sure what is safest. Wifi signal?
Mains In        | Schurter   | DD11.0114.1111 | RS Comp  |   <£10 | Schurter DD11 Series - 2 pole, single fuse, illuminated.
PSU             | Tracopower | TXL 035-0512D  | RS Comp  | £59-14 | 35 W, 5/12 VDC (for lights/arduino). Check currents.
PSU alternative | Tracopower | TXL 060-0512DI | RS Comp  | £81-66 | 60 W, 5/12 VDC (for lights/arduino and extra phone charge.
Controller      | Arduino    | ABX00021       | Amazon   | £37-15 | ARDUINO UNO WiFi REV2. WiFi for clock function.
Switching       | Arduino    | A000110        | Amazon   | £32-99 | ARDUINO 4 RELAYS SHIELD - seems good enough for lights. Check currents
Switched out    | Startech   | USBPLATE4     | Amazon   |  £5-65 | 4 Port USB A Female Slot Plate Adapter. For Xmas lights.
Fixed out       | Startech   | unknown       | Garage!  |  £0-00 | 2 Port USB A Female Slot Plate Adapter. For phones.

## Obtained components

Item            | Mfr        | PN             | Supplier     | Cost   | Notes
----------------|------------|----------------|--------------|--------|---
Box             |            |                |              |        | 
Mains In        | Schurter   | DD11.0114.1111 | RS Comp      |  £6-95 | 
PSU alternative | Tracopower | TXL 060-0512DI | Ebay         | £19-99 | Second hand. Slightly battered.
Controller      | Arduino    | ABX00021       | Ebay (Rapid) | £42-22 | 
Switching       | Arduino    | A000110        | Ebay (Rapid) | £23-19 | 
Switched out    | Startech   | USBPLATE4      | Ebay         |  £5-04 | 
Fixed out       | Startech   | unknown        | Garage!      |  £0-00 | 

## Ideas

Basically to power lights on and off at set times.

But could add subtle clock functionality.
1. Turn off at the hour and then blink on the hour.
1. Blink off 1, 2 or 3 times at 15, 30 or 45 past the hour.
1. Go crazy at 4pm for tea - blink out the tea for two tune rhythm?
1. Others?

Probably not sophis enough for PWM through the relays! :(

Include indicator LEDs in box so lights don't have to be connected for testing?

Power LED lihgt from traco visible?

Other permanently on UBS charger socket? Use the 2 USB A Startech thing in Sid's garage. Wire direct from PSU. But where to get the extra current from as we're already a limit of 5 v PSU. See below.

## Power calculations

### 5 V

4 off switched USB lights at 5 V and 1 A each

2 off unswitched USB charging ports at 5 V and 1 A each

Total = 30 W and 6 A

### 12 V

Arduino 7-12 V, 40 mA per IO pin. So < 1 A.

Total =  12 V x 1 A = 12 W

### Total power

Total = 42 W
