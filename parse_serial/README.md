# Parsing Serial Data via ROS
### Files 
1. `include/parse_serial.h`: `parseSerial` class definitions and headers.
2. `src/parse_serial.cpp`: `parseSerial` member function definitions and `main` node definitions.
3. `msg/K_Asset.msg` & `msg/K_Personel.msg`: message definitions for publishing information.
4. `launch/parse_serial.launch`: launch file w/ serial port initialization parameters. Defaults are `/dev/ttyS19` and `9600`.

### Operational Notes
`parse_serial` is a ROS package build based on a pre-set input information stream through UART serial port. Parsing is done under 2 major functions.
1. `parseSerial::serialReadwMarkers`: This function looks for the marker "$$" for recording input stream and stops at "\**" marker. The function is of type `boolean` and returns **true** if a complete message is received. It stores the actual message to the member variable **inputString** of type `std::string`.
2. `parseSerial::serialParser`: Member function is called inside the main code upon **true** result return from serial reader and parses the message according to its `field` name, which can be K_Personel, K_Asset and K_Update. This is the most error prone part of the package since it is very sensitive to wrongly formatted message data. To minimize the possible pointer and conversion errors, **inputString** must be parsed tidiously.

*Note:* The USB port initialization is done inside the `constructor` of parseSerial class. The `port_name` and the `baud_rate` are parametrized to prevent recompilation each time a new port needs to be listened. Usage with parameters is demonstrated via an example `parse_serial.launch` file.

### Method for validation
An Arduino Uno board is loaded with a simple Serial print code at 9600 bps to print the sample trial stream:
    `$$K_Personel,Oguz,24,74.1,165**$$K_Personel,Mahmut,99,50,170**$$K_Update**$$K_Asset,69,HW**`
being published periodically. Then the corresponding port is listened and published topics `/update` `/personel` and `/asset` are observed. 

### Possible Improvements
1. Parametrization for type of message to parse. Such an implementation can be useful for large projects but an overkill for this size.
2. A neater parser, with more fail-checks. Current one gets the job done as long as input data is not corrupt, i.e it has missing parts inside that is expected to be there.

End of documentation.

*Author*: Yunus Emre İkiz

*email*: yunus.ikiz@ieee.metu.edu.tr

*date*: 26.02.2020 

*Ne mutlu Türk'üm diyene!*