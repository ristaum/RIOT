*** Settings ***
Library        Process
Library        SerialLibrary
Library        MyLib.py

*** Test Cases ***
Example Test
    Connect            /dev/ttyACM0    115200
    Set Timeout        5
    Run Process        make     BOARD\=nucleo-f411re     flash
    i2c_slave_init     0x76
    # Write            Hello, world!
    ${data}            Read until    TT
    
    # ${data}=           Read until         Hello from UART!
    # Log                ${data}
    # Should Contain    ${data}    tgest
    Should Be Equal As Strings    ${data}    TT
    Disconnect
