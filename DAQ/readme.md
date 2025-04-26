MAKE SURE CAN IS CONNECTED FROM THE BAMOCAR to the Dash CAN BUS (only place that can is connected to VCU)

to start, close the arduino ide after setting up your devices and confirm that you are able to see the data you want in the serial console
the devices that you will need are 0x3210 (cooling) and 0x1010 (bamocar).

    To enable these devices type ENABLE=0x3210 into the Serial Console in the Arduino IDE. Also ENABLE=0x1010. 
    
    once you enable it you don't need to re-enable it every time.

    to double check that you enabled the devices you could type "h" into serial console but you should see all the print statements going crazy in the serial console.

# after you are done running your experiment, close the arduino ide COMPLETELY!

# IN VS CODE OPNE UP THE FOLDER DAQ, if you are reading this you should be able to see it lol. 
Run logSaver.py and let it run for a bit before clicking Ctrl+C (force quit the program). You should see a new text file in your folder called log.txt.
    
    Total time need to wait is probably 1/20th of the time you ran the test. so like 10 minutes you can let the program run for 30 minutes. Maybe? idk lol.

enter in the name of the file, it will generate graphs as well as a CSV file. (CSV file name will be "parsed_output.csv)

If you want to run multiple tests, flash the VCU again with the same code by running gevcu-2024-2025.ino and uploading it via Arduino IDE.

## MAKE SURE U R RENAMING THE files "log.txt" to something else after you run each test so that the program does not overwrite to the same file and the data is lost
## Worst comes to worst, take the SD CARD out of the VCU and use the SD card adapter from the blue box and read the txt file manually. 

TO TURN ON THE PUMP GO INTO THE SERIAL CONSOLE OF THE ARDUINO IDE AND WRITE "J"
TO TURN IT OFF WRITE "K"