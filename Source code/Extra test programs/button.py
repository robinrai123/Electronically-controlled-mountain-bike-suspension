import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library

def button_callback(channel):
    print("Button was pushed!\n")
    
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(29, GPIO.IN, GPIO.PUD_UP) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
GPIO.add_event_detect(29,GPIO.RISING,callback=button_callback) # Setup event on pin 10 rising edge


message = input("Press enter to quit\n\n") # Run until someone presses enter


GPIO.cleanup() # Clean up