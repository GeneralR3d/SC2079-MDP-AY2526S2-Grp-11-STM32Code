
import serial
import time
import threading

# Configuration
# Replace '/dev/ttyS0' or '/dev/ttyAMA0' with the correct UART port on your Pi
# For USB-to-Serial adapter, it might be '/dev/ttyUSB0'
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200

def read_from_stm32(ser):
    """
    Continuously read lines from STM32 and print them.
    This runs in a separate thread.
    """
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[STM32]: {line}")
        except OSError:
            break
        except Exception as e:
            print(f"Error reading: {e}")
            break

def send_command(ser, command):
    """
    Send a command string to the STM32.
    Appends \n as required by the STM32 logic.
    """
    print(f"[Pi] Sending: {command}")
    # STM32 expects command terminated by \n or \r
    full_cmd = command + '\n'
    ser.write(full_cmd.encode('utf-8'))

def main():
    try:
        # Initialize Serial Port
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Opened {SERIAL_PORT} at {BAUD_RATE} baud")
        
        # Start reading thread
        reader_thread = threading.Thread(target=read_from_stm32, args=(ser,), daemon=True)
        reader_thread.start()

        # Allow time for connection to settle
        time.sleep(2)

        # Example Sequence
        
        # 1. Drive forward 50cm at PWM 2000
        # Command format: drive(target_cm, base_pwm)
        # Note: Ensure arguments match what sscanf expects: "%f,%d"
        send_command(ser, "drive(50,2000)")
        
        # Wait for operation to complete (adjust timing as needed)
        time.sleep(5)
        
        # 2. Stop
        send_command(ser, "stop")
        
        # Wait a bit
        time.sleep(2)
        
        # 3. Drive again
        send_command(ser, "drive(20,1500)")
        
        time.sleep(3)
        send_command(ser, "stop")

    except serial.SerialException as e:
        print(f"Could not open serial port {SERIAL_PORT}: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
