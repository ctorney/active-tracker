
import telnetlib
import time

timestr = time.strftime("%Y%m%d-%H%M%S")

filename = timestr + '.csv'

HOST = "192.168.4.1"
connect = "connect"

try:
    with telnetlib.Telnet(HOST) as tn:

        tn.write(connect.encode('ascii') + b"\n")

        while True:
            message = tn.read_until(b"\n")
            with open(filename, "a") as outputfile:
                outputfile.write(time.strftime("%H%M%S") + "," + message.decode('ascii'))

except KeyboardInterrupt:
    print('\nHalting execution..')


