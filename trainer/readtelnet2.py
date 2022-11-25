
import telnetlib
import time

import sys

timestr = time.strftime("%Y%m%d-%H%M%S")

filename = timestr + '.csv'

HOST = "192.168.4.1"
connect = "connect"

lastmsgtime = time.time()
timeout = 10

try:
    with telnetlib.Telnet(HOST, timeout=2) as tn:

        tn.write(connect.encode('ascii') + b"\n")

        while True:
            message = tn.read_until(b"\n", timeout=1)
            if len(message):
                lastmsgtime = time.time()
                with open(filename, "a") as outputfile:
                    outputfile.write(time.strftime("%H%M%S") + "," + message.decode('ascii'))
            if (time.time() - lastmsgtime) > timeout:  
                break
        tn.close()
    print('no msg for 10s. Halting...')
except KeyboardInterrupt:
    print('\nHalting execution..')
except:
    print('\nReconnecting...')


