import os
import struct

can_struct_format = '<L??B8sxL'
struct_size = struct.calcsize(can_struct_format)

ANALOGMSG   = 0x000E
CANMSG0     = 0x0140
CANMSG1     = 0x0140
CANMSG2     = 0x0140
CANMSG3     = 0x0140
CANMSG4     = 0x0140
CANMSG5     = 0x0140

def main():
    for root, dirs, files in os.walk('.'):
        for file in files:
            if file.endswith('.bin'):

                print(f"Processing {file} ...")

                basename = file.removesuffix('.bin')

                starttimems = int(basename.split('_')[3])

                logname = basename + '.log'

                logdata = process_bin(file, starttimems)
                
                print(f"Writing {logname} ...")
                with open(logname, 'w') as logfile:
                    logfile.writelines(logdata)

def process_bin(binfile, starttimems):
    logdata = []

    with open(binfile, 'rb') as binfile:
        while chunk := binfile.read(20):
            if len(chunk) != struct_size:
                print("incomplete message")

            rxID, rtr, ext, dlc, rxBuf, mstime = struct.unpack(can_struct_format, chunk)
            rxBuf_list = list(rxBuf[:dlc])
            
            adjtime = mstime - starttimems
            time = adjtime * 0.001

            logline = (f"rxID: {rxID:#05X}, rtr: {rtr:0}, ext: {ext:0}, dlc: {dlc}, "
                        f"rxBuf: {' '.join(f'{b:02X}' for b in rxBuf)}, time: {time:#010.3f}")
            
            logline = logline + '\n'
            logdata.append(logline)
    
    return logdata

if __name__ == "__main__" :
    main()

