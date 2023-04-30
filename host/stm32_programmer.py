import serial
import struct
import os
import sys
import glob

# Status replies from the bootloader 
Flash_HAL_OK                = 0x00
Flash_HAL_ERROR             = 0x01
Flash_HAL_BUSY              = 0x02
Flash_HAL_TIMEOUT           = 0x03
Flash_HAL_INVALID_ADDR      = 0x04

# Bootloader Commands
CMD_BL_GET_VER                  = 0x51
CMD_BL_GET_HELP                 = 0x52
CMD_BL_GET_CID                  = 0x53
CMD_BL_GET_RDP_STATUS           = 0x54
CMD_BL_GO_TO_ADDR               = 0x55
CMD_BL_FLASH_ERASE              = 0x56
CMD_BL_FLASH_WRITE              = 0x57
CMD_BL_FLASH_READ               = 0x58
CMD_BL_ENABLE_RW_PROTECT        = 0x59
CMD_BL_DISABLE_RW_PROTECT       = 0x5A
CMD_BL_READ_FLASH_SECTOR_STATUS = 0x5B
CMD_BL_OTP_READ                 = 0x5C

# Length in bytes of the commands to be sent to the bootloader
CMD_BL_GET_VER_LEN                  = 6
CMD_BL_GET_HELP_LEN                 = 6
CMD_BL_GET_CID_LEN                  = 6
CMD_BL_GET_RDP_STATUS_LEN           = 6
CMD_BL_GO_TO_ADDR_LEN               = 10
CMD_BL_FLASH_ERASE_LEN              = 8
CMD_BL_FLASH_WRITE_LEN              = 11
CMD_BL_FLASH_READ_LEN               = 8
CMD_BL_ENABLE_RW_PROTECT_LEN        = 8
CMD_BL_DISABLE_RW_PROTECT_LEN       = 6
CMD_BL_READ_FLASH_SECTOR_STATUS_LEN = 6

verbose_mode = 0
mem_write_active = 0


#----------------------------- file operations ----------------------------------------

def calc_file_len():
    size = os.path.getsize("user_app.bin")
    return size

def open_the_file():
    global bin_file
    bin_file = open('user_app.bin','rb')

def close_the_file():
    bin_file.close()


#----------------------------- utilitie functions ----------------------------------------

def word_to_byte(addr, index , lowerfirst):
    value = (addr >> ( 8 * ( index -1)) & 0x000000FF )
    return value

def get_crc(buff, length):
    crc = 0xFFFFFFFF
    for data in buff[0:length]:
        crc = crc ^ data
        for i in range(32):
            if(crc & 0x80000000):
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc = (crc << 1)
    return crc


#----------------------------- serial port ----------------------------------------

def serial_ports():
    """ Lists serial port names
        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def serial_port_configuration(port):
    global ser
    try:
        ser = serial.Serial(port, 115200, timeout = 20)
    except:
        print("\n Oops! That was not a valid port")
        
        port = serial_ports()
        if(not port):
            print("\n No ports Detected")
        else:
            print("\n Here are some available ports on your PC. Try Again!")
            print("\n", port)
        return -1
    if ser.is_open:
        print("\n Port Open Success")
    else:
        print("\n Port Open Failed")
    return 0
 
def read_serial_port(length):
    read_value = ser.read(length)
    return read_value

def purge_serial_port():
    ser.reset_input_buffer()
    
def write_to_serial_port(value, *length):
        data = struct.pack('>B', value)
        if (verbose_mode):
            value = bytearray(data)
            print("0x{:02x}" .format(value[0]), end = ' ')
        if(mem_write_active and (not verbose_mode)):
                print("#", end = ' ')
        ser.write(data)


#----------------------------- command processing ----------------------------------------

def process_CMD_BL_GET_VER(length):
    reply = read_serial_port(length)
    version = bytearray(reply)
    print("\n Bootloader Version: ", hex(version[0]))

def process_CMD_BL_GET_HELP(length):
    reply = read_serial_port(length) 
    help_list = bytearray(reply)
    print("\n Supported Commands: ", end=' ')
    for x in help_list:
        print(hex(x), end=' ')
    print()

def process_CMD_BL_GET_CID(length):
    cid = read_serial_port(length)
    device_id = (cid[1] << 8 ) + cid[0]
    print("\n Device ID: ", hex(device_id))
    revision_id = (cid[3] << 8 ) + cid[2]
    print("\n Revision ID: ", hex(revision_id))

def process_CMD_BL_GET_RDP_STATUS(length):
    reply = read_serial_port(length)
    rdp = bytearray(reply)
    print("\n RDP Status: ", hex(rdp[0]))

def process_CMD_BL_GO_TO_ADDR(length):
    reply = read_serial_port(length)
    addr_status = bytearray(reply)
    print("\n Address Status: ", hex(addr_status[0]))

def process_CMD_BL_FLASH_ERASE(length):
    reply = read_serial_port(length)
    if len(reply):
        erase_status = bytearray(reply)
        if(erase_status[0] == Flash_HAL_OK):
            print("\n Erase Status: FLASH_HAL_OK")
        elif (erase_status[0] == Flash_HAL_ERROR):
            print("\n Erase Status: FLASH_HAL_ERROR")
        elif (erase_status[0] == Flash_HAL_BUSY):
            print("\n Erase Status: FLASH_HAL_BUSY")
        elif (erase_status[0] == Flash_HAL_TIMEOUT):
            print("\n Erase Status: FLASH_HAL_TIMEOUT")
        elif (erase_status[0] == Flash_HAL_INVALID_ADDR):
            print("\n Erase Status: FLASH_HAL_INV_SECTOR")
        else:
            print("\n Erase Status: UNKNOWN_ERROR_CODE")
    else:
        print(" Timeout: Bootloader is not responding")

def process_CMD_BL_FLASH_WRITE(length):
    reply = read_serial_port(length)
    write_status = bytearray(reply)
    if(write_status[0] == Flash_HAL_OK):
        print("\n Write_status: FLASH_HAL_OK")
    elif (write_status[0] == Flash_HAL_ERROR):
        print("\n Write_status: FLASH_HAL_ERROR")
    elif (write_status[0] == Flash_HAL_BUSY):
        print("\n Write_status: FLASH_HAL_BUSY")
    elif (write_status[0] == Flash_HAL_TIMEOUT):
        print("\n Write_status: FLASH_HAL_TIMEOUT")
    elif (write_status[0] == Flash_HAL_INVALID_ADDR):
        print("\n Write_status: FLASH_HAL_INV_ADDR")
    else:
        print("\n Write_status: UNKNOWN_ERROR")

protection_mode = ["Write Protection", "Read/Write Protection", "No protection"]
def protection_type(status, n):
    if(status[1] & (1 << 7)):
        # PCROP is enabled
        if(status[0] & (1 << n) ):
            return protection_mode[1]
        else:
            return protection_mode[2]
    else:
        if(status[0] & (1 << n)):
            return protection_mode[2]
        else:
            return protection_mode[0]

def process_CMD_BL_READ_SECTOR_STATUS(length):
    reply = read_serial_port(length)
    status = bytearray(reply)
    if(status[1] & (1 << 7)):
        # PCROP is enabled
        print("\n Flash protection mode: \tPCROP Protection")
    else:
        print("\n Flash protection mode: \tWRITE Protection")
    print("\n ====================================")
    print("\n Sector                     \tProtection") 
    print("\n ====================================")

    for x in range(8):
        print("\n Sector{0}                     \t{1}" .format(x, protection_type(status, x)))
        
def process_CMD_BL_DISABLE_RW_PROTECT(length):
    reply = read_serial_port(length)
    status = bytearray(reply)
    if(status[0]):
        print("\n FAIL")
    else:
        print("\n SUCCESS")

def process_CMD_BL_ENABLE_RW_PROTECT(length):
    reply = read_serial_port(length)
    status = bytearray(reply)
    if(status[0]):
        print("\n FAIL")
    else:
        print("\n SUCCESS")

def decode_menu_command_code(command):
    ret_value = 0

    data_buf = []
    for i in range(255):
        data_buf.append(0)
    
    if(command == 0):
        print("\n Exiting...!")
        raise SystemExit

    elif (command == 1):
        print("\n Command == > BL_GET_VER")
        data_buf[0] = CMD_BL_GET_VER_LEN - 1 
        data_buf[1] = CMD_BL_GET_VER 
        crc32       = get_crc(data_buf, CMD_BL_GET_VER_LEN - 4)
        crc32 = crc32 & 0xffffffff
        data_buf[2] = word_to_byte(crc32, 1, 1) 
        data_buf[3] = word_to_byte(crc32, 2, 1) 
        data_buf[4] = word_to_byte(crc32, 3, 1) 
        data_buf[5] = word_to_byte(crc32, 4, 1) 

        write_to_serial_port(data_buf[0], 1)
        for i in data_buf[1:CMD_BL_GET_VER_LEN]:
            write_to_serial_port(i, CMD_BL_GET_VER_LEN - 1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif (command == 2):
        print("\n Command == > BL_GET_HELP")
        data_buf[0] = CMD_BL_GET_HELP_LEN - 1 
        data_buf[1] = CMD_BL_GET_HELP 
        crc32       = get_crc(data_buf, CMD_BL_GET_HELP_LEN - 4)
        crc32 = crc32 & 0xffffffff
        data_buf[2] = word_to_byte(crc32, 1, 1) 
        data_buf[3] = word_to_byte(crc32, 2, 1) 
        data_buf[4] = word_to_byte(crc32, 3, 1) 
        data_buf[5] = word_to_byte(crc32, 4, 1) 

        write_to_serial_port(data_buf[0], 1)
        for i in data_buf[1:CMD_BL_GET_HELP_LEN]:
            write_to_serial_port(i, CMD_BL_GET_HELP_LEN-1)

        ret_value = read_bootloader_reply(data_buf[1])

    elif (command == 3):
        print("\n Command == > BL_GET_CID")
        data_buf[0] = CMD_BL_GET_CID_LEN - 1 
        data_buf[1] = CMD_BL_GET_CID 
        crc32       = get_crc(data_buf, CMD_BL_GET_CID_LEN - 4)
        crc32 = crc32 & 0xffffffff
        data_buf[2] = word_to_byte(crc32, 1, 1) 
        data_buf[3] = word_to_byte(crc32, 2, 1) 
        data_buf[4] = word_to_byte(crc32, 3, 1) 
        data_buf[5] = word_to_byte(crc32, 4, 1) 

        write_to_serial_port(data_buf[0], 1)
        for i in data_buf[1:CMD_BL_GET_CID_LEN]:
            write_to_serial_port(i, CMD_BL_GET_CID_LEN - 1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif (command == 4):
        print("\n Command == > BL_GET_RDP_STATUS")
        data_buf[0] = CMD_BL_GET_RDP_STATUS_LEN - 1
        data_buf[1] = CMD_BL_GET_RDP_STATUS
        crc32       = get_crc(data_buf, CMD_BL_GET_RDP_STATUS_LEN - 4)
        crc32 = crc32 & 0xffffffff
        data_buf[2] = word_to_byte(crc32, 1, 1)
        data_buf[3] = word_to_byte(crc32, 2, 1)
        data_buf[4] = word_to_byte(crc32, 3, 1)
        data_buf[5] = word_to_byte(crc32, 4, 1)
        
        write_to_serial_port(data_buf[0], 1)
        
        for i in data_buf[1:CMD_BL_GET_RDP_STATUS_LEN]:
            write_to_serial_port(i, CMD_BL_GET_RDP_STATUS_LEN - 1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif (command == 5):
        print("\n Command == > BL_GO_TO_ADDR")
        go_address = input("\n Please enter a 4 byte address in hex: ")
        go_address = int(go_address, 16)
        data_buf[0] = CMD_BL_GO_TO_ADDR_LEN - 1 
        data_buf[1] = CMD_BL_GO_TO_ADDR 
        data_buf[2] = word_to_byte(go_address, 1, 1) 
        data_buf[3] = word_to_byte(go_address, 2, 1) 
        data_buf[4] = word_to_byte(go_address, 3, 1) 
        data_buf[5] = word_to_byte(go_address, 4, 1) 
        crc32       = get_crc(data_buf,CMD_BL_GO_TO_ADDR_LEN - 4) 
        data_buf[6] = word_to_byte(crc32, 1, 1) 
        data_buf[7] = word_to_byte(crc32, 2, 1) 
        data_buf[8] = word_to_byte(crc32, 3, 1) 
        data_buf[9] = word_to_byte(crc32, 4, 1) 

        write_to_serial_port(data_buf[0], 1)
        
        for i in data_buf[1:CMD_BL_GO_TO_ADDR_LEN]:
            write_to_serial_port(i, CMD_BL_GO_TO_ADDR_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif (command == 6):
        print("\n Command == > BL_FLASH_ERASE")
        data_buf[0] = CMD_BL_FLASH_ERASE_LEN - 1 
        data_buf[1] = CMD_BL_FLASH_ERASE 
        sector_num = input("\n Enter sector number (0-7 or 0xFF): ")
        sector_num = int(sector_num, 16)
        data_buf[2] = sector_num
        data_buf[3] = 0
        if(sector_num != 0xff):
            nsec = int(input("\n Enter number of sectors to erase (maximum 8): "))
            data_buf[3] = nsec  

        crc32       = get_crc(data_buf, CMD_BL_FLASH_ERASE_LEN - 4) 
        data_buf[4] = word_to_byte(crc32, 1, 1) 
        data_buf[5] = word_to_byte(crc32, 2, 1) 
        data_buf[6] = word_to_byte(crc32, 3, 1) 
        data_buf[7] = word_to_byte(crc32, 4, 1) 

        write_to_serial_port(data_buf[0], 1)
        
        for i in data_buf[1:CMD_BL_FLASH_ERASE_LEN]:
            write_to_serial_port(i, CMD_BL_FLASH_ERASE_LEN - 1)

        if(sector_num == 0xff):
            print("Bootloader will not respond with status after Flash mass-erase!\n")
        else:
            ret_value = read_bootloader_reply(data_buf[1])
        
    elif (command == 7):
        print("\n Command == > BL_MEM_WRITE")
        bytes_remaining = 0
        t_len_of_file = 0
        bytes_so_far_sent = 0
        len_to_read = 0
        base_mem_address = 0

        data_buf[1] = CMD_BL_FLASH_WRITE

        # first get the total number of bytes in the .bin file.
        t_len_of_file = calc_file_len()

        # keep opening the file
        open_the_file()

        bytes_remaining = t_len_of_file - bytes_so_far_sent

        base_mem_address = input("\n Enter the memory write address here: ")
        base_mem_address = int(base_mem_address, 16)
        global mem_write_active
        while (bytes_remaining):
            mem_write_active = 1
            if(bytes_remaining >= 128):
                len_to_read = 128
            else:
                len_to_read = bytes_remaining

            # get the bytes into buffer by reading file
            for x in range(len_to_read):
                file_read_value = bin_file.read(1)
                file_read_value = bytearray(file_read_value)
                data_buf[7 + x] = int(file_read_value[0])

            # populate base mem address
            data_buf[2] = word_to_byte(base_mem_address, 1, 1)
            data_buf[3] = word_to_byte(base_mem_address, 2, 1)
            data_buf[4] = word_to_byte(base_mem_address, 3, 1)
            data_buf[5] = word_to_byte(base_mem_address, 4, 1)
            data_buf[6] = len_to_read

            mem_write_cmd_total_len = CMD_BL_FLASH_WRITE_LEN + len_to_read

            # first field is "len_to_follow"
            data_buf[0] = mem_write_cmd_total_len - 1
            crc32                    = get_crc(data_buf,mem_write_cmd_total_len - 4)
            data_buf[7 + len_to_read]  = word_to_byte(crc32, 1, 1)
            data_buf[8 + len_to_read]  = word_to_byte(crc32, 2, 1)
            data_buf[9 + len_to_read]  = word_to_byte(crc32, 3, 1)
            data_buf[10 + len_to_read] = word_to_byte(crc32, 4, 1)

            # update base mem address for the next loop
            base_mem_address += len_to_read

            write_to_serial_port(data_buf[0], 1)
        
            for i in data_buf[1:mem_write_cmd_total_len]:
                write_to_serial_port(i, mem_write_cmd_total_len - 1)

            bytes_so_far_sent += len_to_read
            bytes_remaining = t_len_of_file - bytes_so_far_sent
            print("\n Bytes sent so far: {0} -- Bytes remaining: {1}\n" .format(bytes_so_far_sent, bytes_remaining)) 
        
            ret_value = read_bootloader_reply(data_buf[1])
        mem_write_active=0
    
    elif (command == 8):
        print(" Command == > BL_FLASH_READ")
        print("\n This command is not supported")

    elif (command == 9):
        print("\n Command == > BL_ENABLE_RW_PROTECT")
        total_sector = int(input("\n How many sectors do you want to protect? "))
        sector_numbers = [0,0,0,0,0,0,0,0]
        sector_details = 0
        for x in range(total_sector):
            sector_numbers[x] = int(input("\n Enter sector number[{0}]: " .format(x+1)))
            sector_details = sector_details | (1 << sector_numbers[x])

        print("\n WRITE Protection: 1")
        print("\n PCROP Protection: 2")
        mode = input("\n Enter sector protection mode (WRITE or PCROP): ")
        mode = int(mode)
        if(mode != 2 and mode != 1):
            print("\n Invalid option!")
            return

        data_buf[0] = CMD_BL_ENABLE_RW_PROTECT_LEN - 1 
        data_buf[1] = CMD_BL_ENABLE_RW_PROTECT 
        data_buf[2] = sector_details 
        data_buf[3] = mode 
        crc32       = get_crc(data_buf,CMD_BL_ENABLE_RW_PROTECT_LEN - 4) 
        data_buf[4] = word_to_byte(crc32, 1, 1) 
        data_buf[5] = word_to_byte(crc32, 2, 1) 
        data_buf[6] = word_to_byte(crc32, 3, 1) 
        data_buf[7] = word_to_byte(crc32, 4, 1) 

        write_to_serial_port(data_buf[0], 1)
        
        for i in data_buf[1:CMD_BL_ENABLE_RW_PROTECT_LEN]:
            write_to_serial_port(i, CMD_BL_ENABLE_RW_PROTECT_LEN - 1)
        
        ret_value = read_bootloader_reply(data_buf[1])
            
    elif (command == 10):
        print("\n Command == > BL_DISABLE_RW_PROTECT")
        data_buf[0] = CMD_BL_DISABLE_RW_PROTECT_LEN - 1 
        data_buf[1] = CMD_BL_DISABLE_RW_PROTECT 
        crc32       = get_crc(data_buf, CMD_BL_DISABLE_RW_PROTECT_LEN - 4) 
        data_buf[2] = word_to_byte(crc32, 1, 1) 
        data_buf[3] = word_to_byte(crc32, 2, 1) 
        data_buf[4] = word_to_byte(crc32, 3, 1) 
        data_buf[5] = word_to_byte(crc32, 4, 1) 

        write_to_serial_port(data_buf[0], 1)
        
        for i in data_buf[1:CMD_BL_DISABLE_RW_PROTECT_LEN]:
            write_to_serial_port(i, CMD_BL_DISABLE_RW_PROTECT_LEN - 1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif (command == 11):
        print("\n Command == > BL_READ_FLASH_SECTOR_STATUS")
        data_buf[0] = CMD_BL_READ_FLASH_SECTOR_STATUS_LEN - 1 
        data_buf[1] = CMD_BL_READ_FLASH_SECTOR_STATUS 

        crc32       = get_crc(data_buf,CMD_BL_READ_FLASH_SECTOR_STATUS_LEN - 4) 
        data_buf[2] = word_to_byte(crc32, 1, 1) 
        data_buf[3] = word_to_byte(crc32, 2, 1) 
        data_buf[4] = word_to_byte(crc32, 3, 1) 
        data_buf[5] = word_to_byte(crc32, 4, 1) 

        write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:CMD_BL_READ_FLASH_SECTOR_STATUS_LEN]:
            write_to_serial_port(i, CMD_BL_READ_FLASH_SECTOR_STATUS_LEN - 1)
        
        ret_value = read_bootloader_reply(data_buf[1])

    elif (command == 12):
        print("\n Command == > BL_OTP_READ")
        print("\n This command is not supported")

    else:
        print("\n Please input valid command code\n")
        return

    if ret_value == -2 :
        print("\n Timeout: No response from the bootloader")
        print("\n Reset the board and try Again!")
        return

def read_bootloader_reply(command_code):
    len_to_follow = 0 
    ret = -2 

    ack = read_serial_port(2)
    if(len(ack)):
        ack_array = bytearray(ack) 
        if (ack_array[0] == 0xA5):
            # CRC of the last command was verified successfully, received ACK and "len to follow" from the MCU
            len_to_follow = ack_array[1]
            print("\n CRC32 SUCCESS!")
            if (command_code) == CMD_BL_GET_VER :
                process_CMD_BL_GET_VER(len_to_follow)
                
            elif (command_code) == CMD_BL_GET_HELP:
                process_CMD_BL_GET_HELP(len_to_follow)
                
            elif (command_code) == CMD_BL_GET_CID:
                process_CMD_BL_GET_CID(len_to_follow)
                
            elif (command_code) == CMD_BL_GET_RDP_STATUS:
                process_CMD_BL_GET_RDP_STATUS(len_to_follow)
                
            elif (command_code) == CMD_BL_GO_TO_ADDR:
                process_CMD_BL_GO_TO_ADDR(len_to_follow)
                
            elif (command_code) == CMD_BL_FLASH_ERASE:
                process_CMD_BL_FLASH_ERASE(len_to_follow)
                
            elif (command_code) == CMD_BL_FLASH_WRITE:
                process_CMD_BL_FLASH_WRITE(len_to_follow)
                
            elif (command_code) == CMD_BL_READ_FLASH_SECTOR_STATUS:
                process_CMD_BL_READ_SECTOR_STATUS(len_to_follow)
                
            elif (command_code) == CMD_BL_ENABLE_RW_PROTECT:
                process_CMD_BL_ENABLE_RW_PROTECT(len_to_follow)
                
            elif (command_code) == CMD_BL_DISABLE_RW_PROTECT:
                process_CMD_BL_DISABLE_RW_PROTECT(len_to_follow)
                
            else:
                print("\n Invalid command code!\n")
                
            ret = 0
         
        elif ack_array[0] == 0x7F:
            # CRC of the last command was bad, received NACK from the MCU
            print("\n CRC: FAIL \n")
            ret = -1
    else:
        print("\n Timeout: Bootloader not responding...")
        
    return ret

#----------------------------- ask menu implementation ----------------------------------------

name = input("Enter the port: ")
ret = serial_port_configuration(name)
if (ret < 0):
    decode_menu_command_code(0)

while True:
    print("\n +==========================================+")
    print(" |               Menu                       |")
    print(" |       STM32F4 BootLoader v1              |")
    print(" +==========================================+")

    print("\n Which BL command do you want to send?\n")
    print(" BL_GET_VER                            --> 1")
    print(" BL_GET_HELP                           --> 2")
    print(" BL_GET_CID                            --> 3")
    print(" BL_GET_RDP_STATUS                     --> 4")
    print(" BL_GO_TO_ADDR                         --> 5")
    print(" BL_FLASH_ERASE                        --> 6")
    print(" BL_FLASH_WRITE                        --> 7")
    print(" BL_FLASH_READ                         --> 8")
    print(" BL_ENABLE_RW_PROTECT                  --> 9")
    print(" BL_DISABLE_RW_PROTECT                 --> 10")
    print(" BL_READ_FLASH_SECTOR_STATUS           --> 11")
    print(" BL_OTP_READ                           --> 12")
    print(" MENU_EXIT                             --> 0")

    command_code = input("\n Enter the command code: ")

    if(not command_code.isdigit()):
        print("\n Please Input valid code shown above")
    else:
        decode_menu_command_code(int(command_code))

    input("\n Press any key to continue: ")
    purge_serial_port()
