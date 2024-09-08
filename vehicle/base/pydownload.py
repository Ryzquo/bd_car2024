#coding=utf-8

import serial
import serial.tools.list_ports
import time
import os
import sys
import argparse
# import common

'''
下载调用
.\pydownload.py -file test.bin -address 0x800c000 -msg 1
-file test.bin          文件名，默认”test.bin“
-address 0x800c000      下载地址，默认”0x800c000“
-msg 1                  提示信息丰富程度 1简要 5中度可调试 20最多
'''

ver = 0.1


'''
文件夹路径
MC902P
MC602P
'''
#CPDIR = "MC_902P\\"
CPDIR = "MC_602P"




# 显示软件名称版本
# print("Whales Robot pydownload ver:" + str(ver))

#硬件名称
HARD_UNKNOW     =0
HARD_102        =102
HARD_902        =902
HARD_1002       =1002
HARD_602        =602
DEVICE_NAME_902="MC_902P"
DEVICE_NAME_102="MC_102P"
DEVICE_NAME_602="MC_602P"
DEVICE_NAME__UNKNOW="UNKNOW"


#串口默认波特率1M
BUAD         = 1000000#串口通信波特率
TIMEOUT      = 0.2#串口通信超时

#USB转串口，会有PID和VID参数，可通过此参数识别串口，加快串口识别过程
PID=29987
VID=6790


DownloadAddress = 0x8000000+(48*1024)#从第48K的地方开始升级固件,此参数后续作为命令行参数


PING_TIME_DELAY = 0.005#PING指令的回复延时
DW_TIME_DELAY   = 0.005#下载指令的回复延时
RAM2FLASH_TIME_DELAY   = 0.1#下载指令的回复延时
RUNCODE_TIME_DELAY     = 0.05#运行指令的回复延时
PRINT_LEVEL =5#5#5#5信息少,20信息多，小于此数字的Prinf将被显示出来
CRC_TIME_DELAY     = 0.005#运行指令的回复延时

#定义各程序所在地址
FLASH_BASE      =0x8000000
PROGRAM_BOOT    =FLASH_BASE#boot程序所在地址
PROGRAM_GCC     =(FLASH_BASE + (52*1024)) #0x0800C000  GCC编译程序所在地址
PROGRAM_GUI     =(FLASH_BASE + (256*1024))#0x08040000  用户操作界面所在地址
PROGRAM_USBMASS =(FLASH_BASE + (384*1024))#0x08060000  U盘固件所在地址

#MC602总共6个程序，地址如下
MC602_APP1_ADDRESS =(FLASH_BASE + (384*1024))#0x08060000
MC602_APP2_ADDRESS =(FLASH_BASE + (512*1024))#0x08080000
MC602_APP3_ADDRESS =(FLASH_BASE + (612*1024))#0x08099000
MC602_APP4_ADDRESS =(FLASH_BASE + (712*1024))#0x080B2000
MC602_APP5_ADDRESS =(FLASH_BASE + (812*1024))#0x080CB000
MC602_APP6_ADDRESS =(FLASH_BASE + (912*1024))#0x080E4000




#存储CODE的缓存
code=bytearray()

#ping数据包长度
CMD_PING_SEND_SIZE = 8
CMD_PING_REV_SIZE  = 10
#下载文件的数据包长度
CMD_BUFFER_4K_LEN  = 4*1024#纯数据长度
CMD_DW_SEND_SIZE = CMD_BUFFER_4K_LEN+8#数据加命令和校验码长度
CMD_DW_REV_SIZE  = 8#回复数据长度
#复制Ram2Flash指令的数据包长度
CMD_RAM2FLASH_SEND_SIZE = 11
CMD_RAM2FLASH_REV_SIZE  = 11
#将程序名称烧录到STM32上
CMD_SAVEFILENAME_SEND_SIZE = 16
CMD_SAVEFILENAME_REV_SIZE = 16
#指示运行指定地址的程序
CMD_RUNCODE_SEND_SIZE = 11
CMD_RUNCODE_REV_SIZE  = 11

#cmd
CMD_PING                    = 0x01
CMD_SERVO                   = 0x02
CMD_DUMMY                   = 0x00
CMD_WRITEBUFFER             =0x10
CMD_RAM2FLASH               =0x20
CMD_RAM2FLASH_SUCCESS       =0x21
CMD_RAM2FLASH_FAIL          =0x22
CMD_SAVEFILENAME            =0x30
CMD_SAVEFILENAME_SUCCESS    =0x31
CMD_SAVEFILENAME_FAIL       =0x32
CMD_RUNCODE                 =0x40
CMD_RUNCODE_SUCCESS         =0x41
CMD_RUNCODE_FAIL            =0x42
CMD_CRC                     =0x50
CMD_CRC_SUCCESS             =0x51
CMD_CRC_FAIL                =0x52


CMD_PC_2_M32_HEAD0 = 0x55
CMD_PC_2_M32_HEAD1 = 0xAA
CMD_M32_2_PC_HEAD0 = 0x66
CMD_M32_2_PC_HEAD1 = 0xBB

SerialControl=""

FILE_NOLOAD = "FILENOLOAD"#不加载BIN，只运行程序

def Errlog(msg):
    # f=open("{0}/log/errlog".format(common.base_path),"a")
    f=open("log/errlog","a")
    f.writelines(time.strftime('%Y-%m-%d %H:%M:%S'))
    f.writelines(msg)
    f.writelines("\n")
    f.close()

ScratchMSG=""
def PrintLog(level,msg):
    global ScratchMSG
    if level <= PRINT_LEVEL:
        ScratchMSG = ScratchMSG + msg + "\n"
        print(msg)
    # Errlog(msg)


def PrintHex(level,msg,bytes):
    if level < PRINT_LEVEL:
        print(msg)
        l = [hex(int(i)) for i in bytes]
        print(" ".join(l))










# 定义一个256个元素的全0数组
custom_crc32_table = [0 for x in range(0, 256)]
 
# 一个8位数据加到16位累加器中去，只有累加器的高8位或低8位与数据相作用，
# 其结果仅有256种可能的组合值。
def generate_crc32_table():
    global custom_crc32_table
    for i in range(256):
        c = i << 24
        for j in range(8):
            c = (c << 1) ^ 0x04C11DB7 if (c & 0x80000000) else c << 1
        custom_crc32_table[i] = c & 0xffffffff

def crc32_stm(bytes_arr):
    length = len(bytes_arr)
    k = 0
    crc = 0xffffffff
    while length >= 4:
        v = ((bytes_arr[k] << 24) & 0xFF000000) | ((bytes_arr[k+1] << 16) & 0xFF0000) | \
        ((bytes_arr[k+2] << 8) & 0xFF00) | (bytes_arr[k+3] & 0xFF)
        crc = ((crc << 8) & 0xffffffff) ^ custom_crc32_table[0xFF & ((crc >> 24) ^ v)]
        crc = ((crc << 8) & 0xffffffff) ^ custom_crc32_table[0xFF & ((crc >> 24) ^ (v >> 8))]
        crc = ((crc << 8) & 0xffffffff) ^ custom_crc32_table[0xFF & ((crc >> 24) ^ (v >> 16))]
        crc = ((crc << 8) & 0xffffffff) ^ custom_crc32_table[0xFF & ((crc >> 24) ^ (v >> 24))]
        k += 4
        length -= 4
    if length > 0:
        v = 0
        for i in range(length):
            v |= (bytes_arr[k+i] << 24-i*8)
        if length == 1:
            v &= 0xFF000000
        elif length == 2:
            v &= 0xFFFF0000
        elif length == 3:
            v &= 0xFFFFFF00
        crc = (( crc << 8 ) & 0xffffffff) ^ custom_crc32_table[0xFF & ( (crc >> 24) ^ (v ) )];
        crc = (( crc << 8 ) & 0xffffffff) ^ custom_crc32_table[0xFF & ( (crc >> 24) ^ (v >> 8) )];
        crc = (( crc << 8 ) & 0xffffffff) ^ custom_crc32_table[0xFF & ( (crc >> 24) ^ (v >> 16) )];
        crc = (( crc << 8 ) & 0xffffffff) ^ custom_crc32_table[0xFF & ( (crc >> 24) ^ (v >> 24) )];
    return crc


#生成CRC32表
generate_crc32_table()
#reversal_init_crc32_table()

#计算代码缓存的CRC数值
def CrcCodeArra(codearray_4k):
    return crc32_stm(codearray_4k)








def checksumcode(data):
    sum=0
    codelen=len(data)
    for i in range(codelen):
        sum=sum+data[i]
    sum=(sum & 0xff)
    sum=(~sum)&0xff
    return sum


def ReadBin(filename):
    global code
    try:
        file = open(filename, 'rb')  # 以2进制方式打开bin文件
        romtemp = file.read()
        codelen = len(romtemp)  # 代码长度
        code = bytearray(codelen)  # 重新定义缓存区大小
        for i in range(len(romtemp)):
            code[i] = romtemp[i]
        # PrintLog(1, "Read Code :" + filename + "\n---size: "
        #          + str(int(codelen / 1024)) + " kbyte"
        #          + "  checksum: " + str(checksumcode(code)))
        return True
    except Exception as e:
        PrintLog(1, "Read Code Fail: "  + filename)
        return False




def checksum(data):
    sum=0
    codelen=len(data)-1
    for i in range(codelen):
        sum=sum+data[i]
    sum=(sum & 0xff)
    sum=(~sum)&0xff
    return sum


def PingControl(device):
    control_type=0
    cmd_index=0
    # 打开串口
    try:
        ser=serial.Serial(
            port=device,baudrate=BUAD,timeout=TIMEOUT,bytesize=8, 
            parity=serial.PARITY_NONE,stopbits=1,writeTimeout=TIMEOUT
        )
    except Exception as e:
        PrintLog(1,"PingControl Error:" + str(e))
        try:
            ser.close()
        except Exception as e:
            pass
        PrintLog(1,"Error:PORT is in Use")
        return 0
    senddata=bytearray(CMD_PING_SEND_SIZE)
    senddata[0]=CMD_PC_2_M32_HEAD0
    senddata[1]=CMD_PC_2_M32_HEAD1
    senddata[2]= cmd_index
    senddata[3]= CMD_PING
    datelen=len(senddata)
    senddata[4]=  datelen & 0x000000ff
    senddata[5]= (datelen & 0x0000ff00)>>8
    senddata[6] = CMD_DUMMY
    senddata[7]= checksum(senddata)
    try:

        #ping 放慢，使得控制器可从常规程序中退出
        send=bytearray(1)
        for i in range(datelen):
            send[0]=senddata[i]
            result = ser.write(send)
            time.sleep(0.001)
        #result = ser.write(senddata)

        PrintHex(10,"Ping Send byte :",senddata)
        time.sleep(PING_TIME_DELAY)
        revtemp = ser.read(CMD_PING_REV_SIZE)
        PrintHex(10,"Ping Rev byte :",revtemp)
        if len(revtemp) == CMD_PING_REV_SIZE:
            #接收到下位机反馈
            if revtemp[0] == CMD_M32_2_PC_HEAD0 and revtemp[1]==CMD_M32_2_PC_HEAD1 :
                if revtemp[3] == CMD_PING and revtemp[CMD_PING_REV_SIZE-1] ==  checksum(revtemp):
                    control_type = revtemp[6]  + revtemp[7] *256
                    ser.close()
                    return control_type
        return 0
    except Exception as e:
        PrintLog(1,"Error Ping")
        ser.close()
        return 0




def Ping2ServoControl(device):
    control_type=0
    cmd_index=0
    #打开串口
    try:
        ser=serial.Serial( port=device,baudrate= BUAD,timeout=TIMEOUT,bytesize=8,parity=serial.PARITY_NONE,stopbits=1,writeTimeout=TIMEOUT)
    except Exception as e:
        try:
            ser.close()
        except Exception as e:
            pass
        PrintLog(1,"Error:PORT is in Use")
        return 0
    senddata=bytearray(CMD_PING_SEND_SIZE)
    senddata[0]=CMD_PC_2_M32_HEAD0
    senddata[1]=CMD_PC_2_M32_HEAD1
    senddata[2]= cmd_index
    senddata[3]= CMD_SERVO
    datelen=len(senddata)
    senddata[4]=  datelen & 0x000000ff
    senddata[5]= (datelen & 0x0000ff00)>>8
    senddata[6] = CMD_DUMMY
    senddata[7]= checksum(senddata)
    try:

        #ping 放慢，使得控制器可从常规程序中退出
        send=bytearray(1)
        for i in range(datelen):
            send[0]=senddata[i]
            result = ser.write(send)
            time.sleep(0.001)
        #result = ser.write(senddata)

        PrintHex(10,"Ping Send byte :",senddata)
        time.sleep(PING_TIME_DELAY)
        revtemp = ser.read(CMD_PING_REV_SIZE)
        PrintHex(10,"Ping Rev byte :",revtemp)
        if len(revtemp) == CMD_PING_REV_SIZE:
            #接收到下位机反馈
            if revtemp[0] == CMD_M32_2_PC_HEAD0 and revtemp[1]==CMD_M32_2_PC_HEAD1 :
                if revtemp[3] == CMD_SERVO and revtemp[CMD_PING_REV_SIZE-1] ==  checksum(revtemp):
                    control_type = revtemp[6]  + revtemp[7] *256
                    ser.close()
                    return control_type
        return 0
    except Exception as e:
        PrintLog(1,"Error Ping")
        ser.close()
        return 0

#发送4K数据到STM32缓存区上
def SendCode2M32Buffer(ser,codearray_4k):
    cmd_index=0
    senddata=bytearray(CMD_DW_SEND_SIZE)
    senddata[0]=CMD_PC_2_M32_HEAD0
    senddata[1]=CMD_PC_2_M32_HEAD1
    senddata[2]= cmd_index
    senddata[3]= CMD_WRITEBUFFER
    datelen=len(senddata)
    senddata[4]=  datelen & 0x000000ff
    senddata[5]= (datelen & 0x0000ff00)>>8
    senddata[6] = CMD_DUMMY
    #将数据复制到发送缓存上
    for i in range(len(codearray_4k)):
        senddata[7+i]=codearray_4k[i]
    senddata[datelen-1]= checksum(senddata)
    try:
        result = ser.write(senddata)
        PrintHex(21,"SendCode2M32Buffer :",senddata)
        time.sleep(DW_TIME_DELAY)
        revtemp = ser.read(CMD_DW_REV_SIZE)
        PrintHex(21,"SendCode2M32Buffer :",revtemp)
        if len(revtemp) == CMD_DW_REV_SIZE:
            #接收到下位机反馈
            if revtemp[0] == CMD_M32_2_PC_HEAD0 and revtemp[1]==CMD_M32_2_PC_HEAD1 :
                if revtemp[3] == CMD_WRITEBUFFER and revtemp[CMD_DW_REV_SIZE-1] ==  checksum(revtemp):
                    return True
        return False
    except Exception as e:
        PrintLog(1,"Error Download")
        return False

#将4K Buff烧写到FLASH指定的地址上
def CopyM32Buffer2Flash(ser,flashaddress):
    PrintLog(10, "---Ram to Flash : @ " + hex(flashaddress))
    control_type=0
    cmd_index=0
    #打开串口
    senddata=bytearray(CMD_RAM2FLASH_SEND_SIZE)
    senddata[0] = CMD_PC_2_M32_HEAD0
    senddata[1] = CMD_PC_2_M32_HEAD1
    senddata[2] = cmd_index
    senddata[3] = CMD_RAM2FLASH
    datelen = len(senddata)
    senddata[4] = datelen & 0x000000ff
    senddata[5] = (datelen & 0x0000ff00) >> 8
    senddata[6] = flashaddress & 0x000000ff
    senddata[7] = (flashaddress & 0x0000ff00) >> 8
    senddata[8] = (flashaddress & 0x00ff0000) >> 16
    senddata[9] = (flashaddress & 0xff000000) >> 24
    senddata[datelen-1]= checksum(senddata)
    try:
        result = ser.write(senddata)
        #放慢，使得控制器可从常规程序中退出
        PrintHex(10,"Copy Ram 2 Flash Send :",senddata)
        time.sleep(RAM2FLASH_TIME_DELAY)
        revtemp = ser.read(CMD_RAM2FLASH_REV_SIZE)
        PrintHex(10,"Copy Ram 2 Flash Rev :",revtemp)
        if len(revtemp) == CMD_RAM2FLASH_REV_SIZE:
            #接收到下位机反馈
            if revtemp[0] == CMD_M32_2_PC_HEAD0 and revtemp[1]==CMD_M32_2_PC_HEAD1 :
                if revtemp[CMD_RAM2FLASH_REV_SIZE-1] ==  checksum(revtemp):
                    if senddata[6] == revtemp[6] and senddata[7]==revtemp[7] and senddata[8]==revtemp[8] and senddata[9]==revtemp[9]:
                        if revtemp[3] == CMD_RAM2FLASH_SUCCESS :
                            return True
        return False
    except Exception as e:
        PrintLog(1,"Error Ram2Flash")
        return False
    pass








#获取STM32 FLASH CRC数值
def CrcFlash(ser,flashaddress,codearray_4k_crc):
    #PrintLog(20, "---Crc Flash : @ " + hex(flashaddress))
    control_type=0
    cmd_index=0
    #打开串口
    senddata=bytearray(CMD_RAM2FLASH_SEND_SIZE)
    senddata[0] = CMD_PC_2_M32_HEAD0
    senddata[1] = CMD_PC_2_M32_HEAD1
    senddata[2] = cmd_index
    senddata[3] = CMD_CRC
    datelen = len(senddata)
    senddata[4] = datelen & 0x000000ff
    senddata[5] = (datelen & 0x0000ff00) >> 8
    senddata[6] = flashaddress & 0x000000ff
    senddata[7] = (flashaddress & 0x0000ff00) >> 8
    senddata[8] = (flashaddress & 0x00ff0000) >> 16
    senddata[9] = (flashaddress & 0xff000000) >> 24
    senddata[datelen-1]= checksum(senddata)
    
    try:
        result = ser.write(senddata)
        PrintHex(10,"Crc Flash Send :",senddata)
        time.sleep(CRC_TIME_DELAY)
        revtemp = ser.read(CMD_RAM2FLASH_REV_SIZE)
        PrintHex(10,"Crc Flash Rev :",revtemp)
        if len(revtemp) == CMD_RAM2FLASH_REV_SIZE:
            #接收到下位机反馈
            if revtemp[0] == CMD_M32_2_PC_HEAD0 and revtemp[1]==CMD_M32_2_PC_HEAD1 :
                if revtemp[CMD_RAM2FLASH_REV_SIZE-1] ==  checksum(revtemp):
                    revcrc=int(revtemp[6]) + int(revtemp[7])*0x100 + int(revtemp[8])*0x10000+int(revtemp[9])*0x1000000
                    #PrintLog(5,"---Crc Flash = :" +str(hex(revcrc)) + " Crc Code=:" +str(hex(codearray_4k_crc)))
                    if  revcrc == codearray_4k_crc:#crc校验相等，则返回true
                        #PrintLog(5,"---Resume Ram to Flash")
                        PrintLog(5,"---Crc Same,Resume: Flash " +str(hex(revcrc)) + " = Code " +str(hex(codearray_4k_crc)))
                        return True
        return False
    except Exception as e:
        PrintLog(1,"Error Crc Flash")
        return False
    pass



#将文件名保存到STM32上
def SaveNameToStm32(PORTNAME,filename):
    cmd_index=0
    #filename=os.path.basename(filename)#将文件路径转换为纯文件名
    filename=filename.split('.')[0]#只保留文件名，不需要后缀
    PrintLog(10, "---Save Name To Flash :  " + filename)
    filenamearray=bytes(filename, 'utf-8')
    
    #打开串口
    ser = serial.Serial(port=PORTNAME, baudrate=BUAD, timeout=TIMEOUT, bytesize=8, parity=serial.PARITY_NONE,
                        stopbits=1, writeTimeout=TIMEOUT)
    senddata=bytearray(CMD_SAVEFILENAME_SEND_SIZE)
    senddata[0] = CMD_PC_2_M32_HEAD0
    senddata[1] = CMD_PC_2_M32_HEAD1
    senddata[2] = cmd_index
    senddata[3] = CMD_SAVEFILENAME
    datelen = len(senddata)
    senddata[4] = datelen & 0x000000ff
    senddata[5] = (datelen & 0x0000ff00) >> 8
    #文件名
    for i in range(8):
        if i < len(filenamearray):
            senddata[6+i] = filenamearray[0+i]
        else:
            senddata[6 + i] = 0
    #校验码
    senddata[datelen-1]= checksum(senddata)
    try:
        #result = ser.write(senddata)
        #放慢，使得控制器可从常规程序中退出
        send=bytearray(1)
        for i in range(datelen):
            send[0]=senddata[i]
            result = ser.write(send)
            time.sleep(0.001)
        PrintHex(10,"Save Name To Flash Send:",senddata)
        time.sleep(RAM2FLASH_TIME_DELAY)
        revtemp = ser.read(CMD_SAVEFILENAME_REV_SIZE)
        PrintHex(10,"Save Name To Flash Rev:",revtemp)
        if len(revtemp) == CMD_SAVEFILENAME_REV_SIZE:
            #接收到下位机反馈
            if revtemp[0] == CMD_M32_2_PC_HEAD0 and revtemp[1]==CMD_M32_2_PC_HEAD1 :
                if revtemp[CMD_SAVEFILENAME_REV_SIZE-1] ==  checksum(revtemp):
                    if senddata[6] == revtemp[6] and senddata[7]==revtemp[7] and senddata[8]==revtemp[8] and senddata[9]==revtemp[9]:
                        if revtemp[3] == CMD_SAVEFILENAME_SUCCESS :
                            ser.close()
                            return True
        return False
    except Exception as e:
        ser.close()
        PrintLog(1,"Error SaveNameToStm32")
        return False
    pass



#将文件名保存到STM32上
def RunCode(PORTNAME,flashaddress):
    PrintLog(4, "---RunCode : @ " + hex(flashaddress))
    ser = serial.Serial(port=PORTNAME, baudrate=BUAD, timeout=TIMEOUT, bytesize=8, parity=serial.PARITY_NONE,
                        stopbits=1, writeTimeout=TIMEOUT)
    control_type = 0
    cmd_index = 0
    # 打开串口
    senddata = bytearray(CMD_RUNCODE_SEND_SIZE)
    senddata[0] = CMD_PC_2_M32_HEAD0
    senddata[1] = CMD_PC_2_M32_HEAD1
    senddata[2] = cmd_index
    senddata[3] = CMD_RUNCODE
    datelen = len(senddata)
    senddata[4] = datelen & 0x000000ff
    senddata[5] = (datelen & 0x0000ff00) >> 8
    senddata[6] = flashaddress & 0x000000ff
    senddata[7] = (flashaddress & 0x0000ff00) >> 8
    senddata[8] = (flashaddress & 0x00ff0000) >> 16
    senddata[9] = (flashaddress & 0xff000000) >> 24
    senddata[datelen - 1] = checksum(senddata)
    try:
        for times in range(10):
            #result = ser.write(senddata)
            send=bytearray(1)
            for i in range(datelen):
                send[0]=senddata[i]
                result = ser.write(send)
                time.sleep(0.001)
            PrintHex(10, "RunCode Send :", senddata)
            time.sleep(RUNCODE_TIME_DELAY)
            revtemp = ser.read(CMD_RUNCODE_REV_SIZE)
            PrintHex(10, "RunCode Rev :", revtemp)
            if len(revtemp) == CMD_RUNCODE_REV_SIZE:
                # 接收到下位机反馈
                if revtemp[0] == CMD_M32_2_PC_HEAD0 and revtemp[1] == CMD_M32_2_PC_HEAD1:
                    if revtemp[CMD_RUNCODE_REV_SIZE - 1] == checksum(revtemp):
                        if senddata[6] == revtemp[6] and senddata[7] == revtemp[7] and senddata[8] == revtemp[8] and senddata[9] == revtemp[9]:
                            if revtemp[3] == CMD_RUNCODE_SUCCESS:
                                ser.close()
                                PrintLog(4, "---RunCode : ok")
                                return True
        PrintLog(4, "Error RunCode : No Rev data")
        return False
    except Exception as e:
        ser.close()
        PrintLog(1, "Error RunCode")
        return False
    pass



#上层qt回调接口
def QT_DowloadCallbackDefault(process):
    #print("QT_DowloadCallback :", str(process))
    pass
#回调全局变量
qtcallbackCall=QT_DowloadCallbackDefault


def DownCallBack(process):
    pre=int(process)
    if pre > 100:
        pre=100
    qtcallbackCall(pre)


def Download(PORTNAME,ADDRESS):
    global code
    PrintLog(1, "Download Control @ " + hex(ADDRESS) + " ...")
    SuccessState=False
    SEND_TIMES=5#如果数据错误，重新传输的次数
    #打开串口
    ser=serial.Serial( port=PORTNAME,baudrate= BUAD,timeout=TIMEOUT,bytesize=8,parity=serial.PARITY_NONE,stopbits=1,writeTimeout=TIMEOUT)
    senddata = bytearray(CMD_DW_SEND_SIZE)
    addressoffset=0
    DownCallBack(10)#打开串口，则10%
    while True:
        PrintLog(4, "---Download : " + hex(ADDRESS + addressoffset) +  " " +  str(int(addressoffset/1024))  + "k/" + str(int(len(code)/1024)) + "k")
        DownCallBack( 10+addressoffset/len(code) *100)#打开串口，则10%
        codebuffer=bytearray(CMD_BUFFER_4K_LEN)
        for i in range(CMD_BUFFER_4K_LEN):
            if addressoffset+i < len(code):
                codebuffer[i] = code[addressoffset+i]
            else:#如果代码小于4K，则补0xff
                codebuffer[i] =0xff
        SuccessState=False
        #先行判断当前代码CRC数据是否正确，正确情况下跳过当前下载
        if CrcFlash(ser,ADDRESS + addressoffset,CrcCodeArra(codebuffer)) == False:
            for i in range(SEND_TIMES):
                SuccessState = SendCode2M32Buffer(ser,codebuffer)
                if  SuccessState == True:#船速4K数据到控制器上,传输成功
                    break
            if SuccessState == True:#传输成功
                for i in range(SEND_TIMES):#尝试5次
                    SuccessState = CopyM32Buffer2Flash(ser, ADDRESS + addressoffset)
                    if SuccessState == True:
                        break
            if SuccessState == False:#传输有错误
                break
        else:
            SuccessState=True
        addressoffset = addressoffset + CMD_BUFFER_4K_LEN
        if(addressoffset > len(code)):
            break
    if SuccessState == True:
        ser.close()
        PrintLog(1, "---Download Success")
        return True
    else:
        ser.close()
        PrintLog(1, "---Download Fail")
        return False



#连接控制器
def ConnectControl():
    i=0
    PrintLog(1,"Ping Control...")
    port_list = list(serial.tools.list_ports.comports())#获取可用串口
    if(len(port_list) > 0):
        for i in range(0, len(port_list)):#遍历所有串口
            PrintLog(1,"---Com : "  + port_list[i].device + " PID : " + str(port_list[i].pid) + " VID : " + str(port_list[i].vid))
            if port_list[i].pid == PID and port_list[i].vid == VID:#ch34使用的是USB转串口，故带有VID和PID数据
                #控制器使用的串口
                for repeat in range(5):#range(5):#ping 控制器 ，5次
                    control_type = PingControl(port_list[i].device)
                    time.sleep(0.1)#等待50ms               
                    if control_type > 0:#成功PING到控制器
                        PrintLog(1,"---Success: COM = " + port_list[i].device + "  Control_type = " + str(control_type))
                        return (port_list[i].device,control_type)#返回串口号和控制器类型
    PrintLog(1, "---Fail: No Control")
    return ("",0)




#连接控制器，并进入伺服电机调试状态
def ConnectControl2Servoa():
    i=0
    PrintLog(1,"Ping and Servo Control...")
    port_list = list(serial.tools.list_ports.comports())#获取可用串口
    if(len(port_list) > 0):
        for i in range(0, len(port_list)):#遍历所有串口
            PrintLog(1,"---Com : "  + port_list[i].device + " PID : " + str(port_list[i].pid) + " VID : " + str(port_list[i].vid))
            if port_list[i].pid == PID and port_list[i].vid == VID:#ch34使用的是USB转串口，故带有VID和PID数据
                #控制器使用的串口
                for repeat in range(5):#range(5):#ping 控制器 ，5次
                    time.sleep(0.05)#等待50ms
                    for count in range(3):#需要PING3次
                        control_type = Ping2ServoControl(port_list[i].device)   
                        time.sleep(0.05)#等待50ms               
                        if control_type > 0:#成功PING到控制器
                            PrintLog(1,"---Success: COM = " + port_list[i].device + "  Control_type = " + str(control_type))
                            return (port_list[i].device,control_type)#返回串口号和控制器类型
    PrintLog(1, "---Fail: No Control")
    return ("",0)



def main(filename,DownloadAddress,run,runname="RunA"):
    if filename != FILE_NOLOAD:#空文件名表示不下载
        print(filename)
        if ReadBin(filename) == False:  # 读取文件错误
            return False
    for i in range(10):
        COM,Control_type=ConnectControl()
        if COM != "" :
            break
        time.sleep(0.5)
    if COM == "":#未找到控制器
        return False,Control_type
    if filename != FILE_NOLOAD:  # 空文件名表示不下载
        if Download(COM,DownloadAddress) == False:#下载错误
            return False,Control_type
        else:#下载成功
            #根据地址判断，是否需要更新程序文件名到stm32控制器上
            if DownloadAddress == PROGRAM_GCC:#只有GCC程序需要更新文件名到STM32上
                if SaveNameToStm32(COM,runname) == False:#发生错误
                    return False,Control_type
    if run == True:#如果需要运行程序
        if RunCode(COM,DownloadAddress) == False:
            return False,Control_type
    return True,Control_type

#将控制器返回编号转换为控制器型号
def ControlNum2String(controltypenum):
    if controltypenum == HARD_902:
        return DEVICE_NAME_902
    if controltypenum == HARD_102:
        return DEVICE_NAME_102
    if controltypenum == HARD_602:
        return DEVICE_NAME_602
    return DEVICE_NAME__UNKNOW


#对SCRATCH接口
#默认下载并运行程序
#参数 ：callback回调进度的函数
#返回 ：result,ScratchMSG
#result（bool)：，True/False
#ScratchMSG(string)：返回下载log
def Scratch_Download(RunName="RunA",qtcallback=QT_DowloadCallbackDefault):
    global ScratchMSG
    global qtcallbackCall
    #下载GUI用户界面程序********************************************
    # filename="{0}/AI_Module_GCC/MC_902P/AT32Gcc/Debug/Run.bin".format(common.base_path)
    filename="/AI_Module_GCC/MC_902P/AT32Gcc/Debug/Run.bin"
    address=PROGRAM_GCC
    run=True
    ScratchMSG=""
    qtcallbackCall=qtcallback
    result,controltype = main(filename,address,run,runname=RunName)
    return result,ScratchMSG

#测试Scratch_Download函数
'''
result1,msg=Scratch_Download()
print("result1="+str(result1))
print("msg="+msg)
'''



#对SCRATCH接口
#默认下载并运行程序
#参数 ：callback回调进度的函数
#返回 ：result,ScratchMSG
#result（bool)：，True/False
#ScratchMSG(string)：返回下载log
def Scratch_Download_MC602P(RunName="RunA",isrun=False,qtcallback=QT_DowloadCallbackDefault):
    global ScratchMSG
    global qtcallbackCall
    #下载GUI用户界面程序********************************************
    dir_now = os.path.abspath(os.path.join(os.path.dirname(__file__)))
    file=os.path.join(dir_now,"Run.bin")
    #如果直接运行，直接下载到GCC运行区间
    if isrun == True:
        RunName="debug"
        address=PROGRAM_GCC
        run=isrun#下载到程序中，需要控制器按钮运行
    else:
        #602P可下载6个程序
        run=isrun#下载到程序中，需要控制器按钮运行
        if RunName == "RunA":
            address=MC602_APP1_ADDRESS
        elif RunName == "RunB":
            address=MC602_APP2_ADDRESS
        elif RunName == "RunC":
            address=MC602_APP3_ADDRESS
        elif RunName == "RunD":
            address=MC602_APP4_ADDRESS
        elif RunName == "RunE":
            address=MC602_APP5_ADDRESS
        elif RunName == "RunF":
            address=MC602_APP6_ADDRESS    
    ScratchMSG=""
    qtcallbackCall=qtcallback
    result,controltype = main(file,address,run,runname=RunName)
    return result,ScratchMSG

#测试Scratch_Download函数
'''
result1,msg=Scratch_Download_MC602P(RunName="RunF",isrun=True)
print("result1="+str(result1))
print("msg="+msg)
'''



#对SCRATCH接口
#获取设备名称
#返回 ：result,ScratchMSG
#result(string):返回硬件名称： MC_902P / MC_102P
#ScratchMSG(string):返回log
def Scratch_DeviceInfo():
    global ScratchMSG
    result=DEVICE_NAME__UNKNOW
    ScratchMSG=""
    comport,control=ConnectControl()
    result = ControlNum2String(control)
    return result,ScratchMSG

#测试Scratch_DeviceInfo函数
'''
result1,msg=Scratch_DeviceInfo()
print("result1="+result1)
print("msg=" + msg)
'''

#对SCRATCH接口
#开启U盘
#返回 ：result,controlype,ScratchMSG
#result(bool):返回U盘是否开启
#controlype:控制器类型102，902
#ScratchMSG(string):返回log
def Scratch_USBMass():
    global ScratchMSG
    file=FILE_NOLOAD
    address=PROGRAM_USBMASS
    run=True
    ScratchMSG=""
    result,controltype = main(file,address,run)
    return result,ControlNum2String(controltype),ScratchMSG
'''
#测试Scratch_USBMass函数
result1,controltype,msg=Scratch_USBMass()
print("result1="+str(result1)+" "+ controltype)
print("msg=" + msg)
'''


#对SCRATCH接口,指示控制器进入
#获取设备所在串口，设备名称
#返回 ：comport,result
#：comport(string):返回硬件名称： 连接的串口编号
#result(string):返回硬件名称： MC_902P / MC_102P
def Scratch_Ping2Servo():
    global ScratchMSG
    control,msg = Scratch_DeviceInfo()
    if control == DEVICE_NAME__UNKNOW :
        return "",""
    time.sleep(3.0)#等待3.0秒，控制器退出下载状态 
    result=DEVICE_NAME__UNKNOW
    ScratchMSG=""
    comport,control=ConnectControl2Servoa()
    result = ControlNum2String(control)
    if comport == "" :
        comport=""
    return comport,result

#测试Scratch_Ping函数
'''
comport,result1=Scratch_Ping2Servo()
print("com="+ comport + " result1="+result1)

'''



if __name__== "__main__" :
    parser = argparse.ArgumentParser()  # 创建ArgumentParser对象
    #下载GCC文件,并运行此GCC程序****************************************************************************************

    #parser.add_argument('-file', default="D:\work\AT32Project\AT32Gcc\AT32Gcc\Debug\Run.bin")
    '''
    
    parser.add_argument('-file', default="Run.bin")
    parser.add_argument('-address' ,  default="0x0800D000")
    parser.add_argument('-run', default="true")
    '''
    #下载声音文件，此处是下载到外部FLASH地址上***************************************************
    '''
    parser.add_argument('-file', default="Voice_ROM.bin")
    parser.add_argument('-address' ,  default="0x300000")
    parser.add_argument('-run', default="false")
    '''
    #下载GUI用户界面程序********************************************
    #parser.add_argument('-file', default="..\AT32Gcc\Debug\Run.bin")
    '''
    parser.add_argument('-file', default="{0}/AI_Module_GCC/{1}/AT32Gcc/Debug/Run.bin".format(common.base_path, CPDIR))
    parser.add_argument('-address', default="0x08040000")
    parser.add_argument('-run', default="true")
    '''
    
    #不下载程序，直接运行一个程序，这里给的地址是U盘驱动所在地址
    '''
    parser.add_argument('-file', default=FILE_NOLOAD)
    parser.add_argument('-address', default="0x08060000")
    parser.add_argument('-run', default="true")
    '''
    #***********************************************************
    # parser.add_argument('-msg', type=int, default=PRINT_LEVEL)
    # args = parser.parse_args()
    # #地址处理
    # address=int(args.address, 16)
    # #输出信息处理
    # PRINT_LEVEL = args.msg
    # #是否运行程序的处理
    # if args.run == "false":
    #     run=False
    # if args.run == "true":
    #     run=True
    # main(args.file,address,run)
    Scratch_Download_MC602P("RunA", isrun=True)
    #Scratch_Download()
    #Scratch_Update()