''' 
   Script for reading detailed output from vectornav 300
   GPS/INS unit
'''
import serial
import time

def debug_on(func):
    def debugging(*args,**kwargs):
        retval = func(*args,**kwargs);
        print('Scope: debugging %s:%s:%s'%(func,func.__name__,retval));
        return retval
    return debugging

class gps():
    def __init__(self,dev,baudrate=921600,timeout=1):
        self.ser = serial.Serial(dev,timeout=timeout,
                                 baudrate=baudrate, bytesize=8,
                                 parity='N', stopbits=1)
        self.dev = dev
        self.cmd_mode=False
        if(self.dev == self.ser.name):
            print('device configured correctly')
        else:
            print('could not be configured correctly')
    @debug_on
    def reset(self,response):
        val = self.command(b"$VNRST",response)
        return val;
    @debug_on
    def async_off(self):
        val = self.command("$VNWRG,6,0")
        return val;
    @debug_on
    def switch_to_gps_co(self):
        val = self.command("$VNDBS,3,1")
        return val;
    @debug_on
    def reset_factory_settings(self):
        val = self.command("$VNRFS")
    def cmd_mode_tog(self,val):
        if(val):
            self.command("$VNCMD")
        else:
            self.command("exit")
    @debug_on
    def gps_meas(self):
        self.cmd_mode_tog(True)
        val=self.command('gps meas',size=10000)
        self.cmd_mode_tog(False)
        return val;
    def command(self,command,response=True,size=16):
        command+=self.calculate_checksum(command);
        self.ser.write(command+"\r\n")
        if(response):
            time.sleep(1)
            return self.ser.read(5000)
    def calculate_checksum(self,command):
        sum=0;
        for i in command:
            if( i=="$" ):
                continue;
            if( i=="*" ):
                break;
            sum^=ord(i);
        return '*'+hex(sum).split('x')[1];
    def write_register(self,reg_num,arg=''):
        if( arg != ''):
            return self.command('$VNWRG,%d,%d'%(reg_num,arg));
        else:
            return self.command('$VNRRG,%d'%reg_num);

def main():
    g = gps(dev='/dev/vectornav')
    g.cmd_mode_tog(False)
    g.async_off()
    g.switch_to_gps_co()
    #for k in range(0,1):
    #    g.reset_factory_settings()
#    g.reset(True);
    g.async_off();

    while(True):
        g.async_off();
        g.gps_meas();

if __name__=="__main__":
    main()
