#!/usr/bin/python3
# -*- coding: UTF-8 -*-
# RMS-S servo system driver
import serial,traceback,time
import serial.tools.list_ports

now_int=lambda:int(time.time())
now_str=lambda:time.strftime("%Y-%m-%d %H:%M",time.localtime())

class Servo():
    LOGFILE="Servo.log"
    
    HEAD=b'\x3E'
    CMD={'readAngle':b'\x90',
         'OLPower':b'\xA0',#open-loop run with specific power
         'Speed':b'\xA2',
         'Posi':b'\xA3',#multi-cycle position
         'Posi_MaxSpd':b'\xA4',#multi-cycle position
         'Posi_Dir':b'\xA5',#single-cycle position
         'Posi_Spd_Dir':b'\xA6',#single-cycle position
         }
    port_list=[i.name for i in list(serial.tools.list_ports.comports())]
    print("port list:%s"%(port_list))
    usb=serial.Serial('/dev/ttyUSB0',115200)
    print("usb.is_open:%s"%(usb.is_open))

    def log(msg,l=-1):
        st=traceback.extract_stack()[-2]
        if l>=3:
            tempstr="%s<%s:%d> %s\n%s"%(now_str(),st.name,st.lineno,msg,traceback.formatexec(limit=1))
        else:
            tempstr="%s<%s:%d> %s\n"%(now_str(),st.name,st.lineno,msg)
        if l>=0:
            print(tempstr,end="")
        if l>=1:
            with open(Servo.LOGFILE,"a") as f:
                f.write(tempstr)
    
    def calc_ck(b): #calculate checksum
        return bytes((sum([i for i in b])%256,))            
    
    def check_recv(cmd,sid,datalen,recv):
        if len(recv)!=6+datalen: #tested
            Servo.log("recv len wrong: %s should have len of %d"%(recv.hex(),6+datalen),l=2)
            return -3
        hd=Servo.HEAD+Servo.CMD[cmd]+bytes((sid,datalen))
        ck=Servo.calc_ck(hd)
        if recv[0:5]!=hd+ck: #tested
            Servo.log("recv head ck wrong: %s should have head %s"%(recv.hex(),(hd+ck).hex()),l=2)
            return -1
        dck=Servo.calc_ck(recv[5:5+datalen])
        if recv[-1]!=dck[0]: #tested
            Servo.log("recv data ck wrong: %s should have dck %s"%(recv.hex(),dck.hex()),l=2)
            return -2
        return 0
    
    def gen_hd(cmd,sid,datalen):
        hd=Servo.HEAD+Servo.CMD[cmd]+bytes((sid,datalen))
        ck=Servo.calc_ck(hd)
        rhd=hd+ck
        Servo.log("get head: %s"%(rhd.hex()))
        return rhd
        
    def read_angle(sid):
        hd=Servo.gen_hd('readAngle',sid,0)
        
        Servo.usb.flushInput()
        Servo.usb.write(hd)
        
        recv=Servo.usb.read(8)
        Servo.log("get reply: %s"%(recv.hex()))
        if Servo.check_recv('readAngle',sid,2,recv)!=0:
            return -1

        count=int.from_bytes(recv[5:7],'little') #0-4095
        angle=(count*360)/4096.0
        Servo.log("get encoder count and angle: %4d, %f"%(count,angle))
        #print(angle)
        if angle!=0:
            angle=360.0-angle
        return angle
    
    def olpower(sid,power):
        hd=Servo.gen_hd('OLPower',sid,2)
        if 0<=power:
            dt=power.to_bytes(2,'little')
        else:
            dt=(65536+power).to_bytes(2,'little')
        dck=Servo.calc_ck(dt)
        Servo.log("get data: %s"%((dt+dck).hex()))
        
        Servo.usb.flushInput()
        Servo.usb.write(hd+dt+dck)
        
        recv=Servo.usb.read(8)
        Servo.log("recv %s"%(recv.hex()))
        if Servo.check_recv('OLPower',sid,2,recv)!=0:
            return -1
    
        count=int.from_bytes(recv[5:7],'little') #0-4095
        angle=(count*360)/4096.0
        Servo.log("get encoder count and angle: %4d, %f"%(count,angle))
        return angle
        
    def ang_a4(sid,ang,spd):
        if ang!=0:
            ang=360.0-ang
        ang=int(ang*100)
        assert 0<=ang<36000,"ang vaule error"
        spd=int(spd*100)    
        assert 0<=spd<36000,"apeed value error" 
        hd=Servo.gen_hd('Posi_MaxSpd',sid,12)
        dt=ang.to_bytes(8,'little')+spd.to_bytes(4,'little')
        dck=Servo.calc_ck(dt)
        Servo.log("get data: %s"%((dt+dck).hex()))
        
        Servo.usb.flushInput()
        Servo.usb.write(hd+dt+dck)
        
        recv=Servo.usb.read(8)
        Servo.log("recv %s"%(recv.hex()))
        if Servo.check_recv('Posi_MaxSpd',sid,2,recv)!=0:
            return -1
    
        count=int.from_bytes(recv[5:7],'little') #0-4095
        angle=(count*360)/4096.0
        Servo.log("get encoder count and angle: %4d, %f"%(count,angle))
        return angle
        
    def goto_angle(sid,ang,spd,dirc=1):
        #for s2 spd<270 is ok 
        #for s1 spd
        if ang!=0:
            ang=360.0-ang
        ang=int(ang*100)
        assert 0<=ang<36000,"ang vaule error"
        spd=int(spd*100)    
        assert 0<=spd<36000,"apeed value error" 
        
        hd=Servo.gen_hd('Posi_Spd_Dir',sid,8)
        dt=bytes((dirc,))+ang.to_bytes(3,'little')+spd.to_bytes(4,'little')
        dck=Servo.calc_ck(dt)
        Servo.log("get data: %s"%((dt+dck).hex()))
        
        Servo.usb.flushInput()
        Servo.usb.write(hd+dt+dck)
        
        recv=Servo.usb.read(8)
        Servo.log("recv %s"%(recv.hex()))
        if Servo.check_recv('Posi_Spd_Dir',sid,2,recv)!=0:
            return -1
    
        count=int.from_bytes(recv[5:7],'little') #0-4095
        angle=(count*360)/4096.0
        Servo.log("get encoder count and angle: %4d, %f"%(count,angle))
        return angle
        
if __name__=="__main__":
    Servo.ang_a4(1,0,300)
    #Servo.goto_angle(1,270,90,dirc=0)
    #Servo.olpower(1,0)
    #time.sleep(0.5)
    #Servo.goto_angle(1,0,40,dirc=0)
    #Servo.olpower(2,0)
    #while True:
    #    ang=eval(input())
    #    Servo.goto_angle(1,ang,90,dirc=0)
    pass


   
