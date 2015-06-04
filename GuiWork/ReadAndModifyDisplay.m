function ReadAndModifyDisplay(serialObject)

persistent RawDataBuf;

if isempty(RawDataBuf)
    RawDataBuf=zeros(10000,1);
end

persistent writePointer;
persistent readPointer;

if isempty(writePointer)
    writePointer=1;
end

if isempty(readPointer)
    readPointer=1;
end

if(serialObject.BytesAvailable~=0)
    [RecBuf, count]=fread(serialObject,serialObject.BytesAvailable,'uint8');
    if((writePointer+count-1)>10000)
        RawDataBuf(writePointer:10000)=RecBuf(1:end-(writePointer+count-1-10000));
        RawDataBuf(1:(writePointer+count-1-10000))=RecBuf(end-(writePointer+count-10000):end);
        writePointer=(writePointer+count-10000);
    else
        RawDataBuf(writePointer:writePointer+count-1)=RecBuf;
        writePointer=writePointer+count;
    end
end

if(writePointer-readPointer > 140)
    
    if(uint8(RawDataBuf(readPointer))==170 && uint8(RawDataBuf(mod((readPointer+1),10000)))==171 && uint8(RawDataBuf(mod((readPointer+2),10000)))==187)
       readPointer=mod((readPointer+1),10000);
    else           
       Flight_Data.R(k)=bytes2engr(RawDataBuf(readPointer+3:readPointer+4),'short_nb')*0.02;
       Flight_Data.Q(k)=bytes2engr(RawDataBuf(readPointer+5:readPointer+6),'short_nb')*0.02;
       Flight_Data.Ax(k)=bytes2engr(RawDataBuf(readPointer+7:readPointer+8),'short_nb')*0.00025;
       Flight_Data.Ay(k)=bytes2engr(RawDataBuf(readPointer+9:readPointer+10),'short_nb')*0.00025;
       Flight_Data.Az(k)=bytes2engr(RawDataBuf(readPointer+11:readPointer+12),'short_nb')*0.00025;
       
       Flight_Data.NB_Counter(k)=RawDataBuf(readPointer+139);  
       
       GPS.x(k)=bytes2engr(RawDataBuf(readPointer+34:-1:start+27),'double');
       GPS.y(k)=bytes2engr(RawDataBuf(readPointer+42:-1:readPointer+35),'double');
       GPS.z(k)=bytes2engr(RawDataBuf(readPointer+50:-1:readPointer+43),'double');
        
       GPS.vx(k)=bytes2engr(RawDataBuf(readPointer+58:-1:readPointer+51),'double');
       GPS.vy(k)=bytes2engr(RawDataBuf(readPointer+66:-1:readPointer+59),'double');
       GPS.vz(k)=bytes2engr(RawDataBuf(readPointer+74:-1:readPointer+67),'double');
         
       GPS.week(k)=bytes2engr(RawDataBuf(readPointer+96:readPointer+97),'ushort');
       GPS.ms(k)=bytes2engr(RawDataBuf(readPointer+98:readPointer+101),'ulong');       
       
       readPointer=mod((readPointer+140),10000);
    end
end
        

  
        
end