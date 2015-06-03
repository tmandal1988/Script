exit=0;
delayValString={'40','60','80','100','200','300','400','500','Remove All Delays','Exit'};

totaldelay=0;

while (exit==0);
    heading=strcat('Select Delay (in ms) to send, Current Delay = ',num2str(totaldelay),' :');
    DelayVal=menu( heading,'40','60','80','100','200','300','400','500','Remove All Delays','Exit');
    
    if strcmp(delayValString(DelayVal),'Remove All Delays')
        totaldelay=0;
        i=1;
        for i=1:5
            pause(0.01)
        
        end
    end
    
    if strcmp(delayValString(DelayVal),'Exit')
        exit=1;
        i=1;
        totaldelay=0;
        for i=1:5
            pause(0.01)
           
        end
    end
    
    if strcmp(delayValString(DelayVal),'40')
        totaldelay=40;
        i=1;
        for i=1:5
            pause(0.01)
            
        end
    end
    
    if strcmp(delayValString(DelayVal),'60')
        totaldelay=60;
        i=1;
        for i=1:5
            pause(0.01)
            
        end
    end
    
    if strcmp(delayValString(DelayVal),'80')
        totaldelay=80;
        i=1;
        for i=1:5
            pause(0.01)
            
        end
    end
    
    if strcmp(delayValString(DelayVal),'100')
        totaldelay=100;
        i=1;
        for i=1:5
            pause(0.01)
            
        end
    end
    
    if strcmp(delayValString(DelayVal),'200')
        totaldelay=200;
        i=1;
        for i=1:5
            pause(0.01)
            
        end
    end
    
    if strcmp(delayValString(DelayVal),'300')
        totaldelay=300;
        i=1;
        for i=1:5
            pause(0.01)
           
        end
    end
    
    if strcmp(delayValString(DelayVal),'400')
        totaldelay=400;
        i=1;
        for i=1:5
            pause(0.01)
            
        end
    end
    
    if strcmp(delayValString(DelayVal),'500')
        totaldelay=500;
        i=1;
        for i=1:5
            pause(0.01)
            
        end
    end
    
end


