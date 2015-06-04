function varargout = exampleAH(varargin)
% EXAMPLEAH MATLAB code for exampleAH.fig
%      EXAMPLEAH, by itself, creates a new EXAMPLEAH or raises the existing
%      singleton*.
%
%      H = EXAMPLEAH returns the handle to a new EXAMPLEAH or the handle to
%      the existing singleton*.
%
%      EXAMPLEAH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in EXAMPLEAH.M with the given input arguments.
%
%      EXAMPLEAH('Property','Value',...) creates a new EXAMPLEAH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before exampleAH_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to exampleAH_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help exampleAH

% Last Modified by GUIDE v2.5 04-Jun-2015 17:18:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @exampleAH_OpeningFcn, ...
                   'gui_OutputFcn',  @exampleAH_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before exampleAH is made visible.
function exampleAH_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to exampleAH (see VARARGIN)



% Choose default command line output for exampleAH
handles.output = hObject;
fclose(instrfind); %close any open serial ports
set(handles.connect,'Enable','off')
set(handles.send,'Enable','off')
set(handles.startDisplay,'Enable','off')
set(handles.stopDisplay,'Enable','off')
set(handles.delay,'Enable','off')
set(handles.rate,'Enable','off')
set(handles.trim,'Enable','off')
set(handles.delay,'String','0')
set(handles.rate,'String','0')
set(handles.trim,'String','0')
handles.delayval=0;
handles.rateval=0;
handles.trimval=0;

handles.timer = timer(...
    'ExecutionMode', 'fixedRate', ...   % Run timer repeatedly
    'Period', 0.001, ...                % Initial period is 1 sec.
    'TimerFcn', {@update_display,hObject}); % Specify callback


set(handles.LoopBackDisplay,'String','Delay=0 ms, Rate Limit=0 deg/s, Trim=0 deg')



ArtificialHorizonInit(handles.AHpanel);



handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes exampleAH wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = exampleAH_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function delay_Callback(hObject, eventdata, handles)
% hObject    handle to delay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.delayval=str2double(get(hObject,'String'));

if(isnan(handles.delayval) || handles.delayval<10 || handles.delayval>800)
    display('Please for the love of good enter a valid value between 0 and 100')
    handles.delayval=0;
    set(hObject,'String','0')
end

% Update handles structure
guidata(hObject, handles);


% Hints: get(hObject,'String') returns contents of delay as text
%        str2double(get(hObject,'String')) returns contents of delay as a double


% --- Executes during object creation, after setting all properties.
function delay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in send.
function send_Callback(hObject, eventdata, handles)
% hObject    handle to send (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

persistent SendBuf;
SendBuf=[170 171 187 0 0 0 0 0 0 0];

dataBuf=typecast(int16(handles.delayval*10),'int8');
SendBuf(4)=dataBuf(1);
SendBuf(5)=dataBuf(2);
dataBuf=typecast(int16(handles.rateval*100),'int8');
SendBuf(6)=dataBuf(1);
SendBuf(7)=dataBuf(2);
dataBuf=typecast(int16(handles.trimval*100),'int8');
SendBuf(8)=dataBuf(1);
SendBuf(9)=dataBuf(2);
SendBuf(10)=bitxor(uint8(SendBuf(4)+SendBuf(5)+SendBuf(6)+SendBuf(7)+SendBuf(8)+SendBuf(9)),255);
%display(SendBuf)

set(handles.send,'Enable','off')

for i=1:5
    pause(0.001)
   fwrite(handles.serialObject,SendBuf,'uint8');
end

set(handles.send,'Enable','on')

guidata(hObject, handles);



% --- Executes on button press in serialports.
function serialports_Callback(hObject, eventdata, handles)
% hObject    handle to serialports (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


WhichSerial=instrhwinfo('serial');
numSerialPorts=length(WhichSerial.AvailableSerialPorts);

if(numSerialPorts==0 || numSerialPorts<0)
    return
end

ListSerialPorts=cell(numSerialPorts+1,1);

for i=1:numSerialPorts+1
    if(i==1)
        ListSerialPorts{i,1}='Select a Serial Port';
    else
        ListSerialPorts{i,1}=WhichSerial.AvailableSerialPorts{i-1,1};
    end
end

set(handles.serialportslist,'String',ListSerialPorts)
if(~isempty(WhichSerial.AvailableSerialPorts))
    set(handles.connect,'Enable','on')
    set(handles.connect,'String','Open')

end

handles.serialportname='Select a Serial Port';
guidata(hObject,handles)
    


% --- Executes on selection change in serialportslist.
function serialportslist_Callback(hObject, eventdata, handles)
% hObject    handle to serialportslist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
str = get(hObject, 'String');
val = get(hObject,'Value');

handles.serialportname=str{val};
guidata(hObject,handles)




% Hints: contents = cellstr(get(hObject,'String')) returns serialportslist contents as cell array
%        contents{get(hObject,'Value')} returns selected item from serialportslist


% --- Executes during object creation, after setting all properties.
function serialportslist_CreateFcn(hObject, eventdata, handles)
% hObject    handle to serialportslist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in connect.
function connect_Callback(hObject, eventdata, handles)
% hObject    handle to connect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if (strcmp(get(hObject,'String'),'Open'))
    if(strcmp(handles.serialportname,'Select a Serial Port'))
        display('Dont be an idiot, Please select a valid serial port');
    else
        handles.serialObject=serial(handles.serialportname,'Baudrate',115200);
        fopen(handles.serialObject);
        set(handles.connect,'String','Close')
        set(handles.send,'Enable','on')
        set(handles.delay,'Enable','on')
        set(handles.rate,'Enable','on')
        set(handles.trim,'Enable','on')
        set(handles.startDisplay,'Enable','on')
    end
else
    fclose(handles.serialObject);
    set(hObject,'String','Open')
    set(handles.send,'Enable','off')
    set(handles.startDisplay,'Enable','off')
    set(handles.stopDisplay,'Enable','off')
    set(handles.delay,'Enable','off')
    set(handles.rate,'Enable','off')
    set(handles.trim,'Enable','off')
    set(handles.delay,'String','0')
    set(handles.rate,'String','0')
    set(handles.trim,'String','0')
    handles.delayval=0;
    handles.rateval=0;
    handles.trimval=0;
end
    
guidata(hObject,handles)
    





function rate_Callback(hObject, eventdata, handles)
% hObject    handle to rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.rateval=str2double(get(hObject,'String'));

if(isnan(handles.rateval) || handles.rateval<0 || handles.rateval>100)
    display('Please for the love of good enter a valid value between 0 and 100')
    handles.rateval=0;
    set(hObject,'String','0')
end

guidata(hObject,handles)

% Hints: get(hObject,'String') returns contents of rate as text
%        str2double(get(hObject,'String')) returns contents of rate as a double


% --- Executes during object creation, after setting all properties.
function rate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function trim_Callback(hObject, eventdata, handles)
% hObject    handle to trim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.trimval=str2double(get(hObject,'String'));

if(isnan(handles.trimval) || handles.trimval<-8 || handles.trimval>8)
    display('Please for the love of good enter a valid value between 0 and 100')
    handles.trimval=0;
    set(hObject,'String','0')
end


guidata(hObject,handles)

% Hints: get(hObject,'String') returns contents of trim as text
%        str2double(get(hObject,'String')) returns contents of trim as a double


% --- Executes during object creation, after setting all properties.
function trim_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in startDisplay.
function startDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to startDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.stopDisplay,'Enable','on')
set(handles.startDisplay,'Enable','off')

if strcmp(get(handles.timer, 'Running'), 'off')
    start(handles.timer);
end




% --- Executes on button press in stopDisplay.
function stopDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to stopDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.stopDisplay,'Enable','off')
set(handles.startDisplay,'Enable','on')

if strcmp(get(handles.timer, 'Running'), 'on')
    stop(handles.timer);
end

function update_display(hObject,eventdata,hfigure)
% Timer timer1 callback, called each time timer iterates.
% Gets surface Z data, adds noise, and writes it back to surface object.
handles = guidata(hfigure);
ReadAndModifyDisplay(handles.serialObject);



% END USER CODE
% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% START USER CODE
% Necessary to provide this function to prevent timer callback
% from causing an error after GUI code stops executing.
% Before exiting, if the timer is running, stop it.
if strcmp(get(handles.timer, 'Running'), 'on')
    stop(handles.timer);
end
% Destroy timer
delete(handles.timer)
% END USER CODE

% Hint: delete(hObject) closes the figure
delete(hObject);
     

