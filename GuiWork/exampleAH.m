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

% Last Modified by GUIDE v2.5 04-Jun-2015 00:13:51

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
display('fuck off')


% --- Executes on button press in serialports.
function serialports_Callback(hObject, eventdata, handles)
% hObject    handle to serialports (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in serialportslist.
function serialportslist_Callback(hObject, eventdata, handles)
% hObject    handle to serialportslist (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

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
load('/home/dabang/Desktop/Work/Matlab_Analysis/Flight_Data/FPV_Plane/05_21_2015/Flight/2/EKF.mat')
for i=60000:length(EKF.x)    
    ModifyDisplay(EKF.theta(i)*pi/180,EKF.phi(i)*pi/180,EKF.vx(i),-EKF.z(i));
    pause(0.01)  
end

