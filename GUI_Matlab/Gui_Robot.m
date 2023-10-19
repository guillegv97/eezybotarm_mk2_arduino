function varargout = Gui_Robot(varargin)
% GUI_ROBOT MATLAB code for Gui_Robot.fig
%      GUI_ROBOT, by itself, creates a new GUI_ROBOT or raises the existing
%      singleton*.
%
%      H = GUI_ROBOT returns the handle to a new GUI_ROBOT or the handle to
%      the existing singleton*.
%
%      GUI_ROBOT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_ROBOT.M with the given input arguments.
%
%      GUI_ROBOT('Property','Value',...) creates a new GUI_ROBOT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Gui_Robot_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Gui_Robot_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Gui_Robot

% Last Modified by GUIDE v2.5 29-May-2023 10:22:13

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Gui_Robot_OpeningFcn, ...
                   'gui_OutputFcn',  @Gui_Robot_OutputFcn, ...
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


% --- Executes just before Gui_Robot is made visible.
function Gui_Robot_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Gui_Robot (see VARARGIN)

% Choose default command line output for Gui_Robot
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Gui_Robot wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Gui_Robot_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function L1_Callback(hObject, eventdata, handles)
% hObject    handle to L1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of L1 as text
%        str2double(get(hObject,'String')) returns contents of L1 as a double


% --- Executes during object creation, after setting all properties.
function L1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to L1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_1 as text
%        str2double(get(hObject,'String')) returns contents of Theta_1 as a double


% --- Executes during object creation, after setting all properties.
function Theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_2 as text
%        str2double(get(hObject,'String')) returns contents of Theta_2 as a double


% --- Executes during object creation, after setting all properties.
function Theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta_3 as text
%        str2double(get(hObject,'String')) returns contents of Theta_3 as a double


% --- Executes during object creation, after setting all properties.
function Theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_X_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_X as text
%        str2double(get(hObject,'String')) returns contents of Pos_X as a double


% --- Executes during object creation, after setting all properties.
function Pos_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Y_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Y as text
%        str2double(get(hObject,'String')) returns contents of Pos_Y as a double


% --- Executes during object creation, after setting all properties.
function Pos_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pos_Z_Callback(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pos_Z as text
%        str2double(get(hObject,'String')) returns contents of Pos_Z as a double


% --- Executes during object creation, after setting all properties.
function Pos_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
%------------------------------------------------------------------------------------
%------------------------  Funciones basicas  ---------------------------------------
%------------------------------------------------------------------------------------
% Funcion rotacion en X
function m_r=rotacio_x(theta)
m_r=[1 0 0 0;...
     0 cosd(theta) -sind(theta) 0;...
     0 sind(theta) cosd(theta)  0;...
     0 0 0 1];
 return
% Funcion rotacion en Y
 function m_r=rotacio_y(theta)
m_r=[cosd(theta) 0 sind(theta) 0;...
     0 1 0 0;...      
     -sind(theta) 0 cosd(theta)  0;...
     0 0 0 1];
 return
% Funcion rotacion en Z
 function m_r=rotacio_z(theta)
m_r=[cosd(theta) -sind(theta) 0 0;...
     sind(theta) cosd(theta) 0 0 ;...
     0 0 1 0; ...
     0 0 0 1];
 return
% Funcion traslacion
 function m_r=traslacio_z(x,y,z)
m_r=[1 0 0 x;...
     0 1 0 y;...
     0 0 1 z;...
     0 0 0 1];
 return

%------------------------------------------------------------------------------------
%------------------------ Cinematica directa ----------------------------------------
%------------------------------------------------------------------------------------
% --- Executes on button press in btn_Directa.
function btn_Directa_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Directa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes1 = handles.axes1; % Declaramos la pantalla de plot(grafica robot)
cla(axes1); % Borramos la grafica anterior
% Declaración de variables de entrada
L1=str2double(handles.L1.String);
theta1=str2double(handles.Theta_1.String);
theta2=str2double(handles.Theta_2.String);
theta3=str2double(handles.Theta_3.String);
L1 = L1/100;

%Constantes (Distancias del robot)
d1=0.0345;
d2=0.0525;
d3=0.06045;
d4=0.135;% Longitud del eslabón 1
d5=0.147;% Longitud del eslabón 2
d6=0.03331;
%------------------------------------------------------------------------------------
%------------------------------ Restriciones ----------------------------------------
%------------------------------------------------------------------------------------
% Theta 1:
if theta1 < 0 || theta1 > 90
    disp("El valor de theta1 debe estar comprendido entre 0 y 90.");
    set(handles.Pos_X, 'String', '-');
    set(handles.Pos_Y, 'String', '-');
    set(handles.Pos_Z, 'String', '-');
    return
end
% %Distancia 1:
if L1 < 0 || L1 > 0.15
    disp("El valor de theta1 debe estar comprendido entre 0 y 15.");
    set(handles.Pos_X, 'String', '-');
    set(handles.Pos_Y, 'String', '-');
    set(handles.Pos_Z, 'String', '-');
    return
end

%Theta 2:
if theta2 < -65 || theta2 > 60
    disp("El valor de theta2 debe estar comprendido entre -65 y 60.");
    set(handles.Pos_X, 'String', '-');
    set(handles.Pos_Y, 'String', '-');
    set(handles.Pos_Z, 'String', '-');
    return
end
% 
% %Theta 3:
if theta3 < -45 || theta3 > 60
    disp("El valor de theta3 debe estar comprendido entre -45 y 60.");
    set(handles.Pos_X, 'String', '-');
    set(handles.Pos_Y, 'String', '-');
    set(handles.Pos_Z, 'String', '-');
    return
end
% 
% % Maximos y minimos:
if theta2==0
    if theta3<-20
        disp("El valor de theta3 debe ser mayor.");
        set(handles.Pos_X, 'String', '-');
        set(handles.Pos_Y, 'String', '-');
        set(handles.Pos_Z, 'String', '-');
        return
    end
    if theta3>60
        disp("El valor de theta3 debe ser menor.");
        set(handles.Pos_X, 'String', '-');
        set(handles.Pos_Y, 'String', '-');
        set(handles.Pos_Z, 'String', '-');
        return
    end
end
if theta2<0
    if theta3<45
        disp("El valor de theta3 debe ser mayor.");
        set(handles.Pos_X, 'String', '-');
        set(handles.Pos_Y, 'String', '-');
        set(handles.Pos_Z, 'String', '-');
        return
    end
    if theta3>60
        disp("El valor de theta3 debe ser menor.");
        set(handles.Pos_X, 'String', '-');
        set(handles.Pos_Y, 'String', '-');
        set(handles.Pos_Z, 'String', '-');
        return
    end
end
if theta2>0
    if theta3<-45
        disp("El valor de theta3 debe ser mayor.");
        set(handles.Pos_X, 'String', '-');
        set(handles.Pos_Y, 'String', '-');
        set(handles.Pos_Z, 'String', '-');
        return
    end
    if theta3>45
        disp("El valor de theta3 debe ser menor.");
        set(handles.Pos_X, 'String', '-');
        set(handles.Pos_Y, 'String', '-');
        set(handles.Pos_Z, 'String', '-');
        return
    end
end

%Cinematica directa
a0_1=traslacio(0,L1,0);
a1_2=rotacio_z(theta1)*traslacio(0,0,d2+d3);
a2_3=rotacio_y(theta2-90)*traslacio(d4,0,0); % El -90 es para hacer que la posicion 0 del robot este subido hacia arriba
a3_4=rotacio_y(theta3+90)*traslacio(d5,0,0); % El 90 es para hacer que la posicion 0 del robot este horizontal
a4_5=rotacio_y(-theta2-theta3)*traslacio(d6,0,0);
H=a0_1*a1_2*a2_3*a3_4*a4_5

% Obtenemos las posiciones x, y, z en una matriz
M= [H(1,4);
    H(2,4);
    H(3,4);];
% Mostramos los valores X, Y, Z
x=M(1)*100
handles.Pos_X.String=num2str(x); % Declaracion para que se muestre en la GUI
y=M(2)*100
handles.Pos_Y.String=num2str(y);
z=M(3)*100
handles.Pos_Z.String=num2str(z);

% Cordenadas de cada articulacion
% Punto inicio
O=[0,0,0];
% Punto base robot
H_1=a0_1;
A1=[H_1(1,4), H_1(2,4), H_1(3,4)];
% Punto primer motor
H_2=a0_1*a1_2;
A2=[H_2(1,4), H_2(2,4), H_2(3,4)];
% Punto segundo motor
H_3=a0_1*a1_2*a2_3;
A3=[H_3(1,4), H_3(2,4), H_3(3,4)];
% Punto articulacion
H_4=a0_1*a1_2*a2_3*a3_4;
A4=[H_4(1,4), H_4(2,4), H_4(3,4)];
% Punto final
A5=[H(1,4), H(2,4), H(3,4)];

hold on % Unir todas las lineas
rotate3d % Hacer que se pueda rotar la visualizacion

% Mostramos el conjunto de lineas
plot3([0;A1(1)],[0;A1(2)],[0;A1(3)],'r',LineWidth = 3) % LineWidth = 3, hacemos que el ancho de linea sea de 3
plot3([A1(1);A2(1)],[A1(2);A2(2)],[A1(3);A2(3)],'b',LineWidth = 3)
plot3([A2(1);A3(1)],[A2(2);A3(2)],[A2(3);A3(3)],'g',LineWidth = 3)
plot3([A3(1);A4(1)],[A3(2);A4(2)],[A3(3);A4(3)],'y',LineWidth = 3)
plot3([A4(1);A5(1)],[A4(2);A5(2)],[A4(3);A5(3)],'m',LineWidth = 3)

%Añadimos los comentarios a la grafica
axis([-0.2 0.4 0 0.4 0 0.4])
xlabel('x (cm)')
ylabel('y (cm)')
zlabel('z (cm)')
title('Robot')
grid(axes1, 'on');
view(3)
set(gca, 'XDir', 'reverse', 'YDir', 'reverse') % Ajustar la posición de la figura
hold off
%------------------------------------------------------------------------------------
%------------------------ Cinematica inversa ----------------------------------------
%------------------------------------------------------------------------------------
% --- Executes on button press in btn_Inversa.
function btn_Inversa_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Inversa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes1 = handles.axes1;
cla(axes1);
x=str2double(handles.Pos_X.String)/100;
y=str2double(handles.Pos_Y.String)/100;
z=str2double(handles.Pos_Z.String)/100;
% if x < 0 || x > 0.15
%     disp("El valor de X debe estar comprendido entre 0 y 15.");
%     return
% end
% if y < 0 || y > 0.3
%     disp("El valor de Y debe estar comprendido entre 0 y 30.");
%     return
% end
% if z < 0 || z > 0.3
%     disp("El valor de Z debe estar comprendido entre 0 y 15.");
%     return
% end

d1=0.0345;
d2=0.0525;
d3=0.06045;
d4=0.135;
d5=0.147;
d6=0.03331;

L1=y;
if L1<0
    disp("fuera del limite")
    return
elseif L1<=0.150 || L1==0
    L1=y;
    disp("dentro del limite 1")
    ny_EE=0;
    q1 = atan2(ny_EE, x)

% Encuentra los valores para la posición del elemento final x, y, z
    x_4 = x - (d6);
    y_4 = ny_EE;
    z_4 = z-(d2+d3);
% Calculamos la diagonal del centro al punto
    h=sqrt(x_4^2+z_4^2);
    q2=atand(z_4/x_4);
    q3=acosd((h^2+d4^2-d5^2)/(2*h*d4))  % Teorema del Coseno
    q4=(q2+q3)
    q6=acosd((-h^2+d4^2+d5^2)/(2*d5*d4))% Teorema del Coseno
    q7=-(180-(q6+q2+q3))-q4;

    disp("Posición de entrada:")
    fprintf('x:%d\n',x*100)
    fprintf('y:%d\n',y*100)
    fprintf('z:%d\n',z*100)
% Imprime los ángulos de las articulaciones de salida
    disp("Ángulos de salida: ")
    fprintf('L1:%d\n',L1*100)
    fprintf('q1:%d\n',q1)
    fprintf('q2:%d\n',q2)
    fprintf('q3:%d\n',q3)
elseif L1>0.150
    disp("dentro del limite 2")
    L1=0.15-d1;
    ny_EE=y-0.150;
    c=sqrt(x^2+ny_EE^2);
    q1 = atan2d(ny_EE, x)

% Encuentra los valores para la posición del elemento final x, y, z
    x_4 = x - (d6 * cosd(q1));
    y_4 = ny_EE- (d6 * sind(q1));
    z_4 = z-(d2+d3);
% Calculamos la diagonal del centro al punto
    h=sqrt(x_4^2+z_4^2);
    q2=atand(z_4/c)
    q3=acosd((h^2+d4^2-d5^2)/(2*h*d4))  % Teorema del Coseno
    q4=(q2+q3)
    q6=acosd((-h^2+d4^2+d5^2)/(2*d5*d4))% Teorema del Coseno
    q7=-(180-(q6+q2+q3))-q4;

    disp("Posición de entrada:")
    fprintf('x:%d\n',x*100)
    fprintf('y:%d\n',y*100)
    fprintf('z:%d\n',z*100)
% Imprime los ángulos de las articulaciones de salida
    disp("Ángulos de salida: ")
    fprintf('L1:%d\n',L1*100)
    fprintf('q1:%d\n',q1)
    fprintf('q2:%d\n',q4)
    fprintf('q3:%d\n',-q7)
end
L1;
theta1=q1;
theta2=-q4;
theta3=-q7;
disp("valores:")
x_4, y_4, z_4, h, q1, q2, q3, q4, q6, q7

% % Restricciones 
%------------------------------------------------------------------------------------
%------------------------------ Restriciones ----------------------------------------
%------------------------------------------------------------------------------------
% Theta 1: Se ha de revisar pq no funcionan las restricciones 
if theta1 < 0 || theta1 > 90
    disp("No se puede alcanzar esta posicion por culpa de theta1 ")
    set(handles.L1, 'String', '-');
    set(handles.Theta_1, 'String', '-');
    set(handles.Theta_2, 'String', '-');
    set(handles.Theta_3, 'String', '-');
    return
end
%Theta 2:
if theta2+90 < -65 || theta2+90 > 60
    disp("No se puede alcanzar esta posicion por culpa de theta2 ")
    set(handles.L1, 'String', '-');
    set(handles.Theta_1, 'String', '-');
    set(handles.Theta_2, 'String', '-');
    set(handles.Theta_3, 'String', '-');
    return
end
% 
% %Theta 3:
if theta3-90 < -45 || theta3-90 > 60
    disp("No se puede alcanzar esta posicion por culpa de theta3 ")
    set(handles.L1, 'String', '-');
    set(handles.Theta_1, 'String', '-');
    set(handles.Theta_2, 'String', '-');
    set(handles.Theta_3, 'String', '-');
    return
end
% 
% % Maximos y minimos:
if theta2+90==0
    if theta3-90<-20
        disp("El valor de theta3 debe ser mayor.");
        set(handles.L1, 'String', '-');
        set(handles.Theta_1, 'String', '-');
        set(handles.Theta_2, 'String', '-');
        set(handles.Theta_3, 'String', '-');
        return
    end
    if theta3-90>60
        disp("El valor de theta3 debe ser menor.");
        set(handles.L1, 'String', '-');
        set(handles.Theta_1, 'String', '-');
        set(handles.Theta_2, 'String', '-');
        set(handles.Theta_3, 'String', '-');
        return
    end
end
if theta2+90<0
    if theta3-90<45
        disp("El valor de theta3 debe ser mayor.");
        set(handles.L1, 'String', '-');
        set(handles.Theta_1, 'String', '-');
        set(handles.Theta_2, 'String', '-');
        set(handles.Theta_3, 'String', '-');
        return
    end
    if theta3-90>60
        disp("El valor de theta3 debe ser menor.");
        set(handles.L1, 'String', '-');
        set(handles.Theta_1, 'String', '-');
        set(handles.Theta_2, 'String', '-');
        set(handles.Theta_3, 'String', '-');
        return
    end
end
if theta2+90>0
    if theta3-90<-45
        disp("El valor de theta3 debe ser mayor.");
        set(handles.Pos_X, 'String', '-');
        set(handles.Pos_Y, 'String', '-');
        set(handles.Pos_Z, 'String', '-');
        return
    end
    if theta3-90>45
        disp("El valor de theta3 debe ser menor.");
        set(handles.Pos_X, 'String', '-');
        set(handles.Pos_Y, 'String', '-');
        set(handles.Pos_Z, 'String', '-');
        return
    end
end

% Mostramos por pantalla los valores de los grados
handles.L1.String=num2str(round(L1*100, 2)); % Redondeo el resultado a 2 decimales
handles.Theta_1.String=num2str(round(theta1, 2));
handles.Theta_2.String=num2str(round(theta2+90, 2));
handles.Theta_3.String=num2str(round(theta3-90, 2));

%--------------------------------------------------------------------------------------------------------------
% Cinematica directa
a0_1=traslacio(0,L1,0);
a1_2=rotacio_z(theta1)*traslacio(0,0,d2+d3);
a2_3=rotacio_y(theta2)*traslacio(d4,0,0); % El -90 es para hacer que la posicion 0 del robot este subido hacia arriba
a3_4=rotacio_y(theta3)*traslacio(d5,0,0); % El 90 es para hacer que la posicion 0 del robot este horizontal
a4_5=rotacio_y(-theta2-theta3)*traslacio(d6,0,0);
H=a0_1*a1_2*a2_3*a3_4*a4_5
% Obtenemos las posiciones x, y, z en una matriz
M= [H(1,4);
    H(2,4);
    H(3,4);];
%Mostramos los valores X, Y, Z
x=M(1)*100
y=M(2)*100
z=M(3)*100

%Cordenadas de cada articulacion
% Punto inicio
O=[0,0,0];
% Punto base robot
H_1=a0_1;
A1=[H_1(1,4), H_1(2,4), H_1(3,4)];
% Punto primer motor
H_2=a0_1*a1_2;
A2=[H_2(1,4), H_2(2,4), H_2(3,4)];
% Punto segundo motor
H_3=a0_1*a1_2*a2_3;
A3=[H_3(1,4), H_3(2,4), H_3(3,4)];
% Punto articulacion
H_4=a0_1*a1_2*a2_3*a3_4;
A4=[H_4(1,4), H_4(2,4), H_4(3,4)];
% Punto final
A5=[H(1,4), H(2,4), H(3,4)];

hold on % Unir todas las lineas
rotate3d % Hacer que se pueda rotar la visualizacion
% Mostramos el conjunto de lineas
plot3([0;A1(1)],[0;A1(2)],[0;A1(3)],'r',LineWidth = 3) % LineWidth = 3, hacemos que el ancho de linea sea de 3
plot3([A1(1);A2(1)],[A1(2);A2(2)],[A1(3);A2(3)],'b',LineWidth = 3)
plot3([A2(1);A3(1)],[A2(2);A3(2)],[A2(3);A3(3)],'g',LineWidth = 3)
plot3([A3(1);A4(1)],[A3(2);A4(2)],[A3(3);A4(3)],'y',LineWidth = 3)
plot3([A4(1);A5(1)],[A4(2);A5(2)],[A4(3);A5(3)],'m',LineWidth = 3)

%Añadimos los comentarios a la grafica
axis([-0.2 0.4 0 0.4 0 0.4])
xlabel('x (cm)')
ylabel('y (cm)')
zlabel('z (cm)')
title('Robot')
grid(axes1, 'on');
view(3)
set(gca, 'XDir', 'reverse', 'YDir', 'reverse') % Ajustar la posición de la figura
hold off
