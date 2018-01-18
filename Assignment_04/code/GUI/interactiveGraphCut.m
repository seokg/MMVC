%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INTERACTIVE SEGMENTATION
% This code contains the main framework for interactive segmentation. Most
% of it concerns the interface and you won't have to touch it.
% 
% The only function that is of any interest to you is 
% 
% 'segment_ClickedCallback'
% 
% which is placed at the end of this file.
% 
% This function will need your code for creating color histograms, unary
% potentials and pairwise potentials. The m files for the corresponding
% functions were already created, and you will have to complete them.
% The parts you will have to modify are indicated by their task identifier:
% 
% - TASK 2.1 - color histograms
% - TASK 2.2 - unaries
% - TASK 2.3 - pairwise
% - TASK 2.4 - graph cut
% 
% HOW TO USE THE INTERFACE
% - Run the file
% - Open an image
% - Click on the red pencil to draw foreground object, on the blue for
%   background and the black for erasing data.
% - Click on the S to segment
% - When you change lambda, you need to press enter
% - Click on the button with the red and blue lines to load the scribbles
%   you will need to use for the report.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function varargout = interactiveGraphCut(varargin)
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interactiveGraphCut_OpeningFcn, ...
                   'gui_OutputFcn',  @interactiveGraphCut_OutputFcn, ...
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


% --- Executes just before interactiveGraphCut is made visible.
function interactiveGraphCut_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interactiveGraphCut (see VARARGIN)

% Choose default command line output for interactiveGraphCut
handles.output = hObject;

% Initialize the parameters
handles.fgDrawer = false;
handles.bgDrawer = false;
handles.erDrawer = false;
handles.lambda = 1.0;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes interactiveGraphCut wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = interactiveGraphCut_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function openFile_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to openFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[handles.filename, handles.pathname] = uigetfile({'*.png;*.jpg;*.bmp' ; '*'});
addpath(genpath(handles.pathname));

% Read the image, store it in the handles and show it
handles.image   = imread(handles.filename);
handles.width   = size(handles.image,2);
handles.height  = size(handles.image,1);
handles.channel = size(handles.image,3);

if(isfield(handles,'imgPos')) % in case another image was on screen already
    delete(handles.imgPos);
end

f = imshow(handles.filename);
handles.imgPos = f;

% Forground / Background initialization
handles.fgData = [];
handles.bgData = [];

% Make sure all drawing options are set to off
handles.fgDrawer = false;
handles.bgDrawer = false;
handles.erDrawer = false;

% Size of the brush for painting
handles.brushSize = 5;

% Update the structure
guidata(hObject, handles);


% --------------------------------------------------------------------
function fgButton_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to fgButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.fgDrawer = ~handles.fgDrawer;

if(handles.fgDrawer)
    set(handles.imgPos,'ButtonDownFcn',@startDrawFcn);
else
    set(handles.imgPos,'ButtonDownFcn',@stopDrawFcn);
end

set(handles.figure1,'WindowButtonUpFcn', @stopDrawFcn);

guidata(hObject, handles);


% --------------------------------------------------------------------
function bgButton_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to bgButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.fgDrawer = false;
handles.bgDrawer = ~handles.bgDrawer;
handles.erDrawer = false;

if(handles.bgDrawer)
    set(handles.imgPos,'ButtonDownFcn',@startDrawFcn);
else
    set(handles.imgPos,'ButtonDownFcn',@stopDrawFcn);
end

set(handles.figure1,'WindowButtonUpFcn', @stopDrawFcn);

guidata(hObject, handles);


% --------------------------------------------------------------------
function erButton_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to erButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.fgDrawer = false;
handles.bgDrawer = false;
handles.erDrawer = ~handles.erDrawer;

if(handles.erDrawer)
    set(handles.imgPos,'ButtonDownFcn',@startDrawFcn);
else
    set(handles.imgPos,'ButtonDownFcn',@stopDrawFcn);
end

set(handles.figure1,'WindowButtonUpFcn', @stopDrawFcn);

guidata(hObject, handles);

% --------------------------------------------------------------------
function startDrawFcn(hObject, eventdata)
handles = guidata( ancestor(hObject, 'image') );
fprintf('Start drawing\n');

guidata(hObject,handles);

set(handles.figure1, 'WindowButtonMotionFcn', @drawing);

% --------------------------------------------------------------------
function drawing(hObject, eventData)
handles = guidata( ancestor(hObject, 'figure') ); % recover the handles

a = get(handles.imgPos,'Parent'); % axis of the image
set(a, 'Units', 'pixels');
pt = get(a, 'CurrentPoint'); % get current position of the cursor on the image
pt = pt(1,1:2); % CurrentPoint gives 3D coordinates of a projected line, we only need the 2d position

% Paint on the image
paintedPixels = getPaintedPixels(pt,handles.brushSize,handles.width, handles.height);

LineSpec = '';
if (handles.fgDrawer)
    handles.fgData = [handles.fgData ; paintedPixels];
    handles.fgData = unique(handles.fgData,'rows');
    LineSpec = 'r';
    
elseif (handles.bgDrawer)
    handles.bgData = [handles.bgData ; paintedPixels];
    handles.bgData = unique(handles.bgData,'rows');
    LineSpec = 'b';
    
elseif (handles.erDrawer)
    LineSpec = 'k';
    
    if(~isempty(handles.fgData))
        handles.fgData(ismember(handles.fgData,paintedPixels, 'rows'),:) = [];
    end
    
    if(~isempty(handles.bgData))
        handles.bgData(ismember(handles.bgData,paintedPixels, 'rows'),:) = [];
    end
    
end

hold on
plot(paintedPixels(:,1), paintedPixels(:,2), LineSpec);

guidata(hObject, handles);


% --------------------------------------------------------------------
function stopDrawFcn(hObject, eventData)
handles = guidata( ancestor(hObject, 'figure') );
set(handles.figure1, 'WindowButtonMotionFcn', '');

function lamEdit_Callback(hObject, eventdata, handles)
% hObject    handle to lamEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lamEdit as text
%        str2double(get(hObject,'String')) returns contents of lamEdit as a double
handles.lambda = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function lamEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lamEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes when figure1 is resized.
function figure1_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function loadScribbles_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to loadScribbles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(strcmp(handles.filename, 'batman.jpg'))
    load Scribbles/scribbles_batman.mat
    handles.fgData = seed_fg;
    handles.bgData = seed_bg;
elseif(strcmp(handles.filename, 'VanDamme.jpg'))
    load Scribbles/scribbles_JCVD.mat
    handles.fgData = seed_fg;
    handles.bgData = seed_bg;
else
    return;
end

handles.imgPos = imshow(handles.image);
hold on
plot(seed_fg(:,1),seed_fg(:,2),'.r');
hold on
plot(seed_bg(:,1),seed_bg(:,2),'.b');

guidata(hObject, handles);


% --------------------------------------------------------------------
function uipushtool7_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to uipushtool7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[handles.filename_bckg, handles.pathname_bckg] = uigetfile({'*.png;*.jpg;*.bmp' ; '*'});
addpath(genpath(handles.pathname_bckg));

% Read the image, store it in the handles and show it
handles.image_bckg   = imread(handles.filename_bckg);
handles.width_bckg   = size(handles.image_bckg,2);
handles.height_bckg  = size(handles.image_bckg,1);

% Update the structure
guidata(hObject, handles);


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.brushSize = get(handles.slider1, 'Value');

set(handles.text2,'String',num2str(handles.brushSize));

guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --------------------------------------------------------------------
function segment_ClickedCallback(hObject, eventdata, handles)
tic
% hObject    handle to segment (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
addpath(genpath('GraphCut'));

% Set all drawing options to false
handles.fgDrawer = false;
handles.bgDrawer = false;
handles.erDrawer = false;

% Extract the seeds for foreground and background
seed_fg = handles.fgData;
seed_bg = handles.bgData;

% Read the image we want to segment
I = handles.image;
w = handles.width;
h = handles.height;
nPix = h*w;

% Get the color histogramm for the unaries
% I = imgaussfilt(I,2);

%% TASK 2.1 ---------------------------------------------------------------
disp('TASK 2.1: Computing Histogram')
histRes = 10;
cost_fg = getColorHistogram(I,seed_fg, histRes); % complete getColorHistogram.m
cost_bg = getColorHistogram(I,seed_bg, histRes);

% Get lambda
lam = handles.lambda;
disp(lam)
%% Set the unaries and the pairwise terms
% TASK 2.2-2.3 ------------------------------------------------------------
disp('TASK 2.2: Computing Unaries')
unaries = getUnaries(I, lam,cost_fg,cost_bg, seed_fg, seed_bg); % complete getUnaries.m
disp('TASK 2.3: Computing Pairwise')
pairwise = getPairWise(I); % complete getPairWise.m

%% Graph Cut --------------------------------------------------------------
% TASK 2.4
%  Your code here
disp('TASK 2.4: Computing Graph Cut')

myGraph = BK_Create(nPix);
BK_SetUnary(myGraph, unaries);
BK_SetNeighbors(myGraph, pairwise);
BK_Minimize(myGraph);
labels = BK_GetLabeling(myGraph) - 1;

%% Show the results
for i = 1:length(labels)
    x = floor(i/h)+1;
    y = mod(i,h)+1;
    
    if(~labels(i))
        I(y,x,2) = 0;
        I(y,x,3) = 0;
    else
        I(y,x,1) = 0;
        I(y,x,2) = 0;
    end
end

delete(handles.imgPos);
handles.imgPos = imshow(I);

if(isfield(handles, 'image_bckg'))
    I_orig = imread(handles.filename);
    I_bck  = handles.image_bckg;
    
    for i = 1:length(labels)
        x = floor(i/h)+1;
        y = mod(i,h)+1;

        if(labels(i))
            if(x <= handles.width_bckg && y <= handles.height_bckg)
                I_orig(y,x,1) = I_bck(y,x,1);
                I_orig(y,x,2) = I_bck(y,x,2);
                I_orig(y,x,3) = I_bck(y,x,3);
            end
        end
    end
    
    figure(1)
    clf
    imshow(I_orig)
end

guidata(hObject, handles);
toc
