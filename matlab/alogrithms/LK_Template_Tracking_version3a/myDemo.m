clc
clearvars -except data imFig tempFig

%Resolution
w = 304;
h = 240;

%ROI
% minY = 70;
% maxY = 180;
% minX = 70;
% maxX = 200;
minY = 1;
maxY = 240;
minX = 1;
maxX = 304;
if ~exist('tempFig') || ~exist('imFig')
    close all
    tempFig = figure;
    imFig = figure;
end

if ~exist('data')
    disp('Loading data...');
    file = ['/home/miacono/workspace/DATASETS/balldyn/1/ATIS/data.log.txt'];
    data = dlmread(file);
    disp ('Finished loading!');
end

%Taking just channel 1
ch = logical(data(:,1));
data(ch,:) = [];


%Creating vector of dts
currTs = data (end,2);
dt = (currTs - data(:,2)) * 80e-9;

%Laplacian filter
L = [0  1  0;
    1 -4  1;
    0  1  0];

%Initializing variables for loop
maxVar = [];
step = 0.001;
startTime = 15.0;
duration = 5;

mat = zeros(h, w);

figure(tempFig)

for ts = startTime:step:startTime+duration
    
    %Asking if increasing window size or stop
    windSize = ts - startTime;
    prompt = strcat ( 'Temporal window size = ' , num2str(windSize), '. Continue? [y/n]');
    str = input(prompt,'s');
    if isempty(str)
        str = 'Y';
    end
    if str == 'n'
        break;
    end
    
    %Considering events in temporal window
    inWindow = (dt > startTime) & (dt < ts);
    vWind = data(inWindow,:);
    X = vWind(:,4) + 1; %Indices cannot be 0
    Y =  h - vWind(:,5);
    
    %Saving events in matrix
    mat = accumarray ([Y X], 1, size(mat));
    maxVal = max(max(mat));
    imagesc (mat,[0,8]);
end

corners = ginput(2);

template = mat(corners(1,2):corners(2,2),corners(1,1):corners(2,1));

binaryTemplate = template > 0;

imagesc (binaryTemplate, [0,8]);
pause;
%Compute metric
%  mat = conv2(mat,L);
%  v = max(var(mat));
%  maxVar = [maxVar; ts v];


TemplateData.image=template;
TemplateData.weight=ones(size(template));
TemplateData.p=[0 0 0 0 0 0];

Options.TranslationIterations=30;
Options.AffineIterations=0;
Options.RoughSigma=3;
Options.FineSigma=1.5;

T_error = zeros( floor(duration/windSize),1);
i=0;
startTime = 3;
duration = 10;

mat = zeros(w,h);
figure(imFig);

for ts = startTime:windSize:startTime+duration
    %Take events in temporal window
    inWindow = (dt > (ts - windSize)) & (dt < ts);
    vWind = data(inWindow,:);
    X = vWind(:,4) + 1; %Indices cannot be 0
    Y = vWind(:,5) + 1;
    mat = accumarray ([X Y], 1, size(mat));
    
    
    hold on;
    imagesc(mat,[0 8]);
    if i == 1
        center = ginput(1);
        TemplateData.p=[0 0 0 0 center(2) center(1)];
    end
    plot(TemplateData.p(6),TemplateData.p(5),'or');
    drawnow;
    hold off;
    
    i=i+1;
    [TemplateData.p,ROIimage,T_error(i)]=LucasKanadeInverseAffine(mat,TemplateData.p,TemplateData.image,TemplateData.weight,Options);
    
end
