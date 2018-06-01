
data_delayed.N = data.AUV.N; %N is the same across all

% Get all Data
if (param.enabled(1)); data.AUV.Y  = zeros(param.AUV.datalength,  param.tf*param.sensor_sample_rate); end
if (param.enabled(2)); data.WAMV.Y = zeros(param.WAMV.datalength, param.tf*param.sensor_sample_rate); end
if (param.enabled(3)); data.QUAD.Y = zeros(param.QUAD.datalength, param.tf*param.sensor_sample_rate); end

dt = zeros(30,1);
for t = 1:data_delayed.N
    dtt = tic;
    
    % Get AUV data
    [data.AUV.Y(:,t),SR_AUV] = getRawDataAUV(data.AUV.X(:,t), data.AUV.dnu(:,t), data.WAMV.X(1:6,t), param.IMU.gyro_bias);
    data.AUV.Y(:,t)     = data.AUV.Y(:,t) + SR_AUV*randn(size(SR_AUV,1),1);
    data.AUV.IMU(:,t)   = data.AUV.Y(1:9,t);
    data.AUV.HAP(:,t)   = data.AUV.Y(10:12,t);
    
    % Get WAMV data
    [data.WAMV.Y(:,t),SR_WAMV] = getRawDataWAMV(data.WAMV.X(:,t), data.WAMV.dnu(:,t), param.IMU.gyro_bias);
    data.WAMV.Y(:,t)  = data.WAMV.Y(:,t) + SR_WAMV*randn(size(SR_WAMV,1),1);
    data.WAMV.IMU(:,t)   = data.WAMV.Y(1:9,t);
    data.WAMV.GPS(:,t)   = data.WAMV.Y(10:12,t);
    vboffset = param.VB.datalength*map.VB.N;
    data.WAMV.VB(:,t)    = data.WAMV.Y(13:13+vboffset-1, t);
    
    % Get QUAD data
    [data.QUAD.Y(:,t),SR_QUAD] = getRawDataQUAD(data.QUAD.X(:,t), data.QUAD.dnu(:,t), data.WAMV.X(1:6,t), param.IMU.gyro_bias);
    data.QUAD.Y(:,t)  = data.QUAD.Y(:,t) + SR_QUAD*randn(size(SR_QUAD,1),1);
    data.QUAD.IMU(:,t)   = data.QUAD.Y(1:9,t);
    data.QUAD.GPS(:,t)   = data.QUAD.Y(1:3,t);
    vboffset    = param.VB.datalength*(map.VB.N+1);
    lpsoffset   = param.LPS.datalength*(map.LPS.N+1);
    data.QUAD.VB(:,t)    = data.QUAD.Y(13:13+vboffset-1, t);
    data.QUAD.LPS(:,t)   = data.QUAD.Y(13+vboffset:13+vboffset+lpsoffset-1, t);
    
    data_delayed.t(:,t) = t;
    data_delayed.Y(:,t) = [data.WAMV.Y(:,t); data.AUV.Y(:,t); data.QUAD.Y(:,t)];
    
    % TTF
    dt(1:29) = dt(2:30);
    dt(30) = toc(dtt);
    ttf = degrees2dm(sum(dt)/sum(dt ~= 0)*(data.MONO.N - t)/60);
    disp(['ETA data gen: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
end

Ydelay = cell(1,data_delayed.N);
cellCounter = ones(1,data_delayed.N);

for i = 1:t
    once = 1;
    delayCount = param.maxDelay;         %Used to ensure the random delay doesnt exceed bounds
    if (i == t-3)
        delayCount = 3;
    elseif (i == t-2)
        delayCount = 2;
    elseif (i == t-1)
        delayCount = 1;
    elseif (i == t)
        delayCount = 0;
    end
    %Delayed AUV Measurements
    TSdelay = round(delayCount*rand);       %generate random time delay (between 0-4)
    delayed.type = "AUV IMU";               %set data name
    delayed.data = data.AUV.IMU(:,t);       %get correct data
    delayed.timeStamp = i;                  %mark 'calculated' timestamp
    if (isempty(Ydelay{1,i+TSdelay}))       %adds to first entrance of cell array
        Ydelay(1,i+TSdelay) = {delayed};
    else
        for k = cellCounter(i+TSdelay):length(Ydelay(:,1))      %will find the next empty cell and append to it
            if (once == 1)                                      %variable to ensure only appends once
                if ~isempty(Ydelay(k,i+TSdelay))
                    Ydelay(k+1,i+TSdelay) = {delayed};
                    once = 0;
                end
            end
        end
        once = 1;
        cellCounter(i+TSdelay) = cellCounter(i+TSdelay)+1;      %increments append start pt count
    end
    TSdelay = round(delayCount*rand);
    delayed.type = "AUV HAP";
    delayed.data = data.AUV.HAP(:,t);
    delayed.timeStamp = i;
    if (isempty(Ydelay{1,i+TSdelay}))
        Ydelay(1,i+TSdelay) = {delayed};
    else
        for k = cellCounter(i+TSdelay):length(Ydelay(:,1))
            if (once == 1)
                if ~isempty(Ydelay(k,i+TSdelay))
                    Ydelay(k+1,i+TSdelay) = {delayed};
                    once = 0;
                end
            end
        end
        once = 1;
        cellCounter(i+TSdelay) = cellCounter(i+TSdelay)+1;
    end
    %Delayed WAMV Measurements
    TSdelay = round(delayCount*rand);
    delayed.type = "WAMV IMU";
    delayed.data = data.WAMV.IMU(:,t);
    delayed.timeStamp = i;
    if (isempty(Ydelay{1,i+TSdelay}))
        Ydelay(1,i+TSdelay) = {delayed};
    else
        for k = cellCounter(i+TSdelay):length(Ydelay(:,1))
            if (once == 1)
                if ~isempty(Ydelay(k,i+TSdelay))
                    Ydelay(k+1,i+TSdelay) = {delayed};
                    once = 0;
                end
            end
        end
        once = 1;
        cellCounter(i+TSdelay) = cellCounter(i+TSdelay)+1;
    end
    TSdelay = round(delayCount*rand);
    delayed.type = "WAMV GPS";
    delayed.data = data.WAMV.GPS(:,t);
    delayed.timeStamp = i;
   if (isempty(Ydelay{1,i+TSdelay}))
        Ydelay(1,i+TSdelay) = {delayed};
    else
        for k = cellCounter(i+TSdelay):length(Ydelay(:,1))
            if (once == 1)
                if ~isempty(Ydelay(k,i+TSdelay))
                    Ydelay(k+1,i+TSdelay) = {delayed};
                    once = 0;
                end
            end
        end
        once = 1;
        cellCounter(i+TSdelay) = cellCounter(i+TSdelay)+1;
    end
    TSdelay = round(delayCount*rand);
    delayed.type = "WAMV VB";
    delayed.data = data.WAMV.VB(:,t);
    delayed.timeStamp = i;
    if (isempty(Ydelay{1,i+TSdelay}))
        Ydelay(1,i+TSdelay) = {delayed};
    else
        for k = cellCounter(i+TSdelay):length(Ydelay(:,1))
            if (once == 1)
                if ~isempty(Ydelay(k,i+TSdelay))
                    Ydelay(k+1,i+TSdelay) = {delayed};
                    once = 0;
                end
            end
        end
        once = 1;
        cellCounter(i+TSdelay) = cellCounter(i+TSdelay)+1;
    end
    
    %Delayed QUAD Measurements
    TSdelay = round(delayCount*rand);
    delayed.type = "QUAD IMU";
    delayed.data = data.QUAD.IMU(:,t);
    delayed.timeStamp = i;
    if (isempty(Ydelay{1,i+TSdelay}))
        Ydelay(1,i+TSdelay) = {delayed};
    else
        for k = cellCounter(i+TSdelay):length(Ydelay(:,1))
            if (once == 1)
                if ~isempty(Ydelay(k,i+TSdelay))
                    Ydelay(k+1,i+TSdelay) = {delayed};
                    once = 0;
                end
            end
        end
        once = 1;
        cellCounter(i+TSdelay) = cellCounter(i+TSdelay)+1;
    end
    TSdelay = round(delayCount*rand);
    delayed.type = "QUAD GPS";
    delayed.data = data.QUAD.GPS(:,t);
    delayed.timeStamp = i;
    if (isempty(Ydelay{1,i+TSdelay}))
        Ydelay(1,i+TSdelay) = {delayed};
    else
        for k = cellCounter(i+TSdelay):length(Ydelay(:,1))
            if (once == 1)
                if ~isempty(Ydelay(k,i+TSdelay))
                    Ydelay(k+1,i+TSdelay) = {delayed};
                    once = 0;
                end
            end
        end
        once = 1;
        cellCounter(i+TSdelay) = cellCounter(i+TSdelay)+1;
    end
    TSdelay = round(delayCount*rand);
    delayed.type = "QUAD LPS";
    delayed.data = data.QUAD.LPS(:,t);
    delayed.timeStamp = i;
    if (isempty(Ydelay{1,i+TSdelay}))
        Ydelay(1,i+TSdelay) = {delayed};
    else
        for k = cellCounter(i+TSdelay):length(Ydelay(:,1))
            if (once == 1)
                if ~isempty(Ydelay(k,i+TSdelay))
                    Ydelay(k+1,i+TSdelay) = {delayed};
                    once = 0;
                end
            end
        end
        once = 1;
        cellCounter(i+TSdelay) = cellCounter(i+TSdelay)+1;
    end
    TSdelay = round(delayCount*rand);
    delayed.type = "QUAD VB";
    delayed.data = data.QUAD.VB(:,t);
    delayed.timeStamp = i;
    if (isempty(Ydelay{1,i+TSdelay}))
        Ydelay(1,i+TSdelay) = {delayed};
    else
        for k = cellCounter(i+TSdelay):length(Ydelay(:,1))
            if (once == 1)
                if ~isempty(Ydelay(k,i+TSdelay))
                    Ydelay(k+1,i+TSdelay) = {delayed};
                    once = 0;
                end
            end
        end
        once = 1;
        cellCounter(i+TSdelay) = cellCounter(i+TSdelay)+1;
    end
end

cellCounter = zeros(1,size(Ydelay,2));
for ii = 1:size(Ydelay,2)
   for jj = 1:size(Ydelay,1)
       if ~isempty(Ydelay{jj,ii})
          cellCounter(ii) = cellCounter(ii) + 1; 
       end
   end
end
