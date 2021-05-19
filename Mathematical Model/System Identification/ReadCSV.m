% Cam Osborn
% Trim CSV Data to gather only critical information

%Trim CSV to remove Take Off and Landing Time
landingTime = 10;
takeOffTime = 10; 

% Read CSV Data (this can also read xlsx, txt, and dat  files)
filename = 'TestFile.CSV';
data = readmatrix(filename);
size(data);

% Delete data for the first 10 data points
data([1:takeOffTime],:) = [];

% Find the size of the data
maxSize = size(data);

% Set the size to the number of rows in the data
maxSize = maxSize(1);

% Trim the last 10 Values
deleteSize = maxSize-landingTime;
data([deleteSize+1:maxSize],:) = [];

save('TestFile.mat', 'data'); 