function data = importRef(filename)
%IMPORTFILE Import numeric data from a text file as a matrix.
%   REF = IMPORTFILE(FILENAME) Reads data from text file FILENAME for the
%   default selection.
%
%   REF = IMPORTFILE(FILENAME, STARTROW, ENDROW) Reads data from rows
%   STARTROW through ENDROW of text file FILENAME.
%
% Example:
%   Ref = importfile('0234_Ref.csv', 1, 362);
%
%    See also TEXTSCAN.

% Auto-generated by MATLAB on 2018/02/20 18:56:02

%% Initialize variables.
delimiter = ' ';
% if nargin<=2
%     startRow = 1;
%     endRow = inf;
% end
startRow = 1;
%% Format for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string', 'HeaderLines', startRow(1)-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
endRow = size(dataArray, 1);
for block=2:length(startRow)
    frewind(fileID);
    dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true, 'TextType', 'string', 'HeaderLines', startRow(block)-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
    for col=1:length(dataArray)
        dataArray{col} = [dataArray{col};dataArrayBlock{col}];
    end
end

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Create output variable
Ref = table(dataArray{1:end-1}, 'VariableNames', {'VarName1','VarName2','VarName3','VarName4'});
data = [Ref.VarName1 Ref.VarName2 Ref.VarName3 Ref.VarName4];
