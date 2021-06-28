clc
warning('off','all')

tcpServer = tcpip('127.0.0.1',55000,'NetworkRole','Server');
tcpClient = tcpip('127.0.0.1',55001,'NetworkRole','Client');
set(tcpClient,'Timeout',30);

fopen(tcpServer);
a=readfis('Robot Fuzzy Controller');

while(1)
    % init
    data = membrane(1);
    
    % read input
    rawData = fread(tcpServer,3);
    input1=str2double(char(rawData));
    
    rawData = fread(tcpServer,3);
    input2=str2double(char(rawData));
    
    rawData = fread(tcpServer,3);
    input3=str2double(char(rawData));
    
    % calculate
    disp([num2str(input1) ',' num2str(input2) ',' num2str(input3)]);
    output=evalfis([input1;input2;input3],a);
    result1=output(1);
    result2=output(2);
    %[result1,result2] = RobotFuzzyController(input1,input2,input3);
    result=[num2str(result1) ',' num2str(result2)];
    disp(result);
    fprintf('\n');
    
    % write output
    fopen(tcpClient);
    fwrite(tcpClient,result);
    fclose(tcpClient);
end

fclose(tcpServer);