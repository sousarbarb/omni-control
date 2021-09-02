%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% ONLY COMPATIBLE WITH R2021A %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all
clc

%% INITIALISATION

% Set connection parameters
% - master PC:
udpPC.ip = "127.0.0.1"; % Local IP address ({MATLAB,SimTwo}@PC1)
udpPC.portRX = 9094;          % Local port used by MATLAB
udpPC.portTX = 9808;          % Local port used by SimTwo
udpPC.datagramSize = 65000;   % Maximum size (bytes) of each datagram
udpPC.timeout = 5;            % Timeout of read/write ops (in seconds)
% - 5dpo MSL robot:
udp5dpoRob.ip = "127.0.0.1"; % Local IP address ({MATLAB,SimTwo}@PC1)
udp5dpoRob.portRX = 9001;          % Local port used by MATLAB
udp5dpoRob.portTX = 9006;          % Local port used by SimTwo
udp5dpoRob.datagramSize = 65000;   % Maximum size (bytes) of each datagram
udp5dpoRob.timeout = 5;            % Timeout of read/write ops (in seconds)

% Create transmisson and reception channels for master PC
[udpPCRX  ,udpPCTX  ] = setUDPConnection(udpPC);


%% DELETE UDP CONNECTION
flush(udpPCRX)
flush(udpPCTX)
clear udpPCRX udpPCTX
flush(udp5DPORX)
flush(udp5DPOTX)
clear udp5DPORX udp5DPOTX


%% PWM - SEND COMMANDS TEST V : START SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM0,v0-100,v1100,v20#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% PWM - SEND COMMANDS TEST V : SET NEW VELOCITY
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM0,v0-170,v1170,v20#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% PWM - SEND COMMANDS TEST V : STOP SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FRS#","string",udp5dpoRob.ip,udp5dpoRob.portRX);


%% PWM - SEND COMMANDS TEST VN: START SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM0,v0-75,v1-75,v2150#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% PWM - SEND COMMANDS TEST VN: SET NEW VELOCITY
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM0,v0-120,v1-120,v2240#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% PWM - SEND COMMANDS TEST VN: STOP SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FRS#","string",udp5dpoRob.ip,udp5dpoRob.portRX);


%% PWM - SEND COMMANDS TEST W : START SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM0,v0-65,v1-65,v2-65#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% PWM - SEND COMMANDS TEST W : SET NEW VELOCITY
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM0,v0-100,v1-100,v2-100#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% PWM - SEND COMMANDS TEST W : STOP SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FRS#","string",udp5dpoRob.ip,udp5dpoRob.portRX);




%% SEND COMMANDS TEST V : START SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM1,W0-3,W13,W20#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% SEND COMMANDS TEST V : SET NEW VELOCITY
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM1,W0-8,W18,W20#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% SEND COMMANDS TEST V : STOP SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FRS#","string",udp5dpoRob.ip,udp5dpoRob.portRX);


%% SEND COMMANDS TEST VN: START SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM1,W0-2,W1-2,W24#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% SEND COMMANDS TEST VN: SET NEW VELOCITY
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM1,W0-5,W1-5,W210#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% SEND COMMANDS TEST VN: STOP SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FRS#","string",udp5dpoRob.ip,udp5dpoRob.portRX);


%% SEND COMMANDS TEST W : START SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM1,W0-2.5,W1-2.5,W2-2.5#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% SEND COMMANDS TEST W : SET NEW VELOCITY
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FM1,W0-6,W1-6,W2-6#","string",udp5dpoRob.ip,udp5dpoRob.portRX);
%% SEND COMMANDS TEST W : STOP SEQUENCE
% (press Ctrl+Enter to execute only this section)
write(udpPCTX,"FRS#","string",udp5dpoRob.ip,udp5dpoRob.portRX);