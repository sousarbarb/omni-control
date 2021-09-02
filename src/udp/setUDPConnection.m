function [rx,tx] = setUDPConnection(UDPConnParam)
%SETUDPCONNECTION
%
%        [RX,TX] = SETUDPCONNECTION(UDPCONNPARAM)
%
%  Input:
%  UDPCONNPARAM: struct
%    .ip          : IP address of the machine (e.g., 127.0.0.1)
%    .portRX      : Local port from which the desired data will be sent (e.g., 9094)
%    .portTX      : port from which the data is received in MATLAB (in SimTwo, 9808)
%    .datagramSize: maximum size (bytes) of the output datagram (1..65507)
%    .timeout     : timeout of the UDP connection
%
%  Output:
%  RX: UDPPort object for receiving data through the established UDP connection.
%  TX: UDPPort object for transmiting data through the established UDP connection.
%
%Execute read operations:
%  read(RX,NUM_MAX_STRINGS,"string")
%  
%  Returns:
%  DATAGRAM ARRAY
%  ...(i).Data         : string containing the desired data
%  ...(i).SenderAddress: IP address of the sender
%  ...(i).SenderPort   : Port from which it was sent the data
%
%Execute write operations:
%  write(TX,"COMMAND_1,COMMAND_2,...,COMMAND_N#","string",UDPConnParam.ip,UDPConnParam.portTX)
%
%NOTE: I think that with the new function udpport it is possible to set a bi-directional UDP 
%      connection with the same object.
%      However, SETUDPCONNECTION always created two different UDP objects for transmission and
%      reception of data.
%
%Clear UDP sockets:
%  flush(RX/TX)         : flushes all data from both the input and output buffers of the specified
%                         UDP socket.
%  flush(RX/TX,"input") : flushes only the input buffer.
%  flush(RX/TX,"output"): flushes only the output buffer.
%
%Delete UDP connection:
%  flush(RX/TX)
%  clear RX/TX

  UDPConnParam.portRX       = uint16(UDPConnParam.portRX);
  UDPConnParam.portTX       = uint16(UDPConnParam.portTX);
  UDPConnParam.datagramSize = uint16(UDPConnParam.datagramSize);
  % Configure data reception channel
  rx = udpport(...
    "datagram","IPV4",...
    "LocalHost",UDPConnParam.ip,...
    "LocalPort",UDPConnParam.portRX,...
    "OutputDatagramSize",UDPConnParam.datagramSize,...
    "Timeout",UDPConnParam.timeout...
  );
  % Configure data transmission channel
  tx = udpport(...
    "datagram","IPV4",...
    "LocalHost",UDPConnParam.ip,...
    "OutputDatagramSize",UDPConnParam.datagramSize,...
    "Timeout",UDPConnParam.timeout...
  );
  % Flush UDP buffers
  flush(rx);
  flush(tx);
end