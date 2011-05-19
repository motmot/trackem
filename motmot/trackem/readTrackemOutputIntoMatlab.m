%  readTrackemOutputIntoMatlab.m
% 
%  An example m-file demonstrating one method of reading the binary string
%  UDP output of the fview trackem plugin and converting to it to matlab
%  variables.
%
%  Each UDP datagram output by trackem.py is a binary string of the format:
%  '<sLdIIIIII...', where each character in the binary string represents
%  one byte.  The first byte is the camera id in the form of a string
%  character.  The next four bytes contain the frame number in the form of
%  an unsigned long.  The next eight bytes are the timestamp represented as
%  a double.  From then on, every four bytes represents one unsigned int.
%  Two of these adjacent unsigned int variables represent an X,Y pair of
%  values for each tracked point in the image frame.  The byte order of the
%  binary string is little-endian.
%
%  This file creates a udp object, opens it, reads 20 udp datagrams from
%  the buffer, converts each to matlab variables, then displays the 20 sets
%  of tracked points to the screen, one by one.  The datagrams must be read
%  from the buffer quickly enough to prevent the buffer from overflowing.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Use the IP address 127.0.0.1 as the host if both fview and matlab
% are running on the same machine.  Trackem.py sends datagrams to the port
% 28931, specify this as the LocalPort for reading. 
u = udp('127.0.0.1','LocalPort',28931,'InputBufferSize',4096);
fopen(u);

pointsarray = [];
for framen = 1:20,
    [data,count,msg,datagramaddress,datagramport] = fread(u);
    if (length(data) >= 21),
        cam_id = char(uint8(data(1)));
        frame_number = double(typecast(uint8(data(2:5)),'uint32'));
        timestamp = typecast(uint8(data(6:13)),'double');
        points = double(typecast(uint8(data(14:end)),'uint32'));
        if isempty(pointsarray),
            % pointsarray = reshape(points,length(points)/2,2);  
            % Shuo(12/16/2007): 
            % Since reshape takes elements column-wise, the above will do:
            % [x1; y1; x2; y2]-> [x1 x2; y1 y2]
            % but we need to convert [x1; y1; x2; y2] -> [x1 y1; x2 y2]
            pointsarray = reshape(points,2,length(points)/2)';
        else
            %pointsarray(:,:,end+1) = reshape(points,length(points)/2,2);
            %Shuo(12/16/2007): the same reason as above
            pointsarray(:,:,end+1) = reshape(points,2,length(points)/2)';
        end
    end
end

[x,y,n] = size(pointsarray);

for framen = 1:n,
    % clc        % Shuo(12/16/2007): display all the frames
    disp(pointsarray(:,:,framen))
    pause(.2)
end

fclose(u);
delete(u);
clear u
