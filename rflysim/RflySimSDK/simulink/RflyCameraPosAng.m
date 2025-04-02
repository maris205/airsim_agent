function RflyCameraPosAng(x,y,z,roll,pitch,yaw)
% x y z为设置相机的位置，单位m，北东地
%roll pitch yaw为设置相机方向，单位度

if ~exist('x','var')
   x=0;
end


if ~exist('y','var')
   y=0;
end

if ~exist('z','var')
   z=0;
end

if ~exist('roll','var')
   roll=0;
end

if ~exist('pitch','var')
   pitch=0;
end


if ~exist('yaw','var')
   yaw=0;
end

out = uint8(['RflyCameraPosAng ',num2str(x),' ',num2str(y),' ',num2str(z),' ',num2str(roll),' ',num2str(pitch),' ',num2str(yaw)]);
len=length(out);
yy=[out,uint8(zeros(1,52-len))];
yy=[typecast(int32(1234567890),'uint8'),yy];
u=udp('255.255.255.255','RemotePort',20010);
fopen(u);
fwrite(u,yy);
fclose(u);
delete(u)
%u3=udp('127.0.0.1','RemotePort',8848,'LocalPort',8850);%同上

end