function RflySendUE4CMD(x)
% x y z为设置相机的位置，单位m，北东地
%roll pitch yaw为设置相机方向，单位度

if ~exist('x','var')
   x='RflyChangeMapbyName Grasslands';
end

out = uint8(x);
len=length(out);
yy=[out,uint8(zeros(1,52-len))];
yy=[typecast(int32(1234567890),'uint8'),yy];
u=udp('255.255.255.255','RemotePort',20010);
fopen(u);
fwrite(u,yy);
fclose(u);
delete(u)

end