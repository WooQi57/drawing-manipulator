clearvars;

% t_client=tcpip('localhost',30000,'NetworkRole','client');%与本地主机建立连接，端口号为1024，作为客户机连接。
% fopen(t_client);%与一个服务器建立连接，直到建立完成返回，否则报错。

data_recv=[];
% while true
%     data_recv=fscanf(t_client);
%     if ~(isempty(data_recv))
%         break
%     end
% end
  %receive 'connected'

RGB_ = imread('20.jpg');    %image input to convert to binay image 
gg_=rgb2gray(RGB_);
gausFilter = fspecial('gaussian',[5 5],10); 
figure(5)
imshow(gg_);
  
blur=imfilter(gg_,gausFilter,'replicate'); 
figure(4)
imshow(blur);
t=edge(blur,'Canny');

[sizeX,sizeY,tt] = size(RGB_); %Check if image is square
% if 4*sizeX-3*sizeY~=0
%     disp('Input image is not 4:3.')
%     disp('Please load a 4:3 image.')
%     return
% end
RGB=RGB_;
% RGB=imresize(RGB_,[960 960]);  %15cm*20cm 480*640

I = RGB;     %input image is rgb formate 
                      %if you are inputing binary or grayscale image
                      %please use any other image input method
        
     
% Define thresholds for channel 1 based on histogram settings
    channel1Min = 0.000000;
    channel1Max = 255;

% Define thresholds for channel 2 based on histogram settings
    channel2Min = 0.0000;
    channel2Max = 255;

% Define thresholds for channel 3 based on histogram settings
    channel3Min = 100;
    channel3Max = 200;
%     channel3Max = 0.657;%0.557 is good for kenan

% Create mask based on chosen histogram thresholds
    sliderBW = ( (I(:,:,1) >= channel1Min) & (I(:,:,1) <= channel1Max) ) & ...
        (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
        (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
    BW = sliderBW;                                                                  %from input to till here its just convert to bnary image      
                                                                                    %this part of code can be change according to input image to get correct binary image 
    
    %flip is use to adjust how image will be drawn by  robot arm                                                                                
    %BW = flip(BW ,2);                                                                      
%     BW = flip(BW ,1);
%     BW = imrotate(BW,-90); %adjust how drawing looks in Vrep camera 

%     BW = bwmorph(BW,'thicken',1);
    figure(1)
    imshow(t)

    id =im2double(BW);
    ibw =t;
    [a,b]= size(ibw);
    ibw_white =1-ibw;

    [B,L] = bwboundaries(ibw_white,8,'holes');

    figure(2)
    White=ibw_white;
    White(1:a,1:b)=1;
    imshow(White);
    hold on
    ki=1;
    while (ki <= length(B))
        boundary = B{ki};
        id=1;
        if (length(boundary(:,2))<0)
            B(ki)=[];
            continue;
        end
%         while (id <=length(boundary(:,2)))
%             if (boundary(id,2)==1||boundary(id,2)==b||boundary(id,1)==1||boundary(id,1)==a)
% %                 disp('d');
%                 boundary(id,:)=[];
%                 id=id-1;
%             end
%             id=id+1;
%         end
        plot(boundary(:,2), boundary(:,1),'k','LineWidth',1);
        B{ki}=boundary;
        
        ki=ki+1;
    end
    
    %this part of code make the motion trajectory of robot arm 
    x= [];
    y= [];
    z= [];
    count =1;
    
    %min distance of to points:1mm in 15cm*20cm
    dis_min = 1706/300;
    
    figure(3)
    imshow(White);
    hold on
    for k = 1:length(B)
        boundary =B{k};
        start(k)=count;
%         fprintf(t_client,"new_spline\n");%发送文本数据
%         data_recv=fscanf(t_client);  %receive 'next_point'
%         z(count)= 1;  %new curve
        for i=1:length(boundary(:,2))
%             if (boundary(i,2)==1||boundary(i,2)==b||boundary(i,1)==1||boundary(i,1)==a)
%                 continue;
%             end
            if (count >= 2)
                if((boundary(i,2)-x(count-1))^2+(boundary(i,1)-y(count-1))^2>dis_min^2)
                    x(count) = boundary(i,2);
                    y(count) = boundary(i,1);                    
                    z(count) = 0;
                    data=sprintf('%d,%d\n',x(count),y(count));
%                     fprintf(t_client,data);
                    count = count+1;
                end
            else
                x(count) = boundary(i,2);
                y(count) = boundary(i,1);
                z(count) = 0;
                data=sprintf('%d,%d\r\n',x(count),y(count));
%                 fprintf(t_client,data);
                count = count+1;
                
            end            
        end
%         fprintf(t_client,"move\r\n");%发送文本数据
%         data_recv=fscanf(t_client);  %receive 'move_done'
        plot(x(start(k):count-1),y(start(k):count-1),'k','LineWidth',1);
%         z(count-1)=2;  %end of curve
    end
    
% fprintf(t_client,"finish\n");%发送文本数据
    