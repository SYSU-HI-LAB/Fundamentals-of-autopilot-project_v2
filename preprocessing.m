img = imread('C:\Users\User\Desktop\sysu6001200.png');
% 将图片转换为灰度图像
gray_img = rgb2gray(img);
% 将亮度值二值化为0和1
sysu = imbinarize(gray_img);
map = sysu;

figure;
imagesc(map);%imagesc 可以将矩阵或数组的值映射到颜色图像中
colormap(flipud(gray));%colormap 将数据值映射到颜色空间中 colormap(flipud(gray)) 来设置颜色映射表为灰度图像。
hold on;
axis equal;
axis off;
car_long=0.12;
car_wide=0.06;
resolution=0.01;
% coder.varsize( 'map' );
% map=expand_race_track(map,12);

tmpmap=shrink_race_track(map,round(car_wide/resolution/2));
figure;
imagesc(tmpmap);%imagesc 可以将矩阵或数组的值映射到颜色图像中
colormap(flipud(gray));%colormap 将数据值映射到颜色空间中 colormap(flipud(gray)) 来设置颜色映射表为灰度图像。
hold on;
axis equal;
axis off;
map=gen_obstacle(tmpmap,map,12,round(car_wide/resolution),20,5);%单边，shrink后的map,map,半个障碍物边长，应该是车的半个边长，应该是你想更安全的边长
size(map)
out=zeros(600,1200,3);%全黑
[row,col] = find(map == 0);%找没被占据的
for i=1:length(row)
        out(row(i),col(i),:)=[1,1,1];%白
end

figure;
imagesc(map);%imagesc 可以将矩阵或数组的值映射到颜色图像中
colormap(flipud(gray));%colormap 将数据值映射到颜色空间中 colormap(flipud(gray)) 来设置颜色映射表为灰度图像。
hold on;
save('sysu_standard.mat','map','out');
load('sysu_standard.mat');
axis equal;
axis off;
% figure;
% imagesc(map);%imagesc 可以将矩阵或数组的值映射到颜色图像中
% colormap(flipud(gray));%colormap 将数据值映射到颜色空间中 colormap(flipud(gray)) 来设置颜色映射表为灰度图像。
% axis equal;
% axis off;
% data=num2cell(map);
% size(data)

% dsm = get_param('Copy_of_car_sim/track', 'RuntimeObject');
% write(dsm, {data});

% track = timeseries(map);
% save('sysu2.mat','track', '-v7.3');


function new_map=gen_obstacle(tmpmap,map,obstacle_wide,car_wide,safe_distance,obstacle_num_max)
%单边obstacle_wide+car_wide+safe_distance之内没有occ
    obstacle_num=0;
    [row,col] = find(tmpmap == 0);
    while obstacle_num<obstacle_num_max
        idx = randperm(length(row), 1);
        if(col(idx)<100)%不能生成在初始位置
            continue;
        end
        safe_wide=obstacle_wide+car_wide+safe_distance;
        have_at_least_one_neigb_occ=0;%有一个被占据说明不安全，无法生成
           for l= row(idx)-safe_wide : row(idx)+safe_wide
               if (l>600)||(l<1)
                  continue;
               end
               for m= col(idx)-safe_wide : col(idx)+safe_wide
                   if (m>1200)||(m<1)
                      continue;
                   end
                   this_neighbor_occ=tmpmap(l,m);
                   if this_neighbor_occ==1
                       have_at_least_one_neigb_occ=1;
                       break;
                   end
               end
               if  have_at_least_one_neigb_occ==1
                   break;
               end
           end
           if have_at_least_one_neigb_occ==0
               for l= row(idx)-obstacle_wide :row(idx)+obstacle_wide
                   if (l>600)||(l<1)
                      continue;
                   end
                   for m= col(idx)-obstacle_wide : col(idx)+obstacle_wide
                       if (m>1200)||(m<1)
                          continue;
                       end
                       map(l,m)=1;
                   end
               end
                obstacle_num=obstacle_num+1;
               [row,col] = find(map == 0);
           end
    end
    new_map=map;
end
function new_map=expand_race_track(map,index)
%如果八个相邻栅格有一个是被占据，则说明这个栅格在边缘,可以膨胀  %四个相邻也可以试试
    new_map=map;
    for i= 1: size(map,1) 
        for j=1:size(map,2)
           have_at_least_one_neigb_occ=0;
           this_grid_occ=map(i,j);
           if this_grid_occ==1
                continue;
           end
           for l= i-1 : i+1
               if (l>600)||(l<1)
                  continue;
               end
               for m= j-1 : j+1
                   if (m>1200)||(m<1)
                      continue;
                   end
                   this_neighbor_occ=map(l,m);
                   if this_neighbor_occ==1
                       have_at_least_one_neigb_occ=1;
                       break;
                   end
               end
               if  have_at_least_one_neigb_occ==1
                   break;
               end
           end
           %根据have_at_least_one_neigb_occ的值决定膨胀与否
           if  have_at_least_one_neigb_occ==1 
               for l= i-index : i+index
                   if (l>600)||(l<1)
                      continue;
                   end
                   for m= j-index : j+index
                       if (m>1200)||(m<1)
                          continue;
                       end
                       new_map(l,m)=0;
                   end
               end
           end
        end
    end
end
function new_map=shrink_race_track(map,index)
%如果八个相邻栅格有一个是被占据，则说明这个栅格在边缘,可以膨胀  %四个相邻也可以试试
    new_map=map;
    for i= 1: size(map,1) 
        for j=1:size(map,2)
           have_at_least_one_neigb_occ=0;
           this_grid_occ=map(i,j);
           if this_grid_occ==1
                continue;
           end
           for l= i-1 : i+1
               if (l>600)||(l<1)
                  continue;
               end
               for m= j-1 : j+1
                   if (m>1200)||(m<1)
                      continue;
                   end
                   this_neighbor_occ=map(l,m);
                   if this_neighbor_occ==1
                       have_at_least_one_neigb_occ=1;
                       break;
                   end
               end
               if  have_at_least_one_neigb_occ==1
                   break;
               end
           end
           %根据have_at_least_one_neigb_occ的值决定膨胀与否
           if  have_at_least_one_neigb_occ==1 
               for l= i-index : i+index
                   if (l>600)||(l<1)
                      continue;
                   end
                   for m= j-index : j+index
                       if (m>1200)||(m<1)
                          continue;
                       end
                       new_map(l,m)=1;
                   end
               end
           end
        end
    end
end