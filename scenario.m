function [adjust_area_upper,M,I,I_index,S_dimen,S_index,x_standard,y_standard,...
    area_collection_index,xx_un ,channel_acquired_num_UAV,...
    area_index_UAV, area_UAV,N_channel_upper]=scenario(n,M,M1,M2)

I=8; %信道数量
I_index=[1:1:I]; %信道的序列
S_dimen=10; %待定正方形区域一条边的grid数
S_index=[];
adjust_max=0:1:21;
adjust_area_upper=adjust_max(n).*[zeros(1,M1),ones(1,M2)];



for s_row=1:S_dimen
    for s_column=1:S_dimen
    S_index(s_row,s_column)=(s_row-1)*S_dimen+s_column;
    end
end

x_standard_low=4; 
x_standard_upper=5;
y_standard_low=4; 
y_standard_upper=5;

%横长竖短待选区域组合
area_collection_index=[];
xx_un=1;
for x_standard=x_standard_low:1:x_standard_upper
for y_standard=y_standard_low:1:y_standard_upper

for column_1=1:(S_dimen+1-x_standard)
    for row_1=1:(S_dimen+1-y_standard)
    area_collection_index(xx_un,1:x_standard*y_standard)=reshape([S_index([row_1:1:row_1+y_standard-1],[column_1:1:column_1+x_standard-1])],1,x_standard*y_standard);
    if (x_standard<x_standard_upper) || (y_standard<y_standard_upper)
    area_collection_index(xx_un,x_standard*y_standard+1:x_standard_upper*y_standard_upper)=zeros(1,x_standard_upper*y_standard_upper-x_standard*y_standard); 
    end
    xx_un=xx_un+1;
    end
end
end
end
%%
xx_un=xx_un-1; %计算区域待选策略数量
channel_acquired_num_UAV=[];
area_index_UAV=[];
area_UAV=[];
N_channel_upper=4; %信道上限
for m=1:M
    channel_acquired_num_UAV(m)=randi([3,N_channel_upper]);
    area_index_UAV(m)=randi([1,xx_un]);
    area_UAV(m,:)=area_collection_index(area_index_UAV(m),:);
end

end
