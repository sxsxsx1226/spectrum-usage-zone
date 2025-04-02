clc;clear all; close all;

c_max=1;
N=11;
Unity_sum=zeros(c_max,N);


for M=30:5:60   
for n=11:N %grid调节上限制
for ccc=1:c_max

M1=10; %优先级高的用户数量
M2=M-M1; %优先级低的
%%
[adjust_area_upper,M,I,I_index,S_dimen,S_index,x_standard,y_standard,...
    area_collection_index,xx_un ,channel_acquired_num_UAV, area_index_UAV,...
    area_UAV, N_channel_upper]=scenario(n,M,M1,M2);


[incidence_matrix_unweighted,...
    incidence_matrix_weighted]=h_c(M1,area_UAV(1:M1,:),S_dimen); %传递用频区域申请主体数量M，各主体具体的...
%（接上面）区域代号集合area_UAV，以及区域的面积S_dimen*S_dimen个格子。上面生成的是超图关联矩阵和加权超图关联矩阵

[adjacency_matrix_unweighted,adjacency_matrix_weighted]=g_c(M1,area_UAV(1:M1,:),incidence_matrix_unweighted,incidence_matrix_weighted);
%上面是生成的图邻接矩阵和加权图邻接矩阵

%加权图优先级
%看channel_get_wg和index_UAV_wg，这两个有用
[channel_get_wg,priority_low_wg,priority_wg,index_UAV_wg,...
    indictor_wg]=chan_allo_wg(M1,I_index,...
    channel_acquired_num_UAV(1:M1),adjacency_matrix_weighted,...
    adjacency_matrix_unweighted,area_UAV(1:M1,:),N_channel_upper);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%上面就是第一阶段完成了
%下面开始低级别的分配
%首先，将上一轮获得信道的高级别用户的信息择出来

next_channel_get_wg=cell(1,1);
have_chan=[];
hhh_two=1;
for hhh=1:M1
if isempty(channel_get_wg{1,index_UAV_wg(hhh)})==0
[have_chan]=[have_chan, index_UAV_wg(hhh)];
next_channel_get_wg{1,hhh_two}=channel_get_wg{1,index_UAV_wg(hhh)};
hhh_two=hhh_two+1;
end
end
next_channel_get_wg=[next_channel_get_wg,cell(1,M2)]; %所有第二轮用户的信道归属情况

M3=length(have_chan);
%区域调节上限、信道需求、申请区域编号以及其中的内容
next_adjust_area_upper=adjust_area_upper(1,have_chan);
next_adjust_area_upper=[next_adjust_area_upper, adjust_area_upper(1,M1+1:end)];
next_channel_acquired_num_UAV=channel_acquired_num_UAV(1,have_chan);
next_channel_acquired_num_UAV=[next_channel_acquired_num_UAV,channel_acquired_num_UAV(1,M1+1:end)];
next_area_index_UAV=area_index_UAV(1,have_chan);
next_area_index_UAV=[next_area_index_UAV,area_index_UAV(1,M1+1:end)];
next_area_UAV=area_UAV(have_chan,:);
next_area_UAV=[next_area_UAV;area_UAV(M1+1:end,:)];

%得到干扰关系，前面这个关联矩阵只是过程，后面那个邻接矩阵才是关键，也就是next_adjacency_matrix_weighted
[next_incidence_matrix_unweighted,...
    next_incidence_matrix_weighted]=h_c(M3+M2,next_area_UAV,S_dimen); %传递用频区域申请主体数量M，各主体具体的...
%（接上面）区域代号集合area_UAV，以及区域的面积S_dimen*S_dimen个格子。上面生成的是超图关联矩阵和加权超图关联矩阵

[next_adjacency_matrix_unweighted,next_adjacency_matrix_weighted]=g_c(M3+M2,next_area_UAV,next_incidence_matrix_unweighted,next_incidence_matrix_weighted);
%上面是生成的图邻接矩阵和加权图邻接矩阵


%%%建立初始信道归属超图，行是信道，列是用户
channel_hypergraph=zeros(I,M3+M2);
for mmm=1:M3
   ncgw=next_channel_get_wg{1,mmm};
   for nnn=1:length(ncgw)
       channel_hypergraph(ncgw(1,nnn),mmm)=1;
   end
end

%%%建立各个低级别用户与超边初始重叠面积统计表
for iii=1:I
[~,vol]=find(channel_hypergraph(iii,:)~=0);
next_hyper_area{iii,1}=next_area_UAV(vol,:);
next_hyper_area{iii,1}=next_hyper_area{iii,1}(next_hyper_area{iii,1}~=0);
[~,idx]=unique(next_hyper_area{iii,1});
next_hyper_area{iii,1}=next_hyper_area{iii,1}(sort(idx));
end

C_con=cell(I,M2);
C_con_num=zeros(I,M2);%
for iii=1:I
    [~,vol]=find(channel_hypergraph(iii,M3+1:end)==0);
    if isempty(vol)==0
        for jjj=1:length(vol)
            C_con{iii,vol(jjj)}=intersect(next_area_UAV(vol(jjj)+M3,:),next_hyper_area{iii,1});
            C_con_num(iii,vol(jjj))=length(C_con{iii,vol(jjj)});       
        end
    end
end

%统计包含相同点的超边数量
iso=zeros(I,1);
for iii=1:I
    for jjj=1:I
    tf=isequal(channel_hypergraph(iii,:),channel_hypergraph(jjj,:));
    if tf==1
        iso(iii,1)=iso(iii,1)+1;
    end
    end
end

%获得二级用户的优先级

[index_UAV_next]=youxianji((M3+M2),...
    next_channel_acquired_num_UAV,next_adjacency_matrix_weighted,next_area_UAV);
for mmm=1:M3
index_UAV_next(index_UAV_next==mmm)=[];
end


%%%初始化完成

 [ind_next_channel_get_wg,ind_next_area_UAV,channel_hypergraph_ind,C_con_num_ind,...
     CChannel_can_get]=SGCD(I,M3,M2,index_UAV_next,iso,C_con,C_con_num,next_hyper_area,channel_hypergraph,next_area_UAV,...
  next_channel_acquired_num_UAV,next_adjust_area_upper,next_channel_get_wg,next_adjacency_matrix_unweighted);

 %%%收官了
ind_num_channel=zeros(M3+M2,1);
ind_num_area=zeros(M3+M2,1);
for ssss=1:(M3+M2)
    indind_area=ind_next_area_UAV(ssss,:);
    nonZeros=indind_area(indind_area~=0);
    ind_num_area(ssss,1)=length(nonZeros);
    ind_num_channel(ssss,1)=length(ind_next_channel_get_wg{1,ssss});
end

Unity=0;
for sss=1:M2+M3
    Unity=Unity+log(1+ind_num_channel(sss,1)*ind_num_area(sss,1));
end

Unity_sum(ccc,n)=Unity;

if mod(ccc, 50) == 0
  disp(['当前ccc的值为：', num2str(ccc)]);
  disp(['当前n的值为：',num2str(n)]);
  disp(['当前M的值为：',num2str(M)]);  
end
end

Unity_sum_all(n,1)=sum(sum(Unity_sum(:,n)))/c_max;

end
Unity_sum_sum(:,(M/5-5))=Unity_sum_all(:,1);

end

