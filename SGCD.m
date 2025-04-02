function [next_channel_get_wg_ind,next_area_UAV_ind,channel_hypergraph_ind,C_con_num_ind,...
    CChannel_can_get]=SGCD(I,M3,M2,index_UAV_next,iso,C_con,C_con_num,next_hyper_area,channel_hypergraph,next_area_UAV,...
    next_channel_acquired_num_UAV,next_adjust_area_upper,next_channel_get_wg,...
    next_adjacency_matrix_unweighted)
M3_ind=M3;
M2_ind=M2;
index_UAV_next_ind=index_UAV_next; %低级用户的优先级次序
iso_ind=iso;
C_con_ind=C_con; %低级别无人机的区域与每一个超边中的点重合的情况
C_con_num_ind=C_con_num; %低级别无人机的区域与每一个超边中的点重合的数量
C_selected_ind=zeros(I,M2); %断边
next_hyper_area_ind=next_hyper_area; %每一个超边包含的点的区域
channel_hypergraph_ind=channel_hypergraph; %每一个超边包含的点
next_area_UAV_ind=next_area_UAV; %每一个无人机申请的区域
next_channel_acquired_num_UAV_ind=next_channel_acquired_num_UAV; %每一个无人机所要的信道数
next_adjust_area_upper_ind=next_adjust_area_upper; %每一个无人机可调节区域的上限
next_channel_get_wg_ind=next_channel_get_wg; %每一个无人机目前获得的信道编号
next_adjacency_matrix_unweighted_ind=next_adjacency_matrix_unweighted; %目前的干扰关系
CChannel_can_get=cell(1,M2); %低级别无人机可以获得的信道

M2_detail=[1:M2_ind];

for ppp=1:length(M2_detail)
 vol=find(C_con_num_ind(:,index_UAV_next_ind(ppp)-M3)==0);
 CChannel_can_get{1,index_UAV_next_ind(ppp)-M3}=vol;
end
 
for k=1:50
  for s=1:5
    for p=1:length(M2_detail)
        
        if isempty(CChannel_can_get{1,index_UAV_next_ind(p)-M3})==0
            if length(next_channel_get_wg_ind{1,index_UAV_next_ind(p)})<...
                    next_channel_acquired_num_UAV_ind(1,index_UAV_next_ind(p))
                %上面这是判断是否还需要信道
       next_channel_get_wg_ind{1,index_UAV_next_ind(p)}=[next_channel_get_wg_ind{1,index_UAV_next_ind(p)},...
           CChannel_can_get{1,index_UAV_next_ind(p)-M3}(1)];
           %更新信道超图
           channel_hypergraph_ind(CChannel_can_get{1,index_UAV_next_ind(p)-M3}(1),index_UAV_next_ind(p))=1;
           %根据信道超图，得到重叠面积统计表
           for iii=1:I
               [~,vol]=find(channel_hypergraph_ind(iii,:)~=0);
               next_hyper_area_ind{iii,1}=next_area_UAV_ind(vol,:);
               next_hyper_area_ind{iii,1}=next_hyper_area_ind{iii,1}(next_hyper_area_ind{iii,1}~=0);
               [~,idx]=unique(next_hyper_area_ind{iii,1});
               next_hyper_area_ind{iii,1}=next_hyper_area_ind{iii,1}(sort(idx));
           end

           C_con_ind=cell(I,M2);
           C_con_num_ind=zeros(I,M2);%
           for iii=1:I
               [~,vol]=find(channel_hypergraph_ind(iii,M3+1:end)==0);
               if isempty(vol)==0
                  for jjj=1:length(vol)
                      if channel_hypergraph_ind(iii,vol(jjj)+M3)==0
                      C_con_ind{iii,vol(jjj)}=intersect(next_area_UAV_ind(vol(jjj)+M3,:),next_hyper_area_ind{iii,1});
                      C_con_num_ind(iii,vol(jjj))=length(C_con_ind{iii,vol(jjj)});   
                      else
                          C_con_ind{iii,vol(jjj)}=[];
                          C_con_num_ind(iii,vol(jjj))=0;
                      end
                  end
              end
           end

        %统计包含相同点的超边数量
          iso_ind=zeros(I,1);
          for iii=1:I
              for jjj=1:I
                  tf=isequal(channel_hypergraph_ind(iii,:),channel_hypergraph_ind(jjj,:));
                  if tf==1
                     iso_ind(iii,1)=iso_ind(iii,1)+1;
                  end
              end
          end

        %更新其他无人机可获得信道
        CChannel_can_get=cell(1,M2); %低级别无人机可以获得的信道
        for pppp=1:length(M2_detail)
            vol=find(C_con_num_ind(:,index_UAV_next_ind(pppp)-M3)==0);
            if isempty(vol)==0
            for vvv=1:length(vol)
                if channel_hypergraph_ind(vol(vvv),index_UAV_next_ind(pppp))==0
            CChannel_can_get{1,index_UAV_next_ind(pppp)-M3}=...
                [CChannel_can_get{1,index_UAV_next_ind(pppp)-M3},vol(vvv)];
                end
            end
            end
        end

            end
        end
    end
    %上面是结束一个一次顺序
  end
  %结束一个断边
  
  %排排座
  for jjjj=1:I
      for iiii=1:M2
          C_selected_ind(jjjj,iiii)=C_con_num_ind(jjjj,iiii)/iso_ind(jjjj,1);
      end
  end
  % 将二维数组“压扁”为一维，并获取排序后的值和索引
  [sortedValues, flatIndices]=sort(C_selected_ind(:));
  % 计算每行每列的索引
  [rowIndices, colIndices] = ind2sub(size(C_selected_ind), flatIndices);
  
  for kjkj=1:length(sortedValues)
           if sortedValues(kjkj)>0
               if next_adjust_area_upper_ind(1,M3+colIndices(kjkj))>=C_con_num_ind(rowIndices(kjkj),colIndices(kjkj))
                  delete_index=[rowIndices(kjkj),colIndices(kjkj)];
                  next_adjust_area_upper_ind(1,M3+colIndices(kjkj))=...
                  next_adjust_area_upper_ind(1,M3+colIndices(kjkj))-C_con_num_ind(rowIndices(kjkj),colIndices(kjkj));                  
                  
                  delete_area=C_con_ind{delete_index(1),delete_index(2)};
               for didi=1:length(delete_area)
                   zanshi=next_area_UAV_ind(M3+delete_index(2),:);
                   zanshi(zanshi==delete_area(didi))=0;
                   next_area_UAV_ind(M3+delete_index(2),:)=zanshi;  
               end
                   next_area_UAV_ind(M3+delete_index(2),:)=zanshi;                  
                 
                   break;
               end
           end
  end

           for iii=1:I
               [~,vol]=find(channel_hypergraph_ind(iii,:)~=0);
               next_hyper_area_ind{iii,1}=next_area_UAV_ind(vol,:);
               next_hyper_area_ind{iii,1}=next_hyper_area_ind{iii,1}(next_hyper_area_ind{iii,1}~=0);
               [~,idx]=unique(next_hyper_area_ind{iii,1});
               next_hyper_area_ind{iii,1}=next_hyper_area_ind{iii,1}(sort(idx));
           end

           C_con_ind=cell(I,M2);
           C_con_num_ind=zeros(I,M2);%
           for iii=1:I
               [~,vol]=find(channel_hypergraph_ind(iii,M3+1:end)==0);
               if isempty(vol)==0
                  for jjj=1:length(vol)
                      if channel_hypergraph_ind(iii,vol(jjj)+M3)==0
                         C_con_ind{iii,vol(jjj)}=intersect(next_area_UAV_ind(vol(jjj)+M3,:),next_hyper_area_ind{iii,1});
                         C_con_num_ind(iii,vol(jjj))=length(C_con_ind{iii,vol(jjj)});   
                      else
                          C_con_ind{iii,vol(jjj)}=[];
                          C_con_num_ind(iii,vol(jjj))=0;
                      end     
                  end
              end
           end

        %统计包含相同点的超边数量
          iso_ind=zeros(I,1);
          for iii=1:I
              for jjj=1:I
                  tf=isequal(channel_hypergraph_ind(iii,:),channel_hypergraph_ind(jjj,:));
                  if tf==1
                     iso_ind(iii,1)=iso_ind(iii,1)+1;
                  end
              end
          end

        %更新其他无人机可获得信道
         CChannel_can_get=cell(1,M2); %低级别无人机可以获得的信道
        for pppp=1:length(M2_detail)
            vol=find(C_con_num_ind(:,index_UAV_next_ind(pppp)-M3)==0);
            if isempty(vol)==0
            for vvv=1:length(vol)
                if channel_hypergraph_ind(vol(vvv),index_UAV_next_ind(pppp))==0
            CChannel_can_get{1,index_UAV_next_ind(pppp)-M3}=...
                [CChannel_can_get{1,index_UAV_next_ind(pppp)-M3},vol(vvv)];
                end
            end
            end
        end
           
             
end


end