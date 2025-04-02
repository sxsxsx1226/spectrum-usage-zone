
function [index_UAV]=youxianji(M,...
    channel_acquired_num_UAV,adjacency_matrix_weighted,area_UAV)

area_UAV_sum=sum(area_UAV ~= 0, 2);
area_UAV_sum=area_UAV_sum';

adjacency_iter_matrix_weighted=adjacency_matrix_weighted;
index_UAV=[];
indictor=ones(1,M);

for k=1:M
    for m=1:M
        channel_acac=channel_acquired_num_UAV;
        channel_acac(m)=0;
        channel_acac_matrix=[channel_acac];

        row=find(adjacency_iter_matrix_weighted(:,m)>0);
        if isempty(row)==1
            priority_low(m)=0.5;
        else
                priority_low(m)=sum(sum(channel_acac_matrix.*adjacency_iter_matrix_weighted(m,:)));
        end

   end
   priority(k,:)=indictor.*channel_acquired_num_UAV.*area_UAV_sum./priority_low;

   [~,p]=max(priority(k,:));
   index_UAV=[index_UAV,p];
   indictor(1,p)=0;
   adjacency_iter_matrix_weighted(:,p)=0.*adjacency_iter_matrix_weighted(:,p);
   adjacency_iter_matrix_weighted(p,:)=0.*adjacency_iter_matrix_weighted(p,:);
end

end
