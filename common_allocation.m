function [channel_get]=common_allocation(M,I_index,N_channel_upper,...
    index_UAV,channel_acquired_num_UAV,adjacency_matrix_unweighted)

for m=1:M
channel_available{m}=I_index;
end
channel_get={};


for m=1:M
 c_available=length(channel_available{index_UAV(m)});
 if channel_acquired_num_UAV(index_UAV(m))<=c_available

     channel_get{index_UAV(m)}=...
         channel_available{index_UAV(m)}(1:channel_acquired_num_UAV(index_UAV(m)));
     
 else
     channel_get{index_UAV(m)}=[];
 end
 if m~=M
for k=m+1:M
    if adjacency_matrix_unweighted(index_UAV(m),index_UAV(k))==1
logical_idx = ~ismember(channel_available{index_UAV(k)}, channel_get{index_UAV(m)});
channel_available{index_UAV(k)} = channel_available{index_UAV(k)}(logical_idx);
    end
end
end
end
 
end