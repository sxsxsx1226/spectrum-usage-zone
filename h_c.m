function[incidence_matrix_unweighted,...
    incidence_matrix_weighted]=h_c(M,area_UAV,S_dimen)
incidence_matrix_pre=zeros(S_dimen*S_dimen,M); %预初始化一个预关联矩阵，全为0

for s=1:S_dimen*S_dimen
    for m=1:M
        for a=1:length(area_UAV(m,:))
            if area_UAV(m,a)==s
                incidence_matrix_pre(s,m)=1;  %以格点数量为行数量，无人机为列数量，统计哪个无人机在哪个区域上有用频需求
            end

        end
    end
end

incidence_matrix_ing=[];
for s=1:S_dimen*S_dimen
if sum(incidence_matrix_pre(s,:))>1
   incidence_matrix_ing=[incidence_matrix_ing;incidence_matrix_pre(s,:)]; %同一个格点有超过两个无人机的是有干扰的，留下来，成立ing矩阵
end
end

[incidence_matrix_unweighted,~,ic]=unique(incidence_matrix_ing,'rows','stable'); %两个无人机之间可能有多个区域重叠，即归一化边与超边，构成无加权图关联矩阵
counts=accumarray(ic,1);

incidence_matrix_weighted=[];
for s=1:length(counts(:,1))
incidence_matrix_weighted=[incidence_matrix_weighted;incidence_matrix_unweighted(s,:).*counts(s,1)]; %构成加权超图的关联矩阵
end

end

