function [adjacency_matrix_unweighted,adjacency_matrix_weighted]=g_c(M,area_UAV,incidence_matrix_unweighted,incidence_matrix_weighted)
adjacency_matrix_weighted=zeros(M);
adjacency_matrix_unweighted=zeros(M);


if isempty(incidence_matrix_weighted)==0
for s=1:length(incidence_matrix_weighted(:,1))
   indices=find(incidence_matrix_weighted(s,:));
   for i=1:length(indices)
       for j=1:length(indices)
           adjacency_matrix_weighted(indices(i),indices(j))=...
           adjacency_matrix_weighted(indices(i),indices(j))...
               +incidence_matrix_weighted(s,indices(i));
       end
   end
end
end

for m=1:M
adjacency_matrix_weighted(m,m)=0;
end

for m=1:M
    for n=1:M
        if adjacency_matrix_weighted(m,n)~=0
       adjacency_matrix_unweighted(m,n)=1;
        end
    end
end


end