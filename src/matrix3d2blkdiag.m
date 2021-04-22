function b = matrix3d2blkdiag(a)

assert(ndims(a) == 3, "Not 3D matrix")

b_cell = num2cell(a, [1,2]);
b = blkdiag(b_cell{:});

end