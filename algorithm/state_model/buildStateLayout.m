function layout = buildStateLayout(blocks)
%BUILDSTATELAYOUT 为有序状态块分配连续且唯一的全局索引。

arguments
    blocks (1, :) struct
end

layout = struct();
layout.Dimension = 0;
layout.BlockNames = strings(1, numel(blocks));
layout.BlockDimensions = zeros(1, numel(blocks));
layout.Index = struct();
layout.Has = struct();

nextIndex = 1;
for blockIndex = 1:numel(blocks)
    block = blocks(blockIndex);
    name = char(block.Name);
    if isfield(layout.Index, name)
        error("buildStateLayout:DuplicateBlock", ...
            "State block %s appears more than once.", name);
    end
    if block.Dimension < 1 || fix(block.Dimension) ~= block.Dimension
        error("buildStateLayout:InvalidDimension", ...
            "State block %s must have a positive integer dimension.", name);
    end

    blockIndices = nextIndex:(nextIndex + block.Dimension - 1);
    layout.BlockNames(blockIndex) = string(block.Name);
    layout.BlockDimensions(blockIndex) = block.Dimension;
    layout.Index.(name) = blockIndices;
    layout.Has.(name) = true;
    nextIndex = nextIndex + block.Dimension;
end

layout.Dimension = nextIndex - 1;

end
