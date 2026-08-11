function layout = buildStateLayout(blocks)
%BUILDSTATELAYOUT 根据有序状态块分配连续且唯一的全局索引。
%   blocks 的排列顺序就是最终误差状态向量的唯一顺序。返回的 Index 允许
%   其他模块按状态名称访问索引，避免传播硬编码的数字区间。

arguments
    blocks (1, :) struct
end

% Index.<Name> 保存该状态块的连续索引；Has.<Name> 用于判断可选状态是否存在。
% 两者都作为模型布局的一部分创建，动力学、量测和反馈共享同一份定义。
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

    % 每个状态块占用一段连续区间。新增块只会推动后续块的起始位置，使用
    % Layout.Index 的代码无需随全局维数变化而修改。
    blockIndices = nextIndex:(nextIndex + block.Dimension - 1);
    layout.BlockNames(blockIndex) = string(block.Name);
    layout.BlockDimensions(blockIndex) = block.Dimension;
    layout.Index.(name) = blockIndices;
    layout.Has.(name) = true;
    nextIndex = nextIndex + block.Dimension;
end

layout.Dimension = nextIndex - 1;

end
