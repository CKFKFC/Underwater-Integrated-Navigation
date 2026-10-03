# Underwater-CL-2026

面向水下多平台的惯性基协同导航 MATLAB 代码仓库。本仓库用于课题组成员共同开发、维护和验证组合导航、协同定位等相关算法代码。

## 新成员必读

新成员请先阅读：

- [项目开发预备知识](docs/项目开发预备知识.md)

该文档说明了如何加入 GitHub 组织、配置 Git/VS Code/MATLAB 环境、克隆仓库、创建分支、提交代码、发起 Pull Request 以及处理常见协作问题。

## 环境要求

建议开发环境：

- 操作系统：Windows 10/11
- MATLAB：R2021b 及以上版本
- Git：建议安装最新版 Git for Windows
- 编辑器：Visual Studio Code
- VS Code 插件，推荐安装，不强制：
  - Chinese (Simplified) (简体中文) Language Pack for Visual Studio Code
  - MATLAB，发布者为 MathWorks
  - Git Extension Pack
  - GitHub Pull Requests
  - Todo Tree
  - EditorConfig for VS Code，可选，用于统一编辑器格式设置
  - Markdown All in One，可选，用于维护 README 和 docs 文档

具体 MATLAB 工具箱需求会随着算法模块逐步明确。若某个脚本依赖特定工具箱，请在脚本注释或对应文档中说明。

## 快速开始

第一次获取代码：

```powershell
git clone https://github.com/UnderWater-Swarm/Underwater-CL-2026.git
cd Underwater-CL-2026
code .
```

开始开发前同步主分支：

```powershell
git switch main
git pull
```

为自己的任务创建分支：

```powershell
git switch -c feature/姓名-任务简述
```

在 MATLAB 中建议从仓库根目录运行：

```matlab
restoredefaultpath;
addpath(genpath(pwd));
```

## 协作流程

本项目建议采用分支 + Pull Request 的协作方式：

```text
同步 main -> 创建个人分支 -> 开发和测试 -> commit -> push -> Pull Request -> review -> 合并 main
```

基本要求：

- 不直接在 `main` 分支上开发功能代码。
- 每个分支尽量只完成一个明确任务。
- 提交前确认 MATLAB 脚本或相关测试可以运行。
- 不提交 `.mat`、`.fig`、大型数据文件、临时文件和个人本机路径。
- Pull Request 中说明本次改动、测试情况和需要重点检查的内容。

## 文件管理约定

`.gitignore` 已经包含 MATLAB 常见临时文件和生成文件，例如：

- `*.asv`
- `*.m~`
- `*.mat`
- `*.fig`
- `slprj/`
- `codegen/`

如果确实需要共享大型数据或结果文件，请先在群内交流，不要直接提交到仓库。

## 参考文档

- [项目开发预备知识](docs/项目开发预备知识.md)
- [GitHub Flow](https://docs.github.com/en/get-started/using-github/github-flow)
- [VS Code Git 文档](https://code.visualstudio.com/docs/sourcecontrol/intro-to-git)
- [MathWorks MATLAB VS Code 插件](https://marketplace.visualstudio.com/items?itemName=MathWorks.language-matlab)
- [VS Code 简体中文语言包](https://marketplace.visualstudio.com/items?itemName=MS-CEINTL.vscode-language-pack-zh-hans)
- [Git Extension Pack](https://marketplace.visualstudio.com/items?itemName=donjayamanne.git-extension-pack)
- [GitHub Pull Requests](https://marketplace.visualstudio.com/items?itemName=GitHub.vscode-pull-request-github)
- [Todo Tree](https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.todo-tree)
- [EditorConfig for VS Code](https://marketplace.visualstudio.com/items?itemName=EditorConfig.EditorConfig)
- [Markdown All in One](https://marketplace.visualstudio.com/items?itemName=yzhang.markdown-all-in-one)
