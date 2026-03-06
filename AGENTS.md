# AGENTS.md
<!-- check anchors: L1 + L2- ; L1 > L2 ; PROMPT_SELF_MAINTENANCE -->

## 适用范围

- 本规则适用于仓库根目录及全部子目录。

## 硬约束（必须遵守）

- 全程中文回复。
- 用户明确要求“仅回答”时，只给结论，不改代码、不跑命令。
- 未经用户明确指令，不执行 `push`。
- 一个 bug 只做一个 commit（原子提交）。
- 每次任务开始第一行必须声明：`启用规则：L1 + L2-模块名`。
  - 若不涉及模块化规则，写：`启用规则：L1`。

## 提示词加载规则

- 默认加载主提示词：`AI_PROMPT.md`（L1）。
- 涉及具体模块（OLED/菜单、舵机/PS2、I2C/陀螺仪、TOF、电机/CAN）时，再加载：`AI_PROMPT_L2.md`（L2）。
- 冲突优先级：用户最新明确指令 > 本文件 > L1 > L2。

## 执行前后检查

- 执行前：
  - 明确本次改动模块和边界。
  - 明确是否启用 L2 的具体模块。
- 执行后：
  - 每个任务收尾都要做一次“精华沉淀判定”（按 `AI_PROMPT.md` 的 `PROMPT_SELF_MAINTENANCE`）。
  - 若判定需要沉淀，必须同步更新：
    - `AI_PROMPT.md` 或 `AI_PROMPT_L2.md`
    - `AI_PROMPT_CHANGELOG.md`
  - 若本次修改了 AI 协作文档，运行：
    - `./scripts/check_ai_prompt.ps1`
  - 若本次修改了代码，按 L1 规则执行编译验证并汇报风险。
