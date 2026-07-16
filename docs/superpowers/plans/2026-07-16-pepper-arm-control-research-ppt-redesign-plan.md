# 机械臂控制研究五页PPT无图片重排 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将现有五页机械臂控制研究PPT全面重排为无装备图片、控制内容更充分且与参考PPT格式一致的版本。

**Architecture:** 继续使用用户提供的50页PPT作为唯一视觉来源，通过复制模板源页50、47、46、13、47形成新的五页起始稿。所有内容写入继承文本框和原生图形，删除封面图片及空页码占位符，最终覆盖更新现有五页版输出文件。

**Tech Stack:** PowerPoint PPTX、JavaScript ES Modules、`@oai/artifact-tool`、模板跟随脚本、PowerShell、渲染与溢出检查工具。

---

### Task 1: 更新模板页面映射

**Files:**
- Modify: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/template-frame-map.json`
- Modify: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/template-audit.txt`
- Modify: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/deviation-log.txt`

- [ ] **Step 1: 检查源第46页全部继承对象**

使用artifact-tool检查源第46页的中心圆形、外围目标框、标题和页码对象，记录稳定ID、shape ID和边界。

- [ ] **Step 2: 更新五页映射**

```json
{
  "outputSlides": [
    {"outputSlide":1,"sourceSlide":50,"narrativeRole":"无图片封面","reuseMode":"duplicate-slide"},
    {"outputSlide":2,"sourceSlide":47,"narrativeRole":"三栏控制研究内容","reuseMode":"duplicate-slide"},
    {"outputSlide":3,"sourceSlide":46,"narrativeRole":"中心目标与六个分目标","reuseMode":"duplicate-slide"},
    {"outputSlide":4,"sourceSlide":13,"narrativeRole":"控制技术路线闭环","reuseMode":"duplicate-slide"},
    {"outputSlide":5,"sourceSlide":47,"narrativeRole":"四项性能指标","reuseMode":"duplicate-slide"}
  ]
}
```

为每页补齐 `rewrite`、`delete` 编辑目标。第1页明确删除右侧图片及图片边框；第2、3、5页删除空slideNumber占位符。

- [ ] **Step 3: 验证映射**

Run:

```powershell
node "$skill\template_following_scripts\validate_template_plan.mjs" --workspace $ws --map "$ws\template-frame-map.json"
```

Expected: `status=pass`，`issueCount=0`。

### Task 2: 生成无图片五页模板起始稿

**Files:**
- Replace: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/template-starter.pptx`

- [ ] **Step 1: 按新映射复制源页面**

Run:

```powershell
node "$skill\template_following_scripts\prepare_template_starter_deck.mjs" --workspace $ws --pptx "$ws\source-template.pptx" --map "$ws\template-frame-map.json" --out "$ws\template-starter-v2.pptx" --preview-dir "$ws\template-starter-v2-preview" --layout-dir "$ws\template-starter-v2-layout"
```

Expected: 起始稿包含5页，源页顺序为50、47、46、13、47。

- [ ] **Step 2: 检查起始稿**

确认第1页仍含待删除封面图片，第2和第5页为三栏框，第3页为中心图形，第4页为流程关系图。

### Task 3: 写入控制强化内容

**Files:**
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/author-deck-v2.mjs`
- Replace: `outputs/20260716-pepper-arm-control-research/机械臂控制研究部分_五页版.pptx`

- [ ] **Step 1: 第1页删除图片并写入纯蓝封面**

写入标题“面向钢筋捆扎作业的机械臂控制研究”、内部汇报标识和四项控制主线；删除右侧装备图片和图片框，保留蓝色背景。

- [ ] **Step 2: 第2页写入三栏研究内容**

三栏标题分别为“姿态与轨迹”“转场与执行”“预检与安全”。每栏使用5个继承行框表达姿态修正、五阶段运动、MJOINT/MLIN分工、PointC/PointJ同步、双缓冲、CheckTarget、IkSolver和异常停止。

- [ ] **Step 3: 第3页写入中心目标和六个分目标**

中心内容为“安全、准确、连续的跨点捆扎控制”，外围六项目标为姿态跟随、五阶段连续控制、安全转场、局部直线运动、整任务预检、异常安全停止。

- [ ] **Step 4: 第4页写入完整技术路线**

沿用源第13页流程框，写入输入、姿态修正、五阶段生成、全局安全转场、SDK预检、混合执行、状态反馈和后续URDF/MoveIt路线。

- [ ] **Step 5: 第5页写入指标与测试口径**

保留四项指标：≤5 s/点、≤5 mm、≤3°、100%。三栏分别表达效率、精度和控制完整性，并在行框中写入测试边界和测量方式。

- [ ] **Step 6: 导出正式PPT**

Run:

```powershell
node "$ws\author-deck-v2.mjs" "$ws\template-starter-v2.pptx" "C:\Users\Administrator\Documents\efort_bot\outputs\20260716-pepper-arm-control-research\机械臂控制研究部分_五页版.pptx" "$ws\preview-v2"
```

Expected: 输出文件存在，文件大小大于0，artifact-tool可重新导入。

### Task 4: 逐页视觉检查与修正

**Files:**
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/qa/qa-ledger-v2.txt`

- [ ] **Step 1: 检查全部5页原尺寸预览**

逐页检查：封面无图片残留；第2页三栏内容无溢出；第3页中心目标与六项分目标层级清楚；第4页箭头关系正确；第5页指标和测试口径可读。

- [ ] **Step 2: 修正文字密度**

标题必须保持单行。正文过长时缩短描述，不降低到低于模板正文的可读字号。

### Task 5: 最终验证与交付

**Files:**
- Verify: `outputs/20260716-pepper-arm-control-research/机械臂控制研究部分_五页版.pptx`

- [ ] **Step 1: 运行画布溢出检查**

```powershell
python "$skill\container_tools\slides_test.py" "C:\Users\Administrator\Documents\efort_bot\outputs\20260716-pepper-arm-control-research\机械臂控制研究部分_五页版.pptx"
```

Expected: `Test passed. No overflow detected.`

- [ ] **Step 2: 运行模板保真和空占位符检查**

Expected: `status=pass`，`issueCount=0`。

- [ ] **Step 3: 核验内容和源文件**

最终deck必须为5页，不包含图片对象，且包含“姿态与轨迹”“转场与执行”“预检与安全”“≤5 s/点”“≤5 mm”“≤3°”“100%”。源PPT SHA256必须与制作前副本一致。

- [ ] **Step 4: 清理输出目录**

输出目录只保留正式PPTX，不保留预览PNG、检查日志或中间脚本。
