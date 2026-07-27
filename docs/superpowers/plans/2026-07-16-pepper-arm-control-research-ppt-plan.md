# 机械臂控制研究五页PPT Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 使用用户提供的《大棚辣椒植保与运输一体化智能装备》PPT模板，制作并验证一份五页机械臂控制研究专题PPT。

**Architecture:** 在外部临时目录中完成模板审查、页面复制、元素级文本替换、PPTX导出与渲染检查。最终文件单独写入项目 `outputs` 目录，源PPT保持不变；所有页面均从模板源页面复制，不重新搭建视觉主题。

**Tech Stack:** PowerPoint PPTX、JavaScript ES Modules、`@oai/artifact-tool`、模板跟随脚本、PowerPoint/LibreOffice渲染检查、PowerShell。

---

## 文件结构

- 读取：`C:/Users/Administrator/xwechat_files/wxid_5gj02g0ti1o412_ec3a/msg/file/2026-07/大棚辣椒植保与运输一体化智能装备.pptx`
- 读取：`docs/superpowers/specs/2026-07-16-pepper-arm-control-research-ppt-design.md`
- 创建：`%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/template-audit.txt`
- 创建：`%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/source-notes.txt`
- 创建：`%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/template-frame-map.json`
- 创建：`%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/deviation-log.txt`
- 创建：`%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/template-starter.pptx`
- 创建：`%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/author-deck.mjs`
- 创建：`outputs/20260716-pepper-arm-control-research/机械臂控制研究部分_五页版.pptx`

### Task 1: 固化模板页面映射与内容来源

**Files:**
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/template-audit.txt`
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/source-notes.txt`
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/template-frame-map.json`
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/deviation-log.txt`

- [ ] **Step 1: 核对模板审查产物完整性**

检查 `template-manifest.json` 显示 `slideCount: 50`，并确认 `source-slide-01.png` 至 `source-slide-50.png` 均存在。

Run:

```powershell
$ws=Join-Path $env:TEMP 'codex-presentations\019f5c97\pepper-arm-control-research\tmp'
(Get-Content "$ws\template-inspect\template-manifest.json" -Raw | ConvertFrom-Json).slideCount
(Get-ChildItem "$ws\template-inspect\source-slides\source-slide-*.png").Count
```

Expected: 两行均输出 `50`。

- [ ] **Step 2: 记录模板审查结论**

`template-audit.txt` 必须写明：16:9页面、蓝色标题栏、白底正文、中文宋体/微软雅黑/等线层级、页码习惯、128个媒体资源、两张TIFF兼容风险，以及本次只复用不依赖TIFF的页面。

- [ ] **Step 3: 建立五页源页面映射**

映射先采用以下页面并以布局JSON核对文本框容量：

```json
{
  "outputSlides": [
    {"outputSlide":1,"sourceSlide":50,"narrativeRole":"机械臂控制研究封面","reuseMode":"duplicate-slide","editTargets":[]},
    {"outputSlide":2,"sourceSlide":15,"narrativeRole":"研究内容","reuseMode":"duplicate-slide","editTargets":[]},
    {"outputSlide":3,"sourceSlide":29,"narrativeRole":"研究目标","reuseMode":"duplicate-slide","editTargets":[]},
    {"outputSlide":4,"sourceSlide":13,"narrativeRole":"技术路线","reuseMode":"duplicate-slide","editTargets":[]},
    {"outputSlide":5,"sourceSlide":30,"narrativeRole":"性能指标","reuseMode":"duplicate-slide","editTargets":[]}
  ],
  "omittedSourceSlides": []
}
```

根据各源页布局JSON补齐具体 `shapeIds` 和 `action: rewrite/delete/replace`；50张未采用源页全部写入 `omittedSourceSlides`，理由统一为“本次五页机械臂控制专题叙事不需要该页面”。

- [ ] **Step 4: 验证页面映射**

Run:

```powershell
node "$skill\template_following_scripts\validate_template_plan.mjs" --workspace $ws --map "$ws\template-frame-map.json"
```

Expected: 映射验证通过，无缺失源页、无未解析编辑目标。

### Task 2: 生成模板继承起始稿

**Files:**
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/template-starter.pptx`
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/template-starter-preview/`

- [ ] **Step 1: 复制映射页面形成五页起始稿**

Run:

```powershell
node "$skill\template_following_scripts\prepare_template_starter_deck.mjs" --workspace $ws --pptx "$ws\source-template.pptx" --map "$ws\template-frame-map.json" --out "$ws\template-starter.pptx" --preview-dir "$ws\template-starter-preview" --layout-dir "$ws\template-starter-layout" --contact-sheet "$ws\template-starter-contact-sheet.png"
```

Expected: 输出五页 `template-starter.pptx`，预览图数量为5。

- [ ] **Step 2: 检查起始稿页面顺序**

依次确认：封面、研究内容、研究目标、技术路线、性能指标。若源页容量不足，只允许改映射并重新生成起始稿，不允许在页面上覆盖新布局。

### Task 3: 编辑五页继承元素并导出正式PPT

**Files:**
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/author-deck.mjs`
- Create: `outputs/20260716-pepper-arm-control-research/机械臂控制研究部分_五页版.pptx`

- [ ] **Step 1: 初始化artifact-tool工作区并阅读API**

Run:

```powershell
node "$skill\container_tools\setup_artifact_tool_workspace.mjs" --workspace $ws
Get-Content "$skill\artifact_tool\API_QUICK_START.md" -Raw
Get-Content "$skill\artifact_tool\api\API_DOCS.md" -Raw
```

Expected: `$ws/node_modules/@oai/artifact-tool` 可解析。

- [ ] **Step 2: 编写导入、元素替换和导出模块**

`author-deck.mjs` 使用以下结构，实际shape ID取自已验证的frame map：

```js
import { FileBlob, PresentationFile } from '@oai/artifact-tool';

const input = process.argv[2];
const output = process.argv[3];
const deck = await PresentationFile.importPptx(await FileBlob.load(input));

const copy = {
  1: ['面向钢筋捆扎作业的机械臂控制研究', '施工面姿态修正｜五阶段运动｜安全转场｜SDK执行预检'],
  2: ['研究内容：构建从目标点到安全执行的控制链路', '施工面姿态控制', '五阶段局部运动', '安全转场规划', '控制器预检执行'],
  3: ['研究目标：实现安全、准确、连续的跨点捆扎控制', '工具轴随施工面法向修正', 'MLIN保持局部直线轨迹', 'MJOINT完成安全点间转场', '全部目标运动前预检'],
  4: ['施工面与捆扎点', '姿态修正与五阶段计划', '安全转场与碰撞检查', 'SDK整任务预检', 'MJOINT/MLIN混合执行'],
  5: ['跨点完整捆扎周期≤5 s/点', '末端到位位置偏差≤5 mm', '工具姿态角偏差≤3°', '任务目标预检覆盖率100%']
};

// 仅通过frame map中的继承shape ID写入上述copy；
// 保持原字体、字号、段落、填充、线条、页码和模板装饰不变。

const out = await PresentationFile.exportPptx(deck);
await out.save(output);
```

- [ ] **Step 3: 运行导出**

Run:

```powershell
node "$ws\author-deck.mjs" "$ws\template-starter.pptx" "C:\Users\Administrator\Documents\efort_bot\outputs\20260716-pepper-arm-control-research\机械臂控制研究部分_五页版.pptx"
```

Expected: 正式PPTX存在且文件大小大于0。

### Task 4: 自动化与逐页视觉检查

**Files:**
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/preview/`
- Create: `%TEMP%/codex-presentations/019f5c97/pepper-arm-control-research/tmp/qa/qa-ledger.txt`

- [ ] **Step 1: 渲染五页正式稿**

Run:

```powershell
python "$skill\container_tools\render_slides.py" "C:\Users\Administrator\Documents\efort_bot\outputs\20260716-pepper-arm-control-research\机械臂控制研究部分_五页版.pptx"
```

Expected: 生成5张PNG。

- [ ] **Step 2: 检查画布溢出**

Run:

```powershell
python "$skill\container_tools\slides_test.py" "C:\Users\Administrator\Documents\efort_bot\outputs\20260716-pepper-arm-control-research\机械臂控制研究部分_五页版.pptx"
```

Expected: 无超出页面边界的对象。

- [ ] **Step 3: 按页检查并记录**

分别打开5张PNG原图检查：标题单行、正文无裁切、图形无意外遮挡、流程方向清晰、四项指标可读、页码和模板标识一致。将每页检查结果写入 `qa-ledger.txt`，发现问题后只修改对应继承元素并重新导出。

- [ ] **Step 4: 检查模板保真与空占位符**

Run:

```powershell
node "$skill\template_following_scripts\check_template_fidelity.mjs" --workspace $ws --starter-pptx "$ws\template-starter.pptx" --final-pptx "C:\Users\Administrator\Documents\efort_bot\outputs\20260716-pepper-arm-control-research\机械臂控制研究部分_五页版.pptx" --map "$ws\template-frame-map.json" --starter-layout-dir "$ws\template-starter-layout" --final-layout-dir "$ws\layout\final" --edit-dir $ws
```

Expected: 模板保真检查通过，无未处理的空结构占位符。

### Task 5: 最终交付核验

**Files:**
- Verify: `outputs/20260716-pepper-arm-control-research/机械臂控制研究部分_五页版.pptx`

- [ ] **Step 1: 核对页数和关键词**

检查最终deck为5页，并包含：`研究内容`、`研究目标`、`技术路线`、`≤5 s/点`、`≤5 mm`、`≤3°`、`100%`。

- [ ] **Step 2: 核对源文件未被覆盖**

对比源PPT路径、大小和修改时间，确认原文件未发生修改。

- [ ] **Step 3: 交付**

仅交付正式PPTX；临时脚本、审查JSON、预览图和QA记录保留在外部临时工作区，不作为用户交付物。
