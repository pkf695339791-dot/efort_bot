# 机械臂控制总结与甘特图 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 生成并验证一份项目组内部使用的机械臂控制阶段总结 PPT，以及一份覆盖 2026-07-14 至 2026-08-15 的可编辑 Excel 甘特图。

**Architecture:** 使用同一组经仓库证据核验的事实与计划数据驱动两份交付物。PowerPoint 采用 `@oai/artifact-tool` 的 Presentation API 从零创建，Excel 采用同一库的 Workbook API 创建；所有临时构建、预览和 QA 文件放在系统临时目录，最终文件放在仓库 `outputs/20260714-robot-arm-control-summary/`。

**Tech Stack:** JavaScript ES modules、Node.js、`@oai/artifact-tool`、PowerPoint `.pptx`、Excel `.xlsx`、演示文稿渲染与越界检查工具。

## Global Constraints

- 汇报受众为项目组内部，内容仅覆盖机械臂控制。
- 项目以 C++17 为基础，不描述 Python 原型或迁移过程。
- 不表述已完成机器人仿真或实机验证。
- dry-run 和单元测试只能表述为软件层离线验证。
- 7 月 20 日为现场测试里程碑，8 月 15 日为项目完成里程碑。
- PPT 为 16:9，深蓝灰、白色与埃夫特红工程技术风格。
- PPT 不使用墨斗 IDE 截图；主要视觉为原生图形、流程、日志摘要和数据。
- PPT 标题不小于 35 pt，正文不小于 16 pt；封面标题不小于 50 pt。
- Excel 必须保持日期和工期为可编辑的原生日期/公式，状态与甘特条通过条件格式联动。
- 不修改现有机械臂控制代码、配置和用户文件。

---

## File Map

- Create in temporary workspace: `build_gantt.mjs` — 创建、检查、渲染并导出甘特图工作簿。
- Create in temporary workspace: `build_presentation.mjs` — 创建、检查、渲染并导出 10 页演示文稿。
- Create: `outputs/20260714-robot-arm-control-summary/机械臂控制研究计划甘特图.xlsx` — 最终可编辑甘特图。
- Create: `outputs/20260714-robot-arm-control-summary/机械臂控制系统阶段工作总结与研究计划.pptx` — 最终汇报 PPT。
- Preserve: `README.md`, `cpp/**`, `samples/**`, `TIE_QUEUE_SIM.XPL` — 仅作为事实来源，不修改。

### Task 1: 固化事实与计划数据

**Files:**
- Read: `README.md`
- Read: `cpp/README.md`
- Read: `cpp/include/robot_control.hpp`
- Read: `cpp/tests/test_robot_control.cpp`
- Read: `build/Testing/Temporary/LastTest.log`
- Read: `samples/real_config.json`
- Read: `TIE_QUEUE_SIM.XPL`

**Interfaces:**
- Consumes: 设计说明 `docs/superpowers/specs/2026-07-14-robot-arm-control-summary-design.md`。
- Produces: 两个构建器共享的事实清单、测试数据和日程数组。

- [ ] **Step 1: 运行当前 C++ 单元测试以获得最新证据**

Run:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 1`，并出现 `All C++ robot control tests passed.`。

- [ ] **Step 2: 核对软件验证数据**

Run:

```powershell
rg -n "queue.size\(\) == 30|builder.build\(make_points\(20\)\)|make_points\(30\)|starts ==" cpp/tests/test_robot_control.cpp
```

Expected: 找到 30 点队列、60 点队列和 90 点队列的测试断言，以及 A/B 缓冲交替断言。

- [ ] **Step 3: 固定甘特图任务数组**

构建器使用以下数据，不添加未经确认的任务：

```js
const tasks = [
  ["T01", "C++工程、SDK接口和配置项复核", "机械臂控制", "2026-07-14", "2026-07-15", "现场测试基线版本", "进行中", "确认SDK版本、IP、工具及工件坐标"],
  ["T02", "XPL变量映射、双缓冲协议和停止逻辑复核", "机械臂控制", "2026-07-15", "2026-07-17", "接口检查表", "未开始", "核对PC_INT、PC_BOOL和PC_POINTC映射"],
  ["T03", "现场测试用例、安全检查、数据与环境准备", "现场联调", "2026-07-17", "2026-07-19", "测试包与检查清单", "未开始", "准备低速参数、回退条件和日志目录"],
  ["M01", "现场测试", "项目组", "2026-07-20", "2026-07-20", "现场测试记录", "里程碑", "通信、单点低速、三段运动、多点队列、异常停止"],
  ["T04", "测试问题分类与第一轮修复", "机械臂控制", "2026-07-21", "2026-07-24", "问题清单与修复版本", "未开始", "按安全、通信、坐标和运动分类"],
  ["T05", "运动逻辑及末端工具IO完善", "末端工具接口", "2026-07-25", "2026-07-31", "可联动控制版本", "未开始", "工具IO需要现场接口条件"],
  ["T06", "坐标转换、参数整定和接口联调", "现场联调", "2026-08-01", "2026-08-05", "参数基线与联调记录", "未开始", "固化工具、工件、姿态、速度和空间边界"],
  ["T07", "连续运行、边界工况和异常恢复验证", "测试记录", "2026-08-06", "2026-08-10", "稳定性测试记录", "未开始", "覆盖报警、急停、通信中断和越界点"],
  ["T08", "代码整理、操作说明和测试报告", "机械臂控制", "2026-08-11", "2026-08-13", "项目成果包", "未开始", "保证代码、配置、接口和记录一致"],
  ["M02", "内部验收与遗留问题清零", "项目组", "2026-08-14", "2026-08-14", "验收检查表", "里程碑", "仅保留已书面接受的遗留项"],
  ["M03", "项目完成", "项目组", "2026-08-15", "2026-08-15", "最终版本与归档材料", "里程碑", "完成版本、文档和测试记录归档"],
];
```

### Task 2: 创建并验证 Excel 甘特图

**Files:**
- Create in temporary workspace: `build_gantt.mjs`
- Create: `outputs/20260714-robot-arm-control-summary/机械臂控制研究计划甘特图.xlsx`

**Interfaces:**
- Consumes: Task 1 的 `tasks` 数组。
- Produces: 工作表 `研究计划`，任务表位于 `A5:I16`，日历从 `J5` 延伸至 `AP16`。

- [ ] **Step 1: 初始化 artifact-tool 临时工作区**

Run:

```powershell
$tmp = Join-Path $env:TEMP 'codex-presentations\manual-20260714\efort-control-summary\tmp'
& 'C:\Users\Administrator\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe' 'C:\Users\Administrator\.codex\plugins\cache\openai-primary-runtime\presentations\26.709.11516\skills\presentations\container_tools\setup_artifact_tool_workspace.mjs' --workspace $tmp
```

Expected: 临时目录中存在可解析 `@oai/artifact-tool` 的 `node_modules` 链接。

- [ ] **Step 2: 编写工作簿构建器**

构建器必须使用以下核心结构：

```js
import fs from "node:fs/promises";
import { SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const workbook = Workbook.create();
const sheet = workbook.worksheets.add("研究计划");
sheet.showGridLines = false;
sheet.freezePanes.freezeRows(5);
sheet.freezePanes.freezeColumns(9);
sheet.getRange("A1:AP1").merge();
sheet.getRange("A1").values = [["机械臂控制研究计划｜2026-07-14—2026-08-15"]];
sheet.getRange("A5:I5").values = [["编号", "任务", "负责人角色", "开始日期", "结束日期", "工期（天）", "交付物", "状态", "风险备注"]];
sheet.getRange("F6").formulas = [["=E6-D6+1"]];
sheet.getRange("F6:F16").fillDown();
sheet.getRange("J5").values = [[new Date("2026-07-14T00:00:00")]];
sheet.getRange("K5").formulas = [["=J5+1"]];
sheet.getRange("K5:AP5").fillRight();
sheet.getRange("J6:AP16").conditionalFormats.addCustom("=AND(J$5>=$D6,J$5<=$E6,$H6<>\"里程碑\")", { fill: "#2A9D8F" });
sheet.getRange("J6:AP16").conditionalFormats.addCustom("=AND(J$5=$D6,$H6=\"里程碑\")", { fill: "#E84A5F", font: { color: "#FFFFFF", bold: true } });
sheet.getRange("H6:H16").dataValidation = { rule: { type: "list", values: ["未开始", "进行中", "已完成", "里程碑"] } };
const output = await SpreadsheetFile.exportXlsx(workbook);
await output.save("C:/Users/Administrator/Documents/efort_bot/outputs/20260714-robot-arm-control-summary/机械臂控制研究计划甘特图.xlsx");
```

在此结构上补齐 11 行任务数据、摘要区、周末底色、三项里程碑说明、列宽、行高、字体、日期格式和浅色结构边框。

- [ ] **Step 3: 运行构建器并导出预览**

Run:

```powershell
& 'C:\Users\Administrator\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe' "$env:TEMP\codex-presentations\manual-20260714\efort-control-summary\tmp\build_gantt.mjs"
```

Expected: `.xlsx`、工作表 PNG 预览和关键范围检查输出均成功生成。

- [ ] **Step 4: 检查数值、公式和视觉输出**

构建器必须执行：

```js
console.log((await workbook.inspect({ kind: "table", range: "研究计划!A1:AP16", include: "values,formulas", tableMaxRows: 16, tableMaxCols: 42 })).ndjson);
console.log((await workbook.inspect({ kind: "match", searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A", options: { useRegex: true, maxResults: 100 }, summary: "final formula error scan" })).ndjson);
const preview = await workbook.render({ sheetName: "研究计划", range: "A1:AP16", scale: 1.25, format: "png" });
await fs.writeFile(previewPath, new Uint8Array(await preview.arrayBuffer()));
```

Expected: 无公式错误；日期覆盖 2026-07-14 至 2026-08-15；11 个任务/里程碑全部可见；冻结窗格和条件格式存在。

### Task 3: 创建并验证 10 页 PowerPoint

**Files:**
- Create in temporary workspace: `build_presentation.mjs`
- Create: `outputs/20260714-robot-arm-control-summary/机械臂控制系统阶段工作总结与研究计划.pptx`

**Interfaces:**
- Consumes: Task 1 的事实、测试数据和 Task 2 的日程。
- Produces: 10 页、16:9、可编辑的 `.pptx`，以及临时目录中的逐页 PNG、布局 JSON 和整套预览。

- [ ] **Step 1: 编写演示文稿构建器**

使用以下基础对象和可复用函数，所有视觉均为原生可编辑对象：

```js
import fs from "node:fs/promises";
import { Presentation, PresentationFile } from "@oai/artifact-tool";

const deck = Presentation.create({ slideSize: { width: 1280, height: 720 } });
const COLORS = { navy: "#14213D", ink: "#1F2937", paper: "#F7F8FA", red: "#E84A5F", teal: "#2A9D8F", orange: "#F4A261", line: "#D7DCE3" };
function addText(slide, text, position, style) {
  const box = slide.shapes.add({ geometry: "textbox", position, fill: "none", line: { style: "solid", fill: "none", width: 0 } });
  box.text = text;
  box.text.style = style;
  return box;
}
function addTitle(slide, title, kicker) {
  addText(slide, kicker, { left: 72, top: 48, width: 420, height: 28 }, { fontSize: 16, bold: true, color: COLORS.red });
  addText(slide, title, { left: 72, top: 84, width: 1136, height: 54 }, { fontSize: 38, bold: true, color: COLORS.navy });
}
```

- [ ] **Step 2: 按确认的叙事创建 10 页**

每页主结论固定如下：

```js
const slideClaims = [
  "机械臂控制系统阶段工作总结与研究计划",
  "当前目标是让绑扎点安全、连续地转化为机械臂动作",
  "C++控制链路已形成完整的软件实现边界",
  "双缓冲协议把上位机队列与控制器执行解耦",
  "每个绑扎点展开为接近、绑扎、撤离三段动作",
  "安全检查在点位进入执行队列之前拦截风险",
  "软件层已覆盖30、60、90点队列与缓冲切换",
  "当前关键差距集中在现场通信、坐标和工具联动",
  "7月20日现场测试按逐级放行控制风险",
  "8月15日前完成问题闭环、稳定性验证和成果归档",
];
```

第 4 页的连接器先于节点创建；第 7 页明确标注“软件层离线验证”；第 8 页所有实机相关项标注“待现场验证”；第 9 页包含停止升级与回退规则；第 10 页突出 7 月 20 日与 8 月 15 日里程碑。

- [ ] **Step 3: 导出逐页预览、布局和最终 PPTX**

```js
for (const [index, slide] of deck.slides.items.entries()) {
  const stem = `slide-${String(index + 1).padStart(2, "0")}`;
  const png = await deck.export({ slide, format: "png", scale: 1.5 });
  await fs.writeFile(`${previewDir}/${stem}.png`, new Uint8Array(await png.arrayBuffer()));
  const layout = await slide.export({ format: "layout" });
  await fs.writeFile(`${layoutDir}/${stem}.layout.json`, await layout.text());
}
const montage = await deck.export({ format: "webp", montage: true, scale: 1 });
await fs.writeFile(`${qaDir}/deck-montage.webp`, new Uint8Array(await montage.arrayBuffer()));
const pptx = await PresentationFile.exportPptx(deck);
await pptx.save(finalPptx);
```

Expected: 10 张逐页 PNG、10 个布局 JSON、1 个整套预览和最终 `.pptx`。

- [ ] **Step 4: 执行自动布局检查**

Run:

```powershell
& 'C:\Users\Administrator\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' 'C:\Users\Administrator\.codex\plugins\cache\openai-primary-runtime\presentations\26.709.11516\skills\presentations\container_tools\slides_test.py' 'C:\Users\Administrator\Documents\efort_bot\outputs\20260714-robot-arm-control-summary\机械臂控制系统阶段工作总结与研究计划.pptx'
```

Expected: 无越界错误；布局 JSON 中无非预期重叠。

- [ ] **Step 5: 逐页全尺寸视觉检查并修正**

逐页检查标题是否换行、正文是否低于 16 pt、连接线是否穿过节点、图形是否遮挡文字、颜色语义是否一致、未验证内容是否被误写为已完成。发现问题时只修改 `build_presentation.mjs` 并重新运行，直到所有 10 页通过。

### Task 4: 最终一致性与交付检查

**Files:**
- Verify: `outputs/20260714-robot-arm-control-summary/机械臂控制研究计划甘特图.xlsx`
- Verify: `outputs/20260714-robot-arm-control-summary/机械臂控制系统阶段工作总结与研究计划.pptx`

**Interfaces:**
- Consumes: Task 2 和 Task 3 的最终文件。
- Produces: 可交付的 PPT 与 Excel，且两者里程碑、日期和状态表述一致。

- [ ] **Step 1: 检查两个最终文件存在且非空**

Run:

```powershell
Get-Item outputs\20260714-robot-arm-control-summary\*.pptx, outputs\20260714-robot-arm-control-summary\*.xlsx | Select-Object Name,Length,LastWriteTime
```

Expected: 两个文件大小均大于 0，修改时间为当前制作时间。

- [ ] **Step 2: 检查禁止性表述**

在构建器源数据和演示文稿 inspect 输出中搜索：

```powershell
rg -n "Python原型|迁移完成|已完成仿真|已完成实机|仿真验证通过|实机验证通过" "$env:TEMP\codex-presentations\manual-20260714\efort-control-summary\tmp"
```

Expected: 0 个匹配。

- [ ] **Step 3: 检查日期一致性**

确认 PPT 第 9 页与 Excel 的现场测试日期均为 2026-07-20，PPT 第 10 页与 Excel 的项目完成日期均为 2026-08-15。

- [ ] **Step 4: 运行最终 C++ 回归测试**

Run:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 1`，证明交付物制作未影响现有工程。

- [ ] **Step 5: 仅提交最终交付物和计划文件（如用户要求提交）**

```powershell
git status --short
```

Expected: 不暂存或提交任何无关的用户文件；最终回复仅提供 `.pptx` 和 `.xlsx` 链接。
