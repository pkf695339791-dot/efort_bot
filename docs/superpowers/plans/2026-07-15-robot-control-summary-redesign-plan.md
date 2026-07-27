# 机械臂控制总结 PPT 全面重做实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 基于现有“运动控制版”视觉体系，制作一份11页、以机械臂控制能力为主线的项目组内部阶段总结与研究计划PPT。

**Architecture:** 将现有PPT作为唯一视觉参考，通过 artifact-tool 导入、检查、复制合适的源页面并编辑继承元素。内容采用“目标与现状—控制能力—验证证据—能力边界—下一步计划”的叙事，最终另存为独立PPTX并逐页渲染检查。

**Tech Stack:** PowerPoint PPTX、`@oai/artifact-tool` JavaScript ES modules、模板跟随脚本、PowerShell、PNG/layout渲染检查。

## Global Constraints

- 面向项目组内部，不写成对外宣传材料。
- 项目控制软件以 C++17 为基础，不出现 Python 原型内容。
- 以现有“机械臂控制系统阶段工作总结与研究计划_运动控制版.pptx”为唯一视觉参考。
- 最终文件不得覆盖源PPT。
- 明确区分 dry-run、单元测试、机器人仿真和实机验证。
- 不得声称已完成现场标定、碰撞检测、完整避障、机器人仿真、绑扎IO或实机验证。
- 现场测试里程碑为2026年7月20日，项目完成目标为2026年8月15日前。
- 最终输出固定为 `outputs/20260715-robot-arm-control-summary/机械臂控制系统阶段工作总结与研究计划_全面重做版.pptx`。

---

## File Structure

- Read: `outputs/20260714-robot-arm-control-summary/机械臂控制系统阶段工作总结与研究计划_运动控制版.pptx`：视觉参考和可复用页面。
- Read: `docs/superpowers/specs/2026-07-15-robot-control-summary-redesign.md`：内容与边界规范。
- Read: `README.md`、`cpp/README.md`、`TIE_QUEUE_SIM_README.md`：已实现功能和安全边界证据。
- Create under `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/`：模板检查、页面映射、编辑脚本、预览、layout和QA记录。
- Create: `outputs/20260715-robot-arm-control-summary/机械臂控制系统阶段工作总结与研究计划_全面重做版.pptx`：最终交付文件。

---

### Task 1: 检查源PPT并建立页面继承规则

**Files:**
- Read: `outputs/20260714-robot-arm-control-summary/机械臂控制系统阶段工作总结与研究计划_运动控制版.pptx`
- Create: `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/template-inspect/**`
- Create: `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/template-audit.txt`

**Interfaces:**
- Consumes: 源PPTX。
- Produces: 每页PNG、layout JSON、元素ID清单、字体/配色/页脚审计。

- [ ] **Step 1: 初始化 artifact-tool 工作区**

运行：

```powershell
$skill = 'C:\Users\Administrator\.codex\plugins\cache\openai-primary-runtime\presentations\26.709.11516\skills\presentations'
$tmp = Join-Path $env:TEMP 'codex-presentations\019f5c97\robot-control-summary-redesign\tmp'
$node = 'C:\Users\Administrator\.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe'
New-Item -ItemType Directory -Force -Path $tmp | Out-Null
& $node "$skill\container_tools\setup_artifact_tool_workspace.mjs" --workspace $tmp
```

预期：`$tmp/node_modules/@oai/artifact-tool` 可解析。

- [ ] **Step 2: 复制源PPT到ASCII临时路径并完整检查**

将源文件复制为 `$tmp/source-motion-control.pptx`。运行模板检查脚本；Windows若缺少`unzip`，在临时副本中把检查脚本的`unzip -Z1/-p`调用等价替换为系统`tar -tf/-xOf`，不得修改安装目录内的技能文件。

```powershell
Copy-Item -LiteralPath 'outputs\20260714-robot-arm-control-summary\机械臂控制系统阶段工作总结与研究计划_运动控制版.pptx' -Destination "$tmp\source-motion-control.pptx" -Force
& $node "$skill\template_following_scripts\inspect_template_deck.mjs" --workspace $tmp --pptx "$tmp\source-motion-control.pptx"
```

预期：生成全部源页面PNG、layout JSON、`template-inspect.ndjson`和`template-manifest.json`。

- [ ] **Step 3: 审阅全部源页面并写模板审计**

`template-audit.txt`必须记录：页面数量、16:9尺寸、标题/正文字体、主色/强调色、页脚和页码规则、可复用的封面/章节/双栏/流程/时间表页面，以及所有空结构占位符。

- [ ] **Step 4: 生成源PPT总览图并逐页查看**

使用源页面PNG生成montage，同时逐张查看原始尺寸页面。预期：11页输出都能找到可继承的源页面结构；若没有，停止并报告最接近的源页面，不得叠加自制平行模板。

---

### Task 2: 建立11页内容映射与模板starter

**Files:**
- Create: `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/template-frame-map.json`
- Create: `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/deviation-log.txt`
- Create: `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/template-starter.pptx`

**Interfaces:**
- Consumes: Task 1 的源页面ID和可编辑元素ID。
- Produces: 11页继承式starter PPTX。

- [ ] **Step 1: 写入11页内容清单**

页面标题和主结论固定为：

```text
1 机械臂控制阶段工作总结与研究计划
2 控制目标已从“到达点位”扩展为“安全接近并保持施工姿态”
3 C++控制闭环已经形成，现场验证与碰撞建模仍待完成
4 控制链路覆盖规划、SDK预检与控制器混合运动执行
5 工具姿态随施工面法向修正，倾斜面运动不再局限于Z轴
6 每个绑扎点按五阶段完成进入、接近、作业、撤离和退出
7 安全点间采用MJOINT转移，施工面附近采用MLIN约束轨迹
8 全部目标在首批运动下发前完成CheckTarget与IkSolver预检
9 水平面与30°倾斜面dry-run验证了法向偏移和姿态变化
10 当前方案尚不等于完整避障，也尚未完成仿真或实机验证
11 7月20日完成现场测试，8月15日前完成闭环收尾
```

- [ ] **Step 2: 创建模板页面映射**

为每个输出页选择一个源页，`reuseMode`必须是`duplicate-slide`；每个被改写对象必须填写Task 1得到的`shapeId/sourceElementId`，空占位符必须明确`rewrite`或`delete`。源页未使用时写入`omittedSourceSlides`及原因。

- [ ] **Step 3: 验证映射并生成starter**

```powershell
& $node "$skill\template_following_scripts\validate_template_plan.mjs" --workspace $tmp --map "$tmp\template-frame-map.json"
& $node "$skill\template_following_scripts\prepare_template_starter_deck.mjs" --workspace $tmp --pptx "$tmp\source-motion-control.pptx" --map "$tmp\template-frame-map.json" --out "$tmp\template-starter.pptx" --preview-dir "$tmp\template-starter-preview" --layout-dir "$tmp\template-starter-layout" --contact-sheet "$tmp\template-starter-contact-sheet.png"
```

预期：映射校验通过，starter为11页，无未处理结构占位符。

---

### Task 3: 编辑11页继承元素并导出新版PPT

**Files:**
- Create: `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/build-redesign.mjs`
- Create: `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/source-notes.txt`
- Create: `outputs/20260715-robot-arm-control-summary/机械臂控制系统阶段工作总结与研究计划_全面重做版.pptx`

**Interfaces:**
- Consumes: `template-starter.pptx`和元素ID映射。
- Produces: 最终11页PPTX、逐页PNG和layout JSON。

- [ ] **Step 1: 写入可核查内容证据**

`source-notes.txt`记录以下事实来源：五阶段运动和姿态修正来自`cpp/README.md`；PointC/PointJ、错误码和控制器变量来自`TIE_QUEUE_SIM_README.md`；水平/倾斜面结果和2/2测试来自本次已执行的构建及dry-run日志。

- [ ] **Step 2: 用artifact-tool编辑继承对象**

`build-redesign.mjs`必须：导入starter；按frame map中的元素ID调用`presentation.resolve(id)`；只改写列入`editTargets`的文本、表格或图形；保持原字体、字号、颜色、行距和边距；导出每页PNG/layout、montage和PPTX。

核心导出结构：

```javascript
import fs from 'node:fs/promises';
import { FileBlob, PresentationFile } from '@oai/artifact-tool';

const presentation = await PresentationFile.importPptx(
  await FileBlob.load(process.env.STARTER_PPTX),
);
const editPlan = JSON.parse(await fs.readFile(process.env.EDIT_PLAN, 'utf8')).edits;
await fs.mkdir(process.env.PREVIEW_DIR, { recursive: true });
await fs.mkdir(process.env.LAYOUT_DIR, { recursive: true });

// editPlan由已验证的template-frame-map.json生成，全部ID来自源PPT检查。
for (const edit of editPlan) {
  const target = presentation.resolve(edit.id);
  if (edit.action === 'rewrite') target.text = edit.text;
}

for (const [index, slide] of presentation.slides.items.entries()) {
  const stem = `slide-${String(index + 1).padStart(2, '0')}`;
  await fs.writeFile(`${process.env.PREVIEW_DIR}/${stem}.png`,
    new Uint8Array(await (await presentation.export({ slide, format: 'png', scale: 1 })).arrayBuffer()));
  await fs.writeFile(`${process.env.LAYOUT_DIR}/${stem}.layout.json`,
    await (await slide.export({ format: 'layout' })).text());
}

const pptx = await PresentationFile.exportPptx(presentation);
await pptx.save(process.env.FINAL_PPTX);
```

- [ ] **Step 3: 页面内容要求**

- 第5页必须用倾斜面实例说明：法向`[0.5,0,0.866]`时，Tie为`(120,-80,260)`，Approach为`(145,-80,303.301)`，SafeEntry为`(195,-80,389.904)`，B角为30°。
- 第6页必须展示五阶段及运动类型：`MJOINT SafeEntry`、四个`MLIN`局部段。
- 第8页必须展示“完整任务生成→CheckTarget全部目标→IkSolver仅MJOINT→任一失败则零批次下发”。
- 第9页只能表述为dry-run和单元测试证据，不得使用“仿真通过”。
- 第10页必须明确列出现场标定、碰撞模型、障碍物、绑扎IO、仿真、实机验证六项边界。
- 第11页甘特图范围为7月15日至8月15日，突出7月20日现场测试里程碑。

- [ ] **Step 4: 导出PPTX**

预期：最终PPTX存在，11页均可编辑，源PPT保持不变。

---

### Task 4: 全量视觉QA与交付

**Files:**
- Inspect: `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/preview/*.png`
- Inspect: `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/layout/final/*.json`
- Create: `%TEMP%/codex-presentations/019f5c97/robot-control-summary-redesign/tmp/qa-ledger.txt`
- Verify: `outputs/20260715-robot-arm-control-summary/机械臂控制系统阶段工作总结与研究计划_全面重做版.pptx`

**Interfaces:**
- Consumes: Task 3的最终PPTX和渲染文件。
- Produces: 无溢出、无空占位符、可交付的最终PPTX。

- [ ] **Step 1: 逐页原尺寸检查**

逐张检查11页PNG，`qa-ledger.txt`为每页记录标题单行、正文无裁切、层级清晰、数据准确、页脚一致、无意外遮挡。发现问题必须回到Task 3缩短文案或改用更合适的源页面，不得缩小到16pt以下。

- [ ] **Step 2: 运行溢出与模板一致性检查**

```powershell
$python = 'C:\Users\Administrator\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
& $python "$skill\container_tools\slides_test.py" $final
& $node "$skill\template_following_scripts\check_template_fidelity.mjs" --workspace $tmp --starter-pptx "$tmp\template-starter.pptx" --final-pptx $final --map "$tmp\template-frame-map.json" --starter-layout-dir "$tmp\template-starter-layout" --final-layout-dir "$tmp\layout\final" --edit-dir $tmp
```

预期：无画布溢出、无未处理空占位符、模板一致性检查通过。

- [ ] **Step 3: 内容边界扫描**

检查最终PPT文本不得包含：`Python原型`、`仿真验证通过`、`实机验证通过`、`已完成自动避障`、`碰撞检测完成`。允许的准确表述是“dry-run通过”“单元测试2/2通过”“尚未完成仿真或实机验证”。

- [ ] **Step 4: 最终交付**

提供最终PPTX链接，并用新版渲染结果说明重点更新页；不交付临时脚本、布局JSON或QA文件。
