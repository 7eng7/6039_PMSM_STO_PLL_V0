param(
    [string]$ProjectRoot = 'E:\Electric Motor Project\6039_PMSM_STO_PLL_V0',
    [string]$VaultRoot = 'D:\obsidian笔记文件存放\JS的知识库\Q电驱项目\6039_PMSM_STO_PLL_V0_系统解读'
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$Utf8NoBom = New-Object System.Text.UTF8Encoding($false)
$KeilRoot = Join-Path $ProjectRoot 'Keil'
$ObjectRoot = Join-Path $KeilRoot 'Objects'
$PackRoot = 'D:\Keil_v5\ARM\Packs\ETMCU\ET6039_DFP\0.2.4'

function Write-Utf8File([string]$Path, [string]$Content) {
    $parent = Split-Path -Parent $Path
    if (-not (Test-Path -LiteralPath $parent)) {
        New-Item -ItemType Directory -Path $parent -Force | Out-Null
    }
    [System.IO.File]::WriteAllText($Path, $Content, $Utf8NoBom)
}

function Normalize-FullPath([string]$Path) {
    if ($Path.StartsWith('..\')) {
        return [System.IO.Path]::GetFullPath((Join-Path $KeilRoot $Path))
    }
    return $Path
}

function Get-Layer([string]$Path) {
    if ($Path.StartsWith((Join-Path $ProjectRoot 'Application'), [System.StringComparison]::OrdinalIgnoreCase)) { return 'Application' }
    if ($Path.StartsWith((Join-Path $ProjectRoot 'MCSDK_v5.4.4-Full'), [System.StringComparison]::OrdinalIgnoreCase)) { return 'MCSDK' }
    if ($Path.StartsWith($PackRoot, [System.StringComparison]::OrdinalIgnoreCase)) { return 'ET6039_DFP' }
    if ($Path -match '\\CMSIS\\') { return 'CMSIS' }
    return 'Other'
}

function Get-RelativeCodePath([string]$Path, [string]$Layer) {
    switch ($Layer) {
        'Application' { return $Path.Substring((Join-Path $ProjectRoot 'Application').Length).TrimStart('\') }
        'MCSDK' { return $Path.Substring((Join-Path $ProjectRoot 'MCSDK_v5.4.4-Full').Length).TrimStart('\') }
        'ET6039_DFP' { return $Path.Substring($PackRoot.Length).TrimStart('\') }
        'CMSIS' {
            $i = $Path.IndexOf('CMSIS', [System.StringComparison]::OrdinalIgnoreCase)
            return $Path.Substring($i)
        }
        default { return Split-Path -Leaf $Path }
    }
}

function Get-NoteRelativePath([string]$Path) {
    $layer = Get-Layer $Path
    $relative = Get-RelativeCodePath $Path $layer
    return ('04-逐文件解读/{0}/{1}.md' -f $layer, ($relative -replace '\\','/'))
}

function Get-WikiLink([string]$Path, [hashtable]$NoteByPath) {
    $key = $Path.ToLowerInvariant()
    if (-not $NoteByPath.ContainsKey($key)) { return ('`{0}`' -f (Split-Path -Leaf $Path)) }
    $note = $NoteByPath[$key] -replace '\\','/' -replace '\.md$',''
    return ('[[{0}|{1}]]' -f $note, (Split-Path -Leaf $Path))
}

function Get-FileSummary([string]$Name, [string]$Layer, [string]$Text) {
    $known = @{
        'main.c'='应用入口。完成ET6039板级初始化、启动Motor Control中间件，并在前台循环处理按键和低实时性的人机交互；真正的FOC不在while循环中执行。'
        'init_config.c'='ET6039板级外设初始化核心：配置ADC虚拟通道、SRPWM时基与互补输出、GPIO复用、UART和中断，是软件变量落到控制板J10引脚的桥梁。'
        'mc_it.c'='中断入口适配层。把ADC/PWM、SysTick和串口硬件中断转交给MCSDK实时任务或通信状态机，并在异常时保证功率级进入安全状态。'
        'mc_tasks.c'='整套电机控制的调度与状态机中心：组织高频FOC、中频启动与速度环、安全检查、故障迁移以及开环到STO-PLL闭环的切换。'
        'mc_config.c'='静态对象装配文件。用参数宏实例化速度/电流PI、STO-PLL、虚拟速度传感器、Rev-Up、PWM电流反馈、母线电压、温度和状态机对象。'
        'mc_parameters.c'='与采样拓扑直接相关的查表配置，描述不同SVPWM扇区下应读取的ADC通道和结果寄存器；配置错误会直接造成相电流符号或重构错误。'
        'mc_math.c'='FOC数学基础：Clarke、Park、反Park变换以及定点正余弦计算，把三相静止量转换为转子同步d-q量。'
        'mc_api.c'='提供给main和上层应用的简化控制API，如启动、停止、速度斜坡、状态和故障读取。调用通常只是提交命令，实际执行由中频任务异步完成。'
        'mc_interface.c'='应用命令与实时状态机之间的命令邮箱，负责缓存启动、停止、速度/转矩斜坡和故障确认请求，并报告命令执行状态。'
        'motorcontrol.c'='Motor Control中间件总初始化入口，建立系统节拍、调用MCboot并启动UI任务。'
        'pid_regulator.c'='MCSDK定点PI/PID实现，包含比例、积分、微分、积分限幅、输出饱和和防积分饱和逻辑；速度环与d/q电流环都依赖它。'
        'speed_torq_ctrl.c'='速度/转矩控制器。速度模式下由速度误差经PI生成Iq参考，转矩模式下直接使用转矩电流参考，并管理目标斜坡。'
        'revup_ctrl.c'='无感启动多阶段斜坡控制器，在反电势尚不可观测时给出虚拟速度和转矩电流，并判断STO估算是否已收敛。'
        'virtual_speed_sensor.c'='开环启动使用的虚拟速度与角度积分器，根据给定机械加速度生成连续电角度。'
        'sto_pll_speed_pos_fdbk.c'='无感核心：用电机模型从定子电压、电流估算反电势，再用PLL锁定反电势角度，输出电角度、机械速度和可靠性判据。'
        'state_machine.c'='电机运行状态机与故障锁存器，限制IDLE、START、RUN、STOP、FAULT等状态之间的合法迁移。'
        'pwm_curr_fdbk.c'='与具体MCU无关的PWM/相电流反馈基类，计算SVPWM扇区、三相比较值和采样点，并通过函数指针调用ET6039适配实现。'
        'r3_2_f4xx_pwm_curr_fdbk.c'='本工程关键硬件适配层。名称保留F4xx历史命名，但实现已改为ET6039三电阻、双ADC同步采样，负责零偏校准、相电流重构、PWM开关和采样窗口安排。'
        'circle_limitation.c'='电压矢量限幅器。限制Vd、Vq合成幅值不超过当前PWM和母线电压可实现的范围，避免SVPWM过调制破坏电流环。'
        'r_divider_bus_voltage_sensor.c'='母线分压采样和数字滤波，依据阈值与滞回判断欠压、过压，为状态机提供电源故障。'
        'ntc_temperature_sensor.c'='NTC温度采样、滤波、过温阈值和恢复滞回处理。'
        'motor_control_protocol.c'='调试通信寄存器映射层，把上位机命令映射到速度、PI参数、状态、故障和观测变量。'
        'user_interface.c'='通用UI抽象，负责读写MCSDK控制对象的寄存器式接口。'
        'ui_task.c'='非实时通信任务，初始化串口协议并按较低频率处理超时、收发和用户界面。'
        'usart_frame_communication_protocol.c'='串口逐字节收发状态机，负责非阻塞帧接收、发送和超时管理。'
        'frame_communication_protocol.c'='帧通信协议基类，定义帧缓冲、校验、确认和错误响应机制。'
        'et6x_adc.c'='ET6039 ADC底层驱动：配置真实/虚拟通道、触发源、队列、结果寄存器和中断。电流、母线电压、温度采样最终都落到这里。'
        'et6x_srpwm.c'='ET6039 SRPWM底层驱动：配置中心对齐时基、比较值、动作限定、死区、同步和事件触发，是六路功率PWM的最底层软件入口。'
        'et6x_gpio.c'='ET6039 GPIO与引脚复用驱动，控制J10相关数字信号、关断脚、按键和LED。'
        'et6x_interrupt.c'='ET6039中断注册、使能和优先级封装。实时性是否正确与这里及NVIC配置直接相关。'
        'et6x_uart.c'='ET6039 UART底层驱动，为调试协议和实时转速输出提供字节收发能力。'
        'system_et6x.c'='芯片系统时钟和启动阶段配置，决定CPU及外设时钟基准；PWM、控制周期和串口波特率换算都依赖其正确性。'
        'startup_et6039.s'='ET6039复位入口与中断向量表，初始化栈和数据段后跳转到C运行库/main；所有IRQ入口的最上游。'
    }
    if ($known.ContainsKey($Name)) { return $known[$Name] }
    if ($Name -match '^et6x_(.+)\.c$') { return "ET6039设备驱动包中的$($Matches[1].ToUpper())外设底层实现，向板级初始化和实时控制层提供寄存器级封装。" }
    if ($Name.EndsWith('.h')) { return '接口与配置头文件。集中声明数据结构、宏、枚举、函数原型或硬件映射，供对应实现文件及其调用者共享。' }
    if ($Layer -eq 'MCSDK') { return 'MCSDK控制组件。该文件通过对象句柄和函数指针参与电机控制链，具体位置请结合依赖关系与调用者阅读。' }
    if ($Layer -eq 'ET6039_DFP') { return 'ET6039设备支持包文件，负责芯片寄存器、启动代码或外设驱动封装。' }
    return '工程实际构建依赖中的代码文件。请结合本页符号表、依赖和调用关系阅读。'
}

function Get-FunctionMeaning([string]$Name) {
    if ($Name -match 'IRQHandler|IRQ_Handler|Handler$') { return '中断入口；要求短小、确定、不可阻塞，通常清标志并转交实时处理。' }
    if ($Name -match '^(.*_)?Init|^MX_.*Init|^MCboot$') { return '初始化对象或硬件，使其进入已知但通常尚未驱动电机的状态。' }
    if ($Name -match 'Start|TurnOn|SwitchOn|Enable') { return '启动或使能相关功能；需要结合前置状态和保护条件确认是否立即生效。' }
    if ($Name -match 'Stop|TurnOff|SwitchOff|Disable|Emergency') { return '停止或关断相关功能；功率级相关函数需要关注PWM与驱动使能脚的先后顺序。' }
    if ($Name -match 'Get|Read|Calc|Compute|Estimate|Observer') { return '读取、计算或估算数据；检查输入单位、定点缩放、符号和返回值有效条件。' }
    if ($Name -match 'Set|Write|Program|Exec|Command') { return '设置参数或提交命令；部分接口是异步邮箱，并不在调用点立即完成动作。' }
    if ($Name -match 'Clear|Reset') { return '清除状态或复位内部历史量；运行中调用可能造成控制量跳变。' }
    if ($Name -match 'Check|Is|Reliab|Fault|Error') { return '执行状态、可靠性或故障判定，结果通常影响状态机和PWM使能。' }
    if ($Name -match 'Task|Run|Update|Exec') { return '周期任务或状态更新函数，其实际执行频率决定离散控制器行为。' }
    return '完成该模块的一项内部或公开操作；应从调用者、参数单位和被修改的句柄字段三方面理解。'
}

function Get-HardwareRelations([string]$Text) {
    $items = New-Object System.Collections.Generic.List[string]
    if ($Text -match 'SRPWM|PWM_') { $items.Add('**SRPWM/PWM**：与三相U/V/W六路门极控制、中心对齐载波、死区或ADC触发有关。') }
    if ($Text -match '\bADC|VIRTUAL_CH|CURRENT_CH') { $items.Add('**ADC**：与相电流、24 V母线分压、温度等模拟量的触发和结果寄存器有关。') }
    if ($Text -match '\bGPIO|Pin|PIN') { $items.Add('**GPIO/引脚复用**：可能连接控制板按键、LED、J10控制线或驱动板关断/故障信号。') }
    if ($Text -match '\bUART|USART|FCP') { $items.Add('**UART**：与上位机、VOFA或MCSDK调试协议通信有关，不属于高频FOC链。') }
    if ($Text -match '\bACMP|Comparator') { $items.Add('**模拟比较器**：通常用于快速阈值或过流保护，响应路径可能独立于软件ADC。') }
    if ($Text -match '\bEQEP|ENCODER|HALL') { $items.Add('**位置传感接口**：涉及编码器、霍尔或eQEP；本STO-PLL主速度反馈为无感估算，不能混为一谈。') }
    if ($Text -match '\bXBAR') { $items.Add('**XBAR触发路由**：决定PWM事件、ADC触发和保护信号在片内如何互连。') }
    if ($Text -match '\bCRG|Clock|CLOCK|SystemCoreClock') { $items.Add('**时钟树**：影响CPU、PWM、ADC和通信时基，参数换算必须与真实时钟一致。') }
    if ($items.Count -eq 0) { return '本文件不直接操作板级外设，主要通过上层控制对象或函数指针间接影响硬件。' }
    return ($items -join "`n`n")
}

# 从编译器依赖文件恢复实际参与构建的源码/头文件闭包。
$all = New-Object System.Collections.Generic.HashSet[string]([System.StringComparer]::OrdinalIgnoreCase)
Get-ChildItem $ObjectRoot -Filter '*.d' | ForEach-Object {
    $raw = (Get-Content $_.FullName -Raw) -replace '\\\r?\n',' '
    [regex]::Matches($raw, '(?:(?:[A-Za-z]:\\|\.\.\\)[^\s]+\.(?:c|h|s))') | ForEach-Object {
        $path = Normalize-FullPath $_.Value
        if (Test-Path -LiteralPath $path) { [void]$all.Add($path) }
    }
}
$startup = Join-Path $ProjectRoot 'Keil\RTE\Device\ET6039\startup_et6039.s'
if (Test-Path $startup) { [void]$all.Add($startup) }
$files = @($all | Sort-Object)

$NoteByPath = @{}
foreach ($file in $files) { $NoteByPath[$file.ToLowerInvariant()] = Get-NoteRelativePath $file }

# basename索引用于解析#include；同名时优先Application，再MCSDK，再设备包。
$ByBaseName = @{}
foreach ($file in $files) {
    $base = (Split-Path -Leaf $file).ToLowerInvariant()
    if (-not $ByBaseName.ContainsKey($base)) { $ByBaseName[$base] = New-Object System.Collections.Generic.List[string] }
    $ByBaseName[$base].Add($file)
}

$DirectDeps = @{}
$ReverseDeps = @{}
$Definitions = @{}
$AllFunctionOwners = @{}
foreach ($file in $files) {
    $key = $file.ToLowerInvariant()
    $text = Get-Content -LiteralPath $file -Raw -Encoding UTF8
    $deps = New-Object System.Collections.Generic.List[string]
    [regex]::Matches($text, '(?m)^\s*#\s*include\s*[<"]([^>"]+)[>"]') | ForEach-Object {
        $base = (Split-Path -Leaf $_.Groups[1].Value).ToLowerInvariant()
        if ($ByBaseName.ContainsKey($base)) {
            $candidates = @($ByBaseName[$base] | Sort-Object { switch (Get-Layer $_) {'Application'{0};'MCSDK'{1};'ET6039_DFP'{2};default{3}} })
            $dep = $candidates[0]
            if (-not $deps.Contains($dep)) { $deps.Add($dep) }
        }
    }
    $DirectDeps[$key] = @($deps)
    foreach ($dep in $deps) {
        $dk = $dep.ToLowerInvariant()
        if (-not $ReverseDeps.ContainsKey($dk)) { $ReverseDeps[$dk] = New-Object System.Collections.Generic.List[string] }
        $ReverseDeps[$dk].Add($file)
    }

    $defs = New-Object System.Collections.Generic.List[string]
    $pattern = '(?m)^\s*(?:(?:__WEAK|__STATIC_INLINE|static|inline)\s+)*(?:const\s+)?(?:[A-Za-z_]\w*(?:\s*\*)?\s+)+([A-Za-z_]\w*)\s*\([^;{}]*\)\s*\{'
    [regex]::Matches($text, $pattern) | ForEach-Object {
        $name = $_.Groups[1].Value
        if ($name -notin @('if','for','while','switch')) {
            if (-not $defs.Contains($name)) { $defs.Add($name) }
            if (-not $AllFunctionOwners.ContainsKey($name)) { $AllFunctionOwners[$name] = New-Object System.Collections.Generic.List[string] }
            $AllFunctionOwners[$name].Add($file)
        }
    }
    $Definitions[$key] = @($defs)
}

New-Item -ItemType Directory -Path $VaultRoot -Force | Out-Null

$indexRows = New-Object System.Collections.Generic.List[string]
$layerCounts = @{}
foreach ($file in $files) {
    $key = $file.ToLowerInvariant()
    $layer = Get-Layer $file
    if (-not $layerCounts.ContainsKey($layer)) { $layerCounts[$layer] = 0 }
    $layerCounts[$layer]++
    $relative = Get-RelativeCodePath $file $layer
    $name = Split-Path -Leaf $file
    $text = Get-Content -LiteralPath $file -Raw -Encoding UTF8
    $summary = Get-FileSummary $name $layer $text
    $noteRel = $NoteByPath[$key]
    $replacementCount = [regex]::Matches($text, [string][char]0xFFFD).Count
    $encodingWarning = if ($replacementCount -gt 0) {
        "> [!warning] 上游源文件已有损坏字符`n> 原始文件本身含 $replacementCount 个Unicode替换字符（U+FFFD），主要影响供应商自动生成注释。知识库按原样保留，不能把它误认为生成器新增的乱码；寄存器含义应回查ET6039寄存器手册。"
    } else { '' }

    $macros = @([regex]::Matches($text, '(?m)^\s*#\s*define\s+([A-Za-z_]\w*)') | ForEach-Object {$_.Groups[1].Value} | Select-Object -Unique)
    $types = @([regex]::Matches($text, '(?m)}\s*([A-Za-z_]\w*(?:_t)?)\s*;') | ForEach-Object {$_.Groups[1].Value} | Select-Object -Unique)
    $defs = @($Definitions[$key])

    $depLines = if ($DirectDeps[$key].Count) { @($DirectDeps[$key] | ForEach-Object { '- ' + (Get-WikiLink $_ $NoteByPath) }) -join "`n" } else { '- 无工程内直接包含依赖，或只依赖编译器内建环境。' }
    $rev = if ($ReverseDeps.ContainsKey($key)) { @($ReverseDeps[$key] | Sort-Object -Unique | ForEach-Object { '- ' + (Get-WikiLink $_ $NoteByPath) }) -join "`n" } else { '- 未发现其他闭包文件直接 `#include` 本文件；它可能由构建系统直接编译或通过链接使用。' }
    $functionRows = if ($defs.Count) {
        @($defs | ForEach-Object { '| `' + $_ + '()` | ' + (Get-FunctionMeaning $_) + ' |' }) -join "`n"
    } else { '| — | 本文件主要提供宏、类型、全局对象或汇编入口，没有由简化解析器识别到普通C函数定义。 |' }
    $macroText = if ($macros.Count) { ($macros | Select-Object -First 80 | ForEach-Object {'`' + $_ + '`'} ) -join '、' } else {'未发现普通 `#define` 宏。'}
    $typeText = if ($types.Count) { ($types | Select-Object -First 60 | ForEach-Object {'`' + $_ + '`'} ) -join '、' } else {'未发现具名 `typedef struct/enum` 结尾；类型也可能定义在依赖头文件中。'}

    $called = New-Object System.Collections.Generic.List[string]
    foreach ($fn in $AllFunctionOwners.Keys) {
        if ($text -match ('\b' + [regex]::Escape($fn) + '\s*\(') -and $fn -notin $defs) { $called.Add($fn) }
    }
    $callLines = if ($called.Count) {
        @($called | Sort-Object | Select-Object -First 100 | ForEach-Object {
            $owners = @($AllFunctionOwners[$_] | ForEach-Object { Get-WikiLink $_ $NoteByPath }) -join ' / '
            '- `' + $_ + '()` → ' + $owners
        }) -join "`n"
    } else { '- 没有识别到对闭包内具名函数的直接调用，或调用通过函数指针/宏完成。' }

    $sourceFence = if ($name.EndsWith('.s')) {'asm'} elseif ($name.EndsWith('.h') -or $name.EndsWith('.c')) {'c'} else {'text'}
    $sourcePathEscaped = $file -replace '\\','\\'
    $content = @"
---
title: "$name 逐文件解读"
aliases:
  - "$name"
tags:
  - 电驱/PMSM
  - 代码解读/$layer
source_path: "$sourcePathEscaped"
generated_from_build_dependency: true
---

# $name 逐文件解读

> [!summary] 一句话定位
> $summary

$encodingWarning

## 1. 初学者先知道什么

- **所属层级**：$layer
- **工程内路径**：$relative
- **是否真实参与固件**：是。本页文件来自Keil最近一次构建生成的 `.d` 依赖闭包，而不是仅凭文件夹名称猜测。
- **推荐阅读方式**：先看本页“职责、依赖、函数”，再沿双向链接阅读调用者与被调用模块，最后对照文末完整源码。

## 2. 文件职责与控制链位置

$summary

它在整个系统中的意义不能只由文件名判断：MCSDK大量使用“句柄结构体 + 函数指针 + 周期任务”，所以还要结合 [[01-系统总览与执行链]]、[[03-中断与多时间尺度调度]] 和下方反向依赖确认真实执行时机。

## 3. 直接包含的工程依赖

$depLines

## 4. 哪些文件依赖它

$rev

## 5. 主要类型与宏

### 类型

$typeText

### 宏

$macroText

> [!warning] 宏不等于物理量
> 电机控制宏常带定点缩放、单位换算或时钟假设。修改前必须同时确认定义处、使用处、执行频率和硬件量程，参见 [[09-参数体系、单位与定点缩放]]。

## 6. 本文件定义的函数

| 函数 | 初学者解读 |
|---|---|
$functionRows

## 7. 它调用的工程函数

$callLines

> [!note] 调用图的边界
> 本节由源码静态扫描生成，能帮助导航，但宏展开、条件编译、同名函数和函数指针可能造成漏报或多报。实时控制的最终依据仍是Keil配置、链接结果和运行时测量。

## 8. 与硬件的关系

$(Get-HardwareRelations $text)

硬件链路总表见 [[10-ET6039控制板与PD6010B硬件链路]]，引脚和采样参数修改入口见 [[11-硬件参数到代码宏的追踪表]]。

## 9. 阅读和修改时的检查问题

1. 这个函数是在复位初始化、PWM高频中断、速度环中频任务，还是前台循环中执行？
2. 输入输出使用的是安培/伏特/rpm，还是MCSDK定点内部单位？
3. 该值是否跨中断共享，是否需要 `volatile`、临界区或无锁一致性？
4. 它最终是否会改变PWM比较值、驱动使能、ADC通道或故障状态？
5. 修改后应观察什么证据：六路PWM、相电流零偏、母线电压、Iq/Id、STO可靠性还是状态机？

## 10. 完整源码快照

> [!info] 快照说明
> 以下内容在生成知识库时从 $relative 读取，用于离线学习。实际修改仍应在原工程源码中进行，然后重新生成笔记以避免笔记与固件不一致。

```$sourceFence
$text
```

## 11. 关联专题

- [[00-首页与使用说明]]
- [[01-系统总览与执行链]]
- [[04-FOC数学与控制原理]]
- [[05-SVPWM与三相逆变桥]]
- [[06-三电阻双ADC电流采样]]
- [[07-STO-PLL无感观测器]]
- [[08-无感启动、Rev-Up与闭环切换]]
- [[12-保护、故障与安全边界]]
"@
    Write-Utf8File (Join-Path $VaultRoot $noteRel) $content
    $indexRows.Add('| ' + (Get-WikiLink $file $NoteByPath) + ' | ' + $layer + ' | `' + $relative + '` | ' + $summary + ' |')
}

$countsText = @($layerCounts.GetEnumerator() | Sort-Object Name | ForEach-Object { '- **' + $_.Name + '**：' + $_.Value + ' 个' }) -join "`n"
$index = @"
# 实际构建代码总索引

> [!important] 覆盖范围
> 本索引来自 `Keil/Objects/*.d` 的真实编译依赖闭包，共 **$($files.Count)** 个源文件/头文件；它比“只看Keil分组中的39个C文件”更完整，也避免把整个未使用MCSDK误当成当前固件。

$countsText

| 文件笔记 | 层级 | 原始相对路径 | 作用 |
|---|---|---|---|
$($indexRows -join "`n")

## 如何使用本索引

1. 初学者先按 [[02-初学者学习路线]] 阅读专题，不建议从表格第一行顺序啃到最后一行。
2. 遇到具体函数时，通过Obsidian搜索函数名；逐文件页包含完整源码、定义函数和调用关系。
3. 修改参数前先查 [[11-硬件参数到代码宏的追踪表]] 和 [[09-参数体系、单位与定点缩放]]。
4. 每次源码变动后重新运行生成脚本，本页和逐文件源码快照才会同步更新。
"@
Write-Utf8File (Join-Path $VaultRoot '04-逐文件解读\00-实际构建代码总索引.md') $index

# 生成简化的文件依赖Mermaid图；只显示控制核心，避免142节点使Obsidian无法阅读。
$coreNames = @('main.c','motorcontrol.c','mc_tasks.c','mc_math.c','pid_regulator.c','speed_torq_ctrl.c','revup_ctrl.c','virtual_speed_sensor.c','sto_pll_speed_pos_fdbk.c','pwm_curr_fdbk.c','r3_2_f4xx_pwm_curr_fdbk.c','init_config.c','mc_it.c','state_machine.c')
$core = @($files | Where-Object {(Split-Path -Leaf $_) -in $coreNames})
$nodeIds=@{}; $n=0
foreach($f in $core){$nodeIds[$f.ToLowerInvariant()]='N'+$n; $n++}
$edges=New-Object System.Collections.Generic.List[string]
foreach($f in $core){foreach($d in $DirectDeps[$f.ToLowerInvariant()]){if($nodeIds.ContainsKey($d.ToLowerInvariant())){$edges.Add('    '+$nodeIds[$f.ToLowerInvariant()]+' --> '+$nodeIds[$d.ToLowerInvariant()])}}}
$nodes=@($core | ForEach-Object {'    '+$nodeIds[$_.ToLowerInvariant()]+'["'+(Split-Path -Leaf $_)+'"]'}) -join "`n"
$graph=@"
# 核心文件依赖关系图

```mermaid
flowchart TD
$nodes
$($edges -join "`n")
```

> [!note]
> 箭头表示源码直接包含关系，不完全等同于运行时调用方向。运行时顺序请看 [[01-系统总览与执行链]]。
"@
Write-Utf8File (Join-Path $VaultRoot '13-关系图谱\核心文件依赖关系图.md') $graph

# 输出机器可核查清单。
$manifest = @($files | ForEach-Object {
    [pscustomobject]@{ source=$_; layer=(Get-Layer $_); note=$NoteByPath[$_.ToLowerInvariant()]; sha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $_).Hash }
}) | ConvertTo-Json -Depth 4
Write-Utf8File (Join-Path $VaultRoot '_生成信息\代码覆盖清单.json') $manifest

Write-Output "Generated $($files.Count) code notes at $VaultRoot"
