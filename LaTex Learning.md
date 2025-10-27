<h1 align="center">LaTex Learning</h1>

## 参考书籍：LATEX NOTES 雷太赫排版系统简介（第二版） (包太雷)

# 2 入门

文档类声明**P19**：options能确定“正文字号、纸张尺寸、标题后另起新页、栏数、横向打印、章节起始页（奇数、偶数、任意页）、草稿模式等”

```
\documentclass[options]{class}
%class包含article、book、report，用中文文档类对应是ctexart、ctexbook、ctexrep
```

<img title="" src="https://images2018.cnblogs.com/blog/137119/201803/137119-20180316110051760-1695326882.png" alt="Word字体大小对照换算表(字号、磅、英寸、像素)_word 字号转成pt-CSDN博客" width="388" data-align="center">

要正确显示中文：选择XeLaTeX

```
\usepackage[UTF8]{ctex} % 中文支持宏包\newpage  % 换页
```

标题、作者、日期、摘要**P20**

```
\title{LaTeX Notes}%标题
\author{Alpha Huang}%作者
\date{\today}%日期
%上面语句就在导言区
\maketitle%另起标题页（在正文区）
%摘要（book 里没有）
\begin{abstract}
...
\end{abstract}
```

标题层次：article 中没有 chapter，而 report 和 book 则支持所有层次。

```
\part{...} %Level -1
\chapter{...} %Level 0
\section{...} %Level 1
\subsection{...} %Level 2
\subsubsection{...} %Level 3
\paragraph{...} %Level 4
\subparagraph{...} %Level 5
```

```
%标题设置技巧
\part*{总纲}%加“*”会取消自带的编号，只出现“总纲”，但如果要添加进目录就要手动加
```

目录

```
\setcounter{tocdepth}{2} %设 定 目 录 深 度
\tableofcontents %列 出 目 录
%手动添加目录
\addcontentsline{〈文件〉}{〈层级〉}{〈标题文字〉}
```

## 2.3 文字

### 2.3.1 字符输入

有些字符(例如# $ % ^ & _ { } ~等)被用作特殊的控制符，大多前面要加"\"转义；短划线"-",中划线重复两次短划线，长划线（破折号）重复三次。

### 2.3.2 字体样式和大小

衬线字体笔画的边缘部分有些修饰，类似于中文的宋体、仿宋、楷体、魏体等；无衬线字体的笔画则是平滑的，类似于中文的黑体。

字体强调命令：

```
\usepackage{ulem}%导入包
\emph{emphasis}%与周围字体正/斜体相反
\uline{underline}%下划线
\uwave{waveline}%下划波浪线
\sout{strike-out}%删除线
```

字号：

<img title="" src="./images/2025-10-24-10-27-42-image.png" alt="" data-align="center">

### 2.3.3 换行、换页和断字

换行：\\\或\newline；换页：\newpage；断句：\hyphenation{BASIC blar-blar-blar}显式指明断字位置，BASIC 这个词不能断开，blar‐blar‐blar 可以在‐处断开。（这个断句要在导言区设置）

## 2.4 长度

## 2.5 对齐和间距

### 2.5.1段落对齐

```
\begin{flushleft}
居左\\段落
\end{flushleft}

\begin{center}
居中\\段落
\end{center}


\begin{flushright}
居右\\段落
\end{flushright}
```

### 2.5.2 缩进和段落间距

```
%在标准Latex每个章节、节标题之后的第一个段落默认不缩进，解决办法，引入indentfirst
\usepackage{indentfirst}
%控制缩进
\setlength{\parindent}{2em}
%控制段间距
\addtolength{\parskip}{3pt}
%如果要局部设置————使用“{}”进行局部设置
{\setlength{\parindent}{0em}    % 局部：取消缩进
 \setlength{\parskip}{1em}      % 局部：增大段距
 这是局部修改后的第一段。

 这是局部修改后的第二段。}
```

### 2.5.3 行间距

① 原生命令：缺省默认是单倍行距，不仅会改变正文行距，同时也把目录、脚注、图表、标题等的行距给改了。

```
\linespread{1.3} %一 倍 半 行 距
\linespread{1.6} %双 倍 行 距
```

② 加载宏包，可以先全局设置，然后再局部设置（推荐）

```
\usepackage{setspace}
\singlespacing %单 倍 行 距
\onehalfspacing %一 倍 半 行 距
\doublespacing %双 倍 行 距
\setstretch{1.25} %任 意 行 距
```

```
%局部控制行距
\begin{doublespacing}
double\\spacing
\end{doublespacing}
```

## 2.6 特殊段落

### 2.6.1 摘录（如引用诗歌）

LATEX 中有三种摘录环境：quote, quotation, verse。quote 两端都缩进，quotation 在 quote 的基础上增加了首行缩进，verse 比 quote多了第二行起的缩进。

```
\begin{quote}
引文两端\\都缩进。
\end{quote}
```

### 2.6.2 原文打印

就是让文本**不被解释、不被格式化，原样输出**。

### 2.6.3 脚注

重定义命令中可更改“roman”，阿拉伯数字对应**arabic**；小写英文字母对应**alph**；大写英文字母对应**Alph**；小写罗马数字对应**roman**；大写罗马数字对应**Roman**。

```
\renewcommand{\thefootnote}{\roman{footnote}} %设置脚注格式
正文\footnote{脚注}%使用footnote命令
```

### 2.6.4 边注

```
%主要是\marginpar 命令
\marginnote{正常边注}
\reversemarginpar
\marginnote{反向边注}
\normalmarginpar
```

### 2.6.5 注释

%可以用于注释，但是对于大段的注释使用：

```
\begin{comment}
...
\end{comment}
```

## 2.7 列表

### 2.7.1 基本列表

有三种基本列表环境：无序列表、有序列表、描述列表。这些列表可以单独使用，也可以互相嵌套。

<img title="" src="./images/2025-10-24-19-22-19-image.png" alt="" width="451" data-align="center">

## 2.7.2 其他列表

paralist 宏包提供了一系列压缩列表和行间列表环境。

## 2.7.3 定制列表

如要改变无序列表的列表符号和有序列表的编号形式

# 2.8盒子

在 LaTeX 里，“盒子（box）”就是排版的最小单位，所有的文字、图片、表格最终都被打包成盒子再排列组合。

## 2.8.1 初级盒子

最简单的盒子命令是 \mbox（水平盒子，保证在一行内显示内容）和 \fbox。前者把一组对象组合起来，后者在此基础上加了个边框。

## 2.8.2 中级盒子

对齐方式有居中 (缺省) 、居左、居右和分散对齐，分别用 c, l, r,s 来表示。

```
%语法：[宽度][对齐方式]{内容}
\makebox[100pt][c]{仪仗队}
\framebox[100pt][s]{仪仗队}
```

## 2.8.3高级盒子

整个段落可以用 \parbox 命令或 minipage 环境。外部对齐（盒子与盒子）：居顶、居中和居底对齐，分别用 t, c, b 来表示；内部对齐也是。

外部对齐：t——盒子顶部与其他盒子顶部对齐；c——盒子中线与其他盒子中线对齐；b——盒子底部与基线对齐

内部对齐：t——内容贴近盒子上边缘；c——内容居中放置；b——内容贴近盒子下边缘

```
%语法：[外部对齐][高度][内部对齐]{宽度}{内容}
\fbox{%带外框的盒子
\parbox[c][36pt][t]{170pt}{
锦瑟无端五十弦，一弦一柱思华年。庄生晓梦迷蝴蝶，望帝春心托杜鹃。
 }%
}
\hfill%在当前行中插入一个可以自动拉伸（或压缩）的空白，用来推开两侧的内容，使它们尽量靠两边分布
\fbox{%
\begin{minipage}[c][36pt][b]{170pt}
沧海月明珠有泪，蓝田日暖玉生烟。此情可待成追忆，只是当时已惘然。
\end{minipage}%
}
```

## 2.9 交叉引用

核心命令：`\label{标识符}` —— 打标签；`\ref{标识符}` —— 引用编号；`\pageref{标识符}` —— 引用页码。只适用于文档内的图、表、公式

```
%例子介绍
\label{sec:intro}
如图~\ref{fig:setup}~所示
第~\pageref{fig:setup}~页
```

\label会给当前对象（例如章节、图、表、公式）打上一个唯一的“内部标签，会记录该对象的编号和页码。\ref提取编号，\pageref提取页码。

其中“~”：**不换行空格**，常用于让编号和前缀（如“图”“表”“式”）粘在一起，防止被拆行

补充参考文献的引用：

```
\cite{标签}
\begin{thebibliography}{99}%中括号内表示
\bibitem{标签}
\end{thebibliography}
```

其中\cite{标签1，标签2}可以引用多个参考文献；

要实现上标形式和方括号

第一种方法：

```
\usepackage[square,super,comma]{natbib}  % 上标 + 方括号 + 多条合并
```

第二种方法：`iblatex` 宏包 + `biber` 编译

# 3 字体

## 3.1 编码

推荐采用UTF-8格式，现代引擎XeLaTeX / LuaLaTeX，直接

```
\documentclass{ctexart}   % 已自动支持 UTF-8 编码
```

## 3.2 字体格式

### 3.2.1 点阵和矢量字体

 点阵字体是由像素点组成，放大会有锯齿状；轮廓字体又称作矢量字体矢量，缩放平滑，文件稍大。

## 3.2.2 常见字体格式

当前常见的轮廓字体格式有：Type 1, TrueType, OpenType。

### 3.2.3 合纵连横

## 3.3 常见字体

## 3.4 字体的应用

## 3.5 中文解决方案

# 4 数学

**在线公式编辑器：[在线LaTeX公式编辑器-编辑器](https://www.latexlive.com/##)**

为了加载数学功能，要加载amsmath 宏包。

```
\usepackage{amsmath}
```

## 4.1 数学模式

LATEX 的数学模式有两种形式：行间 (inline) 模式和独立 (display) 模式。前者是指在正文中插入数学内容；后者独立排列，可以有或没有编号。

行间公式用 \$...\$，无编号独立公式用\\[...\\]。建议不要用 $$...$$，因为它和 AMS‐LATEX 有冲突。**注意：带编号公式里面直接写入公式内容。**

```
%行间公式
这是一个行内公式：$E = mc^2$，它不会单独占一行。
%这是一个独立公式：
\[
E = mc^
2
\]
%带编号公式
\begin{equation}
E = mc^2
\end{equation}
```

如果公式编号采用**章节+公式序号**类型：可以按照document类型自定义更改section

```
\usepackage{amsmath}
\numberwithin{equation}{section} % 按 section 编号
```

若要改变公式编号the**section**，“-”可以改变连接，\arabic是阿拉伯数字，能换成2.6.3的脚注里的

```
\renewcommand{\theequation}{\thesection-\arabic{equation}}
%\renewcommand是重定义命令
%\theequation用于控制公式编号的显示形式
%\thesection表示当前章节号
\arabic{equation}表示当前公式的计数编号，用阿拉伯数字显示
```

## 4.2 基本元素

### 4.2.1 希腊字母

![](./images/2025-10-26-21-56-53-image.png)

## 4.2.2 上下标和根号

指数或上标用 ^ 表示，下标用 _ 表示，根号用 \sqrt 表示。上下标如果多于一个字母或符号，需要用一对 {} 括起来。

```
\[ x_{ij}^2\quad \sqrt{x}\quad \sqrt[3]{x} \]
```

## 4.2.3 分数

一般分数用 \frac 命令表示；正文中用使用 `\dfrac`可以显得更大；独立公式中希望分数紧凑一点使用 \tfrac。

```
\frac{a}{b}
\dfrac{a}{b}
\tfrac{a}{b}
```

### 4.2.4 运算符

+、-、\*、/、=等可以直接输入，但是一些特殊的

![](./images/2025-10-27-10-04-25-image.png)



### 4.2.5 箭头

![](./images/2025-10-27-10-06-59-image.png)

### 4.2.6 注音和标注

![](./images/2025-10-27-10-07-59-image.png)

![](./images/2025-10-27-10-08-15-image.png)

### 4.2.7 分隔符

各种括号用 ()  []  \{\}  \langle \rangle 等命令表示。因为\\|和|使用随意，引入**amsmath** 宏包用 **\lvert\rvert**和 **\lVert\rVert** 取而代之。可以在上述分隔符前面加 \big \Big \bigg \Bigg 等命令来调整其大小。

```
\usepackage{amsmath}
\[ \Bigg(\bigg(\Big(\big((x)\big)\Big)\bigg)\Bigg)\quad
\Bigg[\bigg[\Big[\big[[x]\big]\Big]\bigg]\Bigg]\quad
\Bigg\{\bigg\{\Big\{\big\{\{x\}\big\}\Big\}\bigg\}\Bigg\}
\]\[
\Bigg\langle\bigg\langle\Big\langle\big\langle\langle x
\rangle\big\rangle\Big\rangle\bigg\rangle\Bigg\rangle\quad
\Bigg\lvert\bigg\lvert\Big\lvert\big\lvert\lvert x
\rvert\big\rvert\Big\rvert\bigg\rvert\Bigg\rvert\quad
\Bigg\lVert\bigg\lVert\Big\lVert\big\lVert\lVert x
\rVert\big\rVert\Big\rVert\bigg\rVert\Bigg\rVert \]
```

<img src="./images/2025-10-27-10-44-16-image.png" title="" alt="" width="436">

### 4.2.8 省略号

```
%独立公式
\[ x_1,x_2,\dots,x_n\quad 1,2,\cdots,n\quad
\vdots\quad \ddots \]
```

![](./images/2025-10-27-10-30-14-image.png)

### 4.2.9 空白间距

![](./images/2025-10-27-10-31-03-image.png)

## 4.3 矩阵

公式编辑器

## 4.4 多行公式

### 4.4.1 长公式

分为无需对齐长公式和对齐长公式

```
%无对齐公式（自带公式编号\begin{equation}）
\begin{multline}
x = a+b+c+{} \\
d+e+f+g
\end{multline}
```

```
%对齐公式（公式编号需要引入\begin{equation}）
\[ \begin{split}
x ={} &a+b+c+{} \\
&d+e+f+g
\end{split} \]
```

![](./images/2025-10-27-10-41-39-image.png)

### 4.4.2 公式组

不需要对齐的公式组可以使用 **gather** 环境（见 例 4.16），需要对齐的公式组用 **align** 环境（见 例 4.17）。自带生成公式编号，如果需要取消在gather等后面加\*

```
%不需对齐公式组
\begin{gather}
a = b+c+d \\
x = y+z
\end{gather}
```

```
%对齐公式组
\begin{align}
a &= b+c+d \\
x &= y+z
\end{align}
```

### 4.4.3 分支公式

分段函数通常用 cases 次环境写成分支公式

```
\[ y=\begin{cases}
-x,\quad x\leq 0 \\
x,\quad x>0
\end{cases} \]
```

![](./images/2025-10-27-11-27-48-image.png)

## 4.5 定理和证明

\newtheorem 命令可以用来定义定理之类的环境，其语法如下。

语法：{环境名}[编号延续]{显示名}[编号层次]

```
%定义环境
\newtheorem{definition}{定义}[section]
\newtheorem{theorem}{定理}[section]
\newtheorem{lemma}[theorem]{引理}
\newtheorem{corollary}[theorem]{推论}
```

然后可以使用环境：

![](./images/2025-10-27-11-30-44-image.png)

## 4.6 数学字体

\mathbb 和 \mathfrak 需要 amsfonts 宏包，\mathscr 需要mathrsfs 宏包。

![](./images/2025-10-27-11-32-03-image.png)
