<h1 align="center">LaTex Learning</h1>

## 参考书籍：LATEX NOTES 雷太赫排版系统简介（第二版） (包太雷)

# 基础配置

文档类声明**P19**：options能确定“正文字号、纸张尺寸、标题后另起新页、栏数、横向打印、章节起始页（奇数、偶数、任意页）、草稿模式等”

```
\documentclass[options]{class}
```

<img title="" src="https://images2018.cnblogs.com/blog/137119/201803/137119-20180316110051760-1695326882.png" alt="Word字体大小对照换算表(字号、磅、英寸、像素)_word 字号转成pt-CSDN博客" width="388" data-align="center">

要正确显示中文：选择XeLaTeX

```
\usepackage[UTF8]{ctex} % 中文支持宏包
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

标题层次

```
\part{...} %Level -1
\chapter{...} %Level 0
\section{...} %Level 1
\subsection{...} %Level 2
\subsubsection{...} %Level 3
\paragraph{...} %Level 4
\subparagraph{...} %Level 5
```

目录

```
\setcounter{tocdepth}{2} %设 定 目 录 深 度
\tableofcontents %列 出 目 录
```
