from __future__ import annotations

import re
import sys
from pathlib import Path


SPECIAL = {
    "\\": r"\textbackslash{}",
    "{": r"\{",
    "}": r"\}",
    "$": r"\$",
    "&": r"\&",
    "#": r"\#",
    "_": r"\_",
    "%": r"\%",
    "~": r"\textasciitilde{}",
    "^": r"\textasciicircum{}",
}


def escape_tex(text: str) -> str:
    return "".join(SPECIAL.get(char, char) for char in text)


def inline_tex(text: str) -> str:
    placeholders: list[str] = []

    def keep(value: str) -> str:
        placeholders.append(value)
        return f"@@LATEX{len(placeholders) - 1}@@"

    text = re.sub(
        r"\[([^\]]+)\]\(([^)]+)\)",
        lambda match: keep(
            r"\href{" + escape_tex(match.group(2)) + "}{" + escape_tex(match.group(1)) + "}"
        ),
        text,
    )
    text = re.sub(
        r"`([^`]+)`",
        lambda match: keep(r"\texttt{" + escape_tex(match.group(1)) + "}"),
        text,
    )
    text = escape_tex(text)
    text = re.sub(r"\*\*([^*]+)\*\*", r"\\textbf{\1}", text)
    text = re.sub(r"(?<!\*)\*([^*]+)\*(?!\*)", r"\\emph{\1}", text)
    for index, value in enumerate(placeholders):
        text = text.replace(f"@@LATEX{index}@@", value)
    return text


def table_spec(column_count: int) -> tuple[str, bool, str]:
    landscape = column_count >= 7
    if column_count == 1:
        widths = [0.94]
    elif column_count == 2:
        widths = [0.25, 0.65]
    elif column_count == 3:
        widths = [0.22, 0.32, 0.32]
    elif column_count == 4:
        widths = [0.18, 0.22, 0.22, 0.22]
    elif column_count <= 6:
        first = 0.18
        remaining = 0.65 / (column_count - 1)
        widths = [first] + [remaining] * (column_count - 1)
    else:
        first = 0.13
        remaining = 0.68 / (column_count - 1)
        widths = [first] + [remaining] * (column_count - 1)
    spec = " ".join(
        rf">{{\RaggedRight\arraybackslash}}p{{{width:.3f}\linewidth}}" for width in widths
    )
    size = r"\scriptsize" if column_count >= 7 else (r"\footnotesize" if column_count >= 5 else r"\small")
    return spec, landscape, size


def split_table_row(line: str) -> list[str]:
    stripped = line.strip()
    if stripped.startswith("|"):
        stripped = stripped[1:]
    if stripped.endswith("|"):
        stripped = stripped[:-1]
    return [cell.strip() for cell in stripped.split("|")]


def is_separator_row(cells: list[str]) -> bool:
    return bool(cells) and all(re.fullmatch(r":?-{3,}:?", cell.replace(" ", "")) for cell in cells)


def render_table(lines: list[str], start: int) -> tuple[list[str], int]:
    rows: list[list[str]] = []
    index = start
    while index < len(lines) and lines[index].lstrip().startswith("|"):
        rows.append(split_table_row(lines[index]))
        index += 1
    if len(rows) < 2 or not is_separator_row(rows[1]):
        return [inline_tex(lines[start]) + r"\par"], start + 1

    header = rows[0]
    body = rows[2:]
    column_count = len(header)
    spec, landscape, size = table_spec(column_count)
    rendered: list[str] = []
    if landscape:
        rendered.append(r"\begin{landscape}")
    rendered.extend(
        [
            r"\begin{center}",
            size,
            r"\setlength{\tabcolsep}{3.2pt}",
            r"\renewcommand{\arraystretch}{1.28}",
            rf"\begin{{longtable}}{{{spec}}}",
            r"\toprule",
            r"\rowcolor{ReportNavy}",
            " & ".join(r"\color{white}\bfseries " + inline_tex(cell) for cell in header) + r" \\",
            r"\midrule",
            r"\endfirsthead",
            r"\toprule",
            r"\rowcolor{ReportNavy}",
            " & ".join(r"\color{white}\bfseries " + inline_tex(cell) for cell in header) + r" \\",
            r"\midrule",
            r"\endhead",
            r"\midrule",
            rf"\multicolumn{{{column_count}}}{{r}}{{\color{{ReportMuted}}\footnotesize 次ページへ続く}} \\",
            r"\endfoot",
            r"\bottomrule",
            r"\endlastfoot",
        ]
    )
    for row_index, row in enumerate(body):
        row = (row + [""] * column_count)[:column_count]
        if row_index % 2:
            rendered.append(r"\rowcolor{ReportLight}")
        rendered.append(" & ".join(inline_tex(cell) for cell in row) + r" \\")
    rendered.extend([r"\end{longtable}", r"\end{center}"])
    if landscape:
        rendered.append(r"\end{landscape}")
    return rendered, index


def preamble() -> str:
    return r"""\documentclass[10pt,a4paper,openany]{ltjsbook}
\usepackage[margin=21mm,headheight=24pt,headsep=8mm,footskip=12mm]{geometry}
\usepackage{luatexja-fontspec}
\setmainjfont[
  Path=C:/texlive/2025/texmf-dist/fonts/opentype/public/haranoaji/,
  BoldFont=HaranoAjiMincho-Bold.otf
]{HaranoAjiMincho-Regular.otf}
\setsansjfont[
  Path=C:/texlive/2025/texmf-dist/fonts/opentype/public/haranoaji/,
  BoldFont=HaranoAjiGothic-Bold.otf
]{HaranoAjiGothic-Regular.otf}
\setmonojfont[
  Path=C:/texlive/2025/texmf-dist/fonts/opentype/public/haranoaji/,
  BoldFont=HaranoAjiGothic-Bold.otf
]{HaranoAjiGothic-Regular.otf}
\setmainfont[
  Path=C:/texlive/2025/texmf-dist/fonts/opentype/public/lm/,
  BoldFont=lmroman10-bold.otf,
  ItalicFont=lmroman10-italic.otf
]{lmroman10-regular.otf}
\setsansfont[
  Path=C:/texlive/2025/texmf-dist/fonts/opentype/public/lm/,
  BoldFont=lmsans10-bold.otf,
  ItalicFont=lmsans10-oblique.otf
]{lmsans10-regular.otf}
\setmonofont[
  Path=C:/texlive/2025/texmf-dist/fonts/opentype/public/lm/,
  BoldFont=lmmono10-regular.otf,
  ItalicFont=lmmono10-regular.otf,
  BoldItalicFont=lmmono10-regular.otf
]{lmmono10-regular.otf}
\usepackage[table,dvipsnames]{xcolor}
\definecolor{ReportNavy}{HTML}{17324D}
\definecolor{ReportTeal}{HTML}{008C95}
\definecolor{ReportAmber}{HTML}{E8913A}
\definecolor{ReportInk}{HTML}{263747}
\definecolor{ReportMuted}{HTML}{66788A}
\definecolor{ReportLight}{HTML}{EDF4F6}
\definecolor{ReportPale}{HTML}{F7FAFB}
\definecolor{CodeBg}{HTML}{17242F}
\definecolor{CodeText}{HTML}{E7EEF3}
\usepackage{graphicx}
\usepackage{tikz}
\usetikzlibrary{calc,positioning}
\usepackage{eso-pic}
\usepackage{booktabs}
\usepackage{array}
\usepackage{longtable}
\usepackage{pdflscape}
\usepackage{ragged2e}
\usepackage{listings}
\usepackage{fancyhdr}
\usepackage{hyperref}
\usepackage{setspace}
\usepackage{etoolbox}
\hypersetup{
  colorlinks=true,
  linkcolor=ReportTeal,
  urlcolor=ReportTeal,
  citecolor=ReportTeal,
  pdftitle={CoDrone EDU 制御・実験・Git変更 完全詳細報告書},
  pdfauthor={PythonProject}
}
\urlstyle{same}
\setlength{\parindent}{0pt}
\setlength{\parskip}{0.62em}
\setstretch{1.10}
\setcounter{secnumdepth}{3}
\setcounter{tocdepth}{2}
\renewcommand{\labelitemi}{\color{ReportTeal}\small\textbullet}
\renewcommand{\labelitemii}{\color{ReportAmber}\scriptsize\textbullet}

\lstset{
  basicstyle=\ttfamily\footnotesize\color{CodeText},
  backgroundcolor=\color{CodeBg},
  frame=single,
  rulecolor=\color{ReportTeal},
  framesep=7pt,
  xleftmargin=1.2em,
  xrightmargin=0.4em,
  breaklines=true,
  breakatwhitespace=false,
  showstringspaces=false,
  columns=fullflexible,
  keepspaces=true,
  aboveskip=0.8em,
  belowskip=0.8em
}

\pagestyle{fancy}
\fancyhf{}
\fancyhead[L]{\sffamily\footnotesize\color{ReportTeal}\bfseries CODRONE EDU CONTROL RESEARCH}
\fancyhead[R]{\sffamily\footnotesize\color{ReportMuted}\nouppercase{\leftmark}}
\fancyfoot[L]{\sffamily\scriptsize\color{ReportMuted}PythonProject / Technical Evidence Report}
\fancyfoot[R]{\sffamily\small\color{ReportNavy}\bfseries\thepage}
\renewcommand{\headrulewidth}{0.6pt}
\renewcommand{\headrule}{\hbox to\headwidth{\color{ReportTeal}\leaders\hrule height \headrulewidth\hfill}}
\renewcommand{\footrulewidth}{0pt}

\makeatletter
\renewcommand{\@makechapterhead}[1]{%
  \vspace*{-15pt}%
  \begin{tikzpicture}[remember picture,overlay]
    \fill[ReportLight] (0,0.35) rectangle (\textwidth,-1.45);
    \fill[ReportTeal] (0,0.35) rectangle (0.18,-1.45);
  \end{tikzpicture}%
  \vspace*{7pt}%
  {\hspace*{0.28cm}\sffamily\color{ReportTeal}\small\bfseries CHAPTER \thechapter\par}%
  \vspace{5pt}%
  {\sffamily\color{ReportNavy}\huge\bfseries #1\par}%
  \vspace{24pt}%
}
\renewcommand{\@makeschapterhead}[1]{%
  \vspace*{-15pt}%
  \begin{tikzpicture}[remember picture,overlay]
    \fill[ReportLight] (0,0.35) rectangle (\textwidth,-1.45);
    \fill[ReportAmber] (0,0.35) rectangle (0.18,-1.45);
  \end{tikzpicture}%
  \vspace*{7pt}%
  {\hspace*{0.28cm}\sffamily\color{ReportAmber}\small\bfseries EXECUTIVE SECTION\par}%
  \vspace{5pt}%
  {\sffamily\color{ReportNavy}\huge\bfseries #1\par}%
  \vspace{24pt}%
}
\renewcommand\section{\@startsection{section}{1}{0pt}%
  {2.2ex plus .5ex minus .2ex}{1.0ex}%
  {\sffamily\Large\bfseries\color{ReportNavy}}}
\renewcommand\subsection{\@startsection{subsection}{2}{0pt}%
  {1.8ex plus .4ex minus .2ex}{0.7ex}%
  {\sffamily\large\bfseries\color{ReportTeal}}}
\renewcommand\subsubsection{\@startsection{subsubsection}{3}{0pt}%
  {1.4ex plus .3ex minus .2ex}{0.5ex}%
  {\sffamily\normalsize\bfseries\color{ReportInk}}}
\makeatother

\newcommand{\reportlabel}[1]{%
  \par\vspace{0.55em}%
  \noindent\colorbox{ReportTeal}{%
    \parbox{\dimexpr\linewidth-2\fboxsep\relax}{%
      \sffamily\bfseries\color{white}#1%
    }%
  }\par\vspace{0.20em}%
}
\newcommand{\reportquote}[1]{%
  \par\noindent
  \begin{tikzpicture}
    \node[anchor=north west,inner sep=10pt,outer sep=0pt,
      fill=ReportLight,text width=\dimexpr\linewidth-24pt\relax,
      draw=ReportTeal,line width=0.8pt] {\color{ReportInk}\itshape #1};
  \end{tikzpicture}\par
}

\begin{document}
\frontmatter
\begin{titlepage}
\begin{tikzpicture}[remember picture,overlay]
  \fill[ReportNavy] (current page.south west) rectangle (current page.north east);
  \fill[ReportTeal] ($(current page.north west)+(0,-2.0)$)
    -- ($(current page.north east)+(0,-0.7)$)
    -- (current page.north east)
    -- (current page.north west) -- cycle;
  \fill[ReportAmber] ($(current page.south west)+(0,1.65)$)
    rectangle ($(current page.south east)+(0,1.78)$);
  \draw[ReportTeal!55,line width=1.1pt]
    ($(current page.center)+(-7.0,-4.1)$) circle (5.3);
  \draw[ReportTeal!30,line width=0.7pt]
    ($(current page.center)+(-7.0,-4.1)$) circle (4.5);
\end{tikzpicture}
\vspace*{3.4cm}
{\sffamily\color{ReportTeal!30}\bfseries\large TECHNICAL EVIDENCE REPORT\par}
\vspace{0.7cm}
{\sffamily\color{white}\bfseries\fontsize{28}{34}\selectfont
CoDrone EDU\\[0.25em]
制御・実験・Git変更\\[0.25em]
完全詳細報告書\par}
\vspace{1.0cm}
{\color{ReportTeal!55}\rule{0.70\textwidth}{1.2pt}\par}
\vspace{0.8cm}
{\sffamily\color{white!82}\large
初期API実験から多軸ハイブリッド制御、\\
センサ限界、代替測位、全ログ・全テストまで\par}
\vfill
\begin{minipage}{0.72\textwidth}
\sffamily\color{white!75}
\begin{tabular}{@{}ll@{}}
\color{ReportAmber}\bfseries 対象期間 & 2026-05-19 -- 2026-07-21\\[0.35em]
\color{ReportAmber}\bfseries 作成日 & 2026-07-27\\[0.35em]
\color{ReportAmber}\bfseries Repository & PythonProject\\[0.35em]
\color{ReportAmber}\bfseries HEAD & 011b1d17963a70d6f8baa49c19f6f1f870829aa6
\end{tabular}
\end{minipage}
\vspace{1.3cm}
\end{titlepage}

\cleardoublepage
\thispagestyle{empty}
\vspace*{2cm}
{\sffamily\color{ReportNavy}\Huge\bfseries Report Scope\par}
\vspace{0.5cm}
{\color{ReportTeal}\rule{\textwidth}{1.0pt}\par}
\vspace{0.8cm}
\begin{center}
\renewcommand{\arraystretch}{1.6}
\begin{tabular}{>{\sffamily\bfseries\color{ReportTeal}}l >{\RaggedRight\arraybackslash}p{0.60\textwidth}}
Git履歴 & 全25コミット\\
PID飛行ログ & 全42件\\
加速度・inertialログ & 全18件\\
球風ログ & 全7件\\
Dead reckoningログ & 全3件\\
自動テスト & 全35件成功\\
構文検証 & 全Pythonファイル成功
\end{tabular}
\end{center}
\vfill
{\small\color{ReportMuted}
本書は、確認事実、強い推定、思想上の解釈を分離し、成功例だけでなく、
runaway、timeout、欠損ログ、撤回された仮説も同じ重要度で記録する。}

\cleardoublepage
\tableofcontents
\mainmatter
\color{ReportInk}
"""


def ending() -> str:
    return r"""
\backmatter
\cleardoublepage
\thispagestyle{empty}
\begin{tikzpicture}[remember picture,overlay]
  \fill[ReportNavy] (current page.south west) rectangle (current page.north east);
  \fill[ReportTeal] (current page.south west) rectangle ($(current page.south east)+(0,0.18)$);
\end{tikzpicture}
\vspace*{0.31\textheight}
\begin{center}
{\sffamily\color{white}\fontsize{25}{31}\selectfont\bfseries
Evidence before assumption.\\
Safety before speed.\\
Observability before control.\par}
\vspace{1.2cm}
{\sffamily\color{ReportTeal!35}\large
CoDrone EDU Control Research / PythonProject\par}
\end{center}
\end{document}
"""


def convert(markdown: str) -> str:
    lines = markdown.splitlines()
    output: list[str] = [preamble()]
    index = 0
    in_code = False
    list_stack: list[tuple[int, str]] = []
    paragraph: list[str] = []

    def flush_paragraph() -> None:
        nonlocal paragraph
        if paragraph:
            output.append(inline_tex(" ".join(part.strip() for part in paragraph)) + r"\par")
            paragraph = []

    def close_lists(target_indent: int = -1) -> None:
        while list_stack and list_stack[-1][0] >= target_indent:
            _, environment = list_stack.pop()
            output.append(rf"\end{{{environment}}}")

    while index < len(lines):
        line = lines[index]
        stripped = line.strip()

        if in_code:
            if stripped.startswith("```"):
                output.append(r"\end{lstlisting}")
                in_code = False
            else:
                output.append(line)
            index += 1
            continue

        if stripped.startswith("```"):
            flush_paragraph()
            close_lists()
            language = stripped[3:].strip()
            option = f"[language={language}]" if language in {"Python", "python"} else ""
            output.append(rf"\begin{{lstlisting}}{option}")
            in_code = True
            index += 1
            continue

        if line.lstrip().startswith("|") and index + 1 < len(lines):
            flush_paragraph()
            close_lists()
            rendered, next_index = render_table(lines, index)
            output.extend(rendered)
            index = next_index
            continue

        heading = re.match(r"^(#{1,6})\s+(.+)$", stripped)
        if heading:
            flush_paragraph()
            close_lists()
            level = len(heading.group(1))
            title = inline_tex(heading.group(2))
            if level == 1:
                # The document title is rendered by the designed title page.
                pass
            elif level == 2:
                if heading.group(2) == "最初に結論":
                    output.append(rf"\chapter*{{{title}}}")
                    output.append(rf"\addcontentsline{{toc}}{{chapter}}{{{title}}}")
                    output.append(r"\markboth{最初に結論}{最初に結論}")
                else:
                    output.append(rf"\chapter{{{title}}}")
            elif level == 3:
                output.append(rf"\section{{{title}}}")
            elif level == 4:
                output.append(rf"\subsection{{{title}}}")
            else:
                output.append(rf"\subsubsection*{{{title}}}")
            index += 1
            continue

        if re.fullmatch(r"-{3,}", stripped):
            flush_paragraph()
            close_lists()
            if len(output) == 1:
                index += 1
                continue
            output.append(r"\vspace{0.3em}{\noindent\color{ReportLight}\rule{\linewidth}{0.8pt}}\vspace{0.3em}")
            index += 1
            continue

        standalone_label = re.fullmatch(r"\*\*([^*]+)\*\*", stripped)
        if standalone_label:
            flush_paragraph()
            close_lists()
            output.append(r"\reportlabel{" + inline_tex(standalone_label.group(1)) + "}")
            index += 1
            continue

        if stripped.startswith(">"):
            flush_paragraph()
            close_lists()
            quote_lines: list[str] = []
            while index < len(lines) and lines[index].strip().startswith(">"):
                quote_lines.append(lines[index].strip()[1:].strip())
                index += 1
            output.append(r"\reportquote{" + inline_tex(" ".join(quote_lines)) + "}")
            continue

        list_match = re.match(r"^(\s*)([-*]|\d+\.)\s+(.+)$", line)
        if list_match:
            flush_paragraph()
            indent = len(list_match.group(1).expandtabs(2))
            environment = "enumerate" if list_match.group(2).endswith(".") and list_match.group(2)[0].isdigit() else "itemize"
            while list_stack and indent < list_stack[-1][0]:
                _, closing = list_stack.pop()
                output.append(rf"\end{{{closing}}}")
            if not list_stack or indent > list_stack[-1][0] or environment != list_stack[-1][1]:
                if list_stack and indent == list_stack[-1][0] and environment != list_stack[-1][1]:
                    _, closing = list_stack.pop()
                    output.append(rf"\end{{{closing}}}")
                output.append(rf"\begin{{{environment}}}")
                list_stack.append((indent, environment))
            output.append(r"\item " + inline_tex(list_match.group(3)))
            index += 1
            continue

        if not stripped:
            flush_paragraph()
            close_lists()
            index += 1
            continue

        # Skip source-title metadata already represented on the designed title page.
        if index < 9 and (
            stripped.startswith("作成日:")
            or stripped.startswith("対象期間:")
            or stripped.startswith("対象リポジトリ:")
            or stripped.startswith("対象ブランチ:")
            or stripped.startswith("作成時HEAD:")
        ):
            index += 1
            continue

        paragraph.append(line)
        index += 1

    flush_paragraph()
    close_lists()
    if in_code:
        output.append(r"\end{lstlisting}")
    output.append(ending())
    return "\n".join(output)


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: markdown_to_latex_report.py INPUT.md OUTPUT.tex", file=sys.stderr)
        return 2
    source = Path(sys.argv[1])
    target = Path(sys.argv[2])
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(convert(source.read_text(encoding="utf-8")), encoding="utf-8", newline="\n")
    print(f"wrote {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
