% @file 16384_doc.cls
% @brief Implements a generic document class that 16-384 documents can all use
% to present a standard interface.
%
% @author Oscar Bezi (odb@andrew)
% @author Garrison Johnston (johnsg7@rpi.edu)
% @author Puneet Singhal (psinghal@andrew.cmu.edu)
%		Changes: added "\renewcommand{\thepartno}{\arabic{partno}}" to match the style with gradescope
%
% @since 12 February 2016
%===============================================================================

\LoadClass[12pt,addpoints]{exam}

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{16384_doc}[2016/02/12 Provides standard formatting for 16-384 handout documents.]

\RequirePackage[utf8]{inputenc}
\RequirePackage[T1]{fontenc}
\RequirePackage{amsmath,amsfonts,amssymb}
\RequirePackage[left=1in, right=1in, top=1in, bottom=1in]{geometry}
\RequirePackage{graphicx}
\RequirePackage{lastpage}
\RequirePackage{etoolbox}
\RequirePackage{titlesec}
\RequirePackage{indentfirst}
\RequirePackage{enumitem}
\RequirePackage{titlesec}
\RequirePackage{nicefrac}
\RequirePackage{hyperref}

%% Hyperref setup
\usepackage{xcolor}
\usepackage{tcolorbox}

\hypersetup{
    colorlinks,
    linkcolor={red!50!black},
    citecolor={blue!50!black},
    urlcolor={blue!80!black}
}

% UPDATE THIS EACH SEMESTER
\newcommand{\semester}{Fall 2017}
\newcommand{\instructor}{\href{http://choset.com/}{Prof. Howie Choset}}

% change the numbering of questions and subquestions
\renewcommand{\thepartno}{\arabic{partno}}
% Use a sans-serif font.
\renewcommand{\familydefault}{\sfdefault}
% A nice size for making grading sheet checkboxes out of a tabular.  \setlength{\tabcolsep}{20pt}
\renewcommand{\arraystretch}{2}
\setlist[itemize]{noitemsep,nolistsep}

\newcommand{\sectionbreak}{\clearpage}

% The page header.
\lhead{Robot Kinematics and Dynamics}
\rhead{\assignmentname}
\headrule

% Uniform title format
\title{
    \assignmentname
    \ifdef{\writeup}{\\\textbf{Answer Key: Course Staff Only}}{}
}
\author{Robot Kinematics and Dynamics \\ \instructor}
\date{\ifdef{\duedate}{Due: 9:00AM EST, \duedate}{\examdate}}
\date{\ifdef{\duedatetime}{Due: \duedatetime}{}}

% Formatting for the assignment questions.
\bracketedpoints
\qformat{\thequestion) \thequestiontitle\hfill\textit{\thepoints}}

\ifdef{\writeup}{\printanswers}

% Definitions
\newcommand{\define}[2]{
\hangindent=2em
\hangafter=1
\noindent
\textbf{#1:}#2\\
}

% Written section for assignments
\newcommand{\writtenSection}{
\section{Written Questions} 
For the following problems, fully evaluate all answers unless otherwise
specified.

Answers for written questions must be typed.  We recommend \LaTeX, Microsoft
Word, OpenOffice, or similar.  However, diagrams can be hand-drawn and scanned
in.

%To submit, upload a PDF to Canvas\footnotemark.  Late submissions will not be accepted.

Unless otherwise specified, \textbf{all units are in radians, meters, and seconds}, where
appropriate.

%\footnotetext{\texttt{\href{https://canvas.cmu.edu/}{https://canvas.cmu.edu/}}}
}

% Feedback section for assignments
\newcommand{\feedback}[1]{
\section{Feedback} \begin{questions}{\titledquestion{Feedback Form}[5]
We are always looking to improve the class! To that end, we're looking
        for your feedback on the assignments.  When you've completed the
        assignment, please fill out the \href{#1}{feedback form}. 
        %After completion, you will be given a code. Submit this code as the answer to this question. 
}
\end{questions}}



% Checklist
\newlist{checklist}{itemize}{2}
\setlist[checklist]{label=$\square$}

\newenvironment{submissionChecklist}
{
    \newpage
    \section{Submission Checklist}
    \begin{checklist}
}
{
    \end{checklist}
}

\newcommand{\resourceURL}{https://sites.google.com/site/robotkinematicscmu/resources}


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Macros for math shortcuts!

% Inline matrices.
\newcommand{\smcolvec}[1]{\ensuremath{\left[\begin{smallmatrix}#1\end{smallmatrix}\right]}}

% Larger matrices.
\newcommand{\colvec}[1]{\ensuremath{\begin{bmatrix}#1\end{bmatrix}}}

% Radians show up all the time.
\newcommand{\piover}[1]{\ensuremath{\frac{\pi}{#1}}}

% R^2 and such.
\newcommand\RR[1][]{\ifthenelse{\equal{#1}{}}{\ensuremath{\mathbb{R}}}{\ensuremath{\mathbb{R}^{#1}}}
}

% Shorthand for common variables.
\newcommand{\var}[2]{\ensuremath{#1_{#2}}}
\newcommand{\vard}[2]{\ensuremath{\dot{#1}_{#2}}}

\renewcommand\th[1][]{\var{\theta}{#1}}
\newcommand\thd[1][]{\vard{\theta}{#1}}

\renewcommand\l[1][]{\var{l}{#1}}
\newcommand\ld[1][]{\vard{l}{#1}}

\newcommand\x[1][]{\var{x}{#1}}
\newcommand\xd[1][]{\vard{x}{#1}}

\newcommand\y[1][]{\var{y}{#1}}
\newcommand\yd[1][]{\vard{y}{#1}}

\renewcommand\d[1][]{\var{d}{#1}}
\newcommand\dd[1][]{\vard{d}{#1}}

\newcommand\Q{\ensuremath{\mathbf{Q}}}
\newcommand\Qd{\ensuremath{\dot{\mathbf{Q}}}}

\newcommand\W{\ensuremath{\mathbf{W}}}
\newcommand\Wd{\ensuremath{\dot{\mathbf{W}}}}

% Standardize vectors and matrices.
\renewcommand{\vec}[1]{\ensuremath{\mathbf{#1}}}
\newcommand{\mat}[1]{\ensuremath{\mathbf{#1}}}
\renewcommand{\frame}[1]{\ensuremath{\mathbf{\{#1\}}}}

% Shorthand for inverses.
\newcommand{\inv}[1]{\ensuremath{\left(#1\right)^{-1}}}

% Nice transposes.
\newcommand{\trans}[1]{\ensuremath{\left(#1\right)^{\top}}}

% Homogeneous transforms.
\newcommand{\transform}[3]{\mat{#1_{#2}^{#3}}}
\renewcommand{\H}[2]{\transform{H}{#1}{#2}}
\newcommand{\R}[2]{\transform{R}{#1}{#2}}
\renewcommand{\t}[2]{\transform{d}{#1}{#2}}

\newcommand{\p}[1]{\transform{p}{}{#1}}
\renewcommand{\v}[1]{\transform{v}{}{#1}}
\newcommand{\f}[1][]{\ensuremath{f_{#1}}}

% Jacobians
\newcommand{\J}{\mat{J}}
\newcommand{\Jt}{\trans{\J}}

% Partial derivatives.
\newcommand{\partialfrac}[2]{\ensuremath{\frac{\partial #1}{\partial #2}}}

%Vector Norms
\newcommand{\norm}[1]{\left\lVert#1\right\rVert}

% Rank
\DeclareMathOperator{\rank}{rank}
