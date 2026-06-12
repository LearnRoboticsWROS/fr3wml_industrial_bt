#!/usr/bin/env python3
"""
Render STAGE2_GUIDE.md to STAGE2_GUIDE.pdf with reportlab.

Supports the subset of Markdown actually used in the guide:
  * # / ## / ### / #### headings
  * paragraphs with **bold**, `code`, and `[text](link)` inline
  * fenced ```code blocks```
  * bullet lists (`- ` or `* `)
  * GFM tables (| a | b |)
  * blockquotes (> …)
  * horizontal rules (---)
  * inline HTML-escaping

No external deps beyond reportlab, which is already available in the env.
"""
from __future__ import annotations

import html
import re
import sys
from pathlib import Path
from typing import List

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import cm
from reportlab.platypus import (
    HRFlowable,
    PageBreak,
    Paragraph,
    Preformatted,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


# ───────────────────────── styles ─────────────────────────
styles = getSampleStyleSheet()

body = ParagraphStyle(
    "Body", parent=styles["BodyText"],
    fontName="Helvetica", fontSize=10, leading=14,
    spaceAfter=6, alignment=TA_LEFT,
)
h1 = ParagraphStyle("H1", parent=body, fontName="Helvetica-Bold",
                    fontSize=20, leading=24, spaceBefore=12, spaceAfter=10,
                    textColor=colors.HexColor("#0b2447"))
h2 = ParagraphStyle("H2", parent=body, fontName="Helvetica-Bold",
                    fontSize=15, leading=20, spaceBefore=14, spaceAfter=8,
                    textColor=colors.HexColor("#19376d"))
h3 = ParagraphStyle("H3", parent=body, fontName="Helvetica-Bold",
                    fontSize=12, leading=16, spaceBefore=10, spaceAfter=6,
                    textColor=colors.HexColor("#576cbc"))
h4 = ParagraphStyle("H4", parent=body, fontName="Helvetica-Bold",
                    fontSize=11, leading=14, spaceBefore=8, spaceAfter=4)
bullet = ParagraphStyle("Bul", parent=body, leftIndent=14,
                        bulletIndent=4, spaceAfter=2)
blockquote = ParagraphStyle("BQ", parent=body, leftIndent=16, rightIndent=16,
                            fontName="Helvetica-Oblique",
                            textColor=colors.HexColor("#444444"),
                            borderColor=colors.HexColor("#cccccc"),
                            borderPadding=6, borderWidth=0)
code_style = ParagraphStyle("Code", parent=body,
                            fontName="Courier", fontSize=8.5, leading=11,
                            textColor=colors.HexColor("#111111"),
                            backColor=colors.HexColor("#f3f3f3"),
                            borderPadding=6,
                            leftIndent=4, rightIndent=4, spaceAfter=6)


# ───────────────────── inline formatting ─────────────────────
# Order matters: code spans first so their contents aren't mangled by bold.
CODE_SPAN  = re.compile(r"`([^`]+)`")
BOLD_SPAN  = re.compile(r"\*\*(.+?)\*\*")
LINK_SPAN  = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")


def inline(text: str) -> str:
    """Apply inline markdown → reportlab <font>/<b>/<link> markup, HTML-escaping the rest."""
    tokens: List[str] = []

    def emit(s: str) -> None:
        tokens.append(s)

    # Walk with regex-based tokenisation, replacing spans with placeholders.
    placeholders: List[str] = []

    def stash(rendered: str) -> str:
        placeholders.append(rendered)
        return f"\x00{len(placeholders) - 1}\x00"

    text = CODE_SPAN.sub(
        lambda m: stash(
            f'<font face="Courier" size="9" color="#b91c1c">'
            f'{html.escape(m.group(1))}</font>'),
        text,
    )
    text = LINK_SPAN.sub(
        lambda m: stash(
            f'<link href="{html.escape(m.group(2), quote=True)}" '
            f'color="#1d4ed8"><u>{html.escape(m.group(1))}</u></link>'),
        text,
    )
    text = BOLD_SPAN.sub(
        lambda m: stash(f"<b>{html.escape(m.group(1))}</b>"),
        text,
    )

    text = html.escape(text, quote=False)

    # Re-inject placeholders.
    def unplace(m: re.Match) -> str:
        return placeholders[int(m.group(1))]

    text = re.sub(r"\x00(\d+)\x00", unplace, text)
    return text


# ───────────────────── markdown parser (line-based) ─────────────────────
def parse(md: str):
    lines = md.splitlines()
    story = []
    i = 0
    n = len(lines)

    def flush_paragraph(buf: List[str]) -> None:
        if not buf:
            return
        raw = " ".join(x.strip() for x in buf if x.strip())
        if raw:
            story.append(Paragraph(inline(raw), body))

    para_buf: List[str] = []

    while i < n:
        line = lines[i]
        stripped = line.rstrip()

        # Fenced code block
        if stripped.startswith("```"):
            flush_paragraph(para_buf); para_buf = []
            i += 1
            code_lines: List[str] = []
            while i < n and not lines[i].rstrip().startswith("```"):
                code_lines.append(lines[i])
                i += 1
            i += 1  # consume closing fence
            code_text = "\n".join(code_lines) or " "
            story.append(Preformatted(code_text, code_style))
            continue

        # Headings
        m = re.match(r"^(#{1,4})\s+(.*)$", stripped)
        if m:
            flush_paragraph(para_buf); para_buf = []
            level = len(m.group(1))
            text = m.group(2).strip()
            style = {1: h1, 2: h2, 3: h3, 4: h4}[level]
            story.append(Paragraph(inline(text), style))
            i += 1
            continue

        # Horizontal rule
        if re.match(r"^\s*(-{3,}|\*{3,})\s*$", stripped):
            flush_paragraph(para_buf); para_buf = []
            story.append(HRFlowable(width="100%", thickness=0.6,
                                    color=colors.HexColor("#999999"),
                                    spaceBefore=6, spaceAfter=10))
            i += 1
            continue

        # Tables (header then separator then rows)
        if ("|" in stripped and i + 1 < n
                and re.match(r"^\s*\|?\s*[:\- ]+\|[:\- |]+\s*$", lines[i + 1])):
            flush_paragraph(para_buf); para_buf = []
            header = [c.strip() for c in stripped.strip().strip("|").split("|")]
            i += 2
            rows: List[List[str]] = []
            while i < n and "|" in lines[i] and lines[i].strip():
                cells = [c.strip() for c in lines[i].strip().strip("|").split("|")]
                # Pad or truncate to header width
                if len(cells) < len(header):
                    cells += [""] * (len(header) - len(cells))
                elif len(cells) > len(header):
                    cells = cells[:len(header)]
                rows.append(cells)
                i += 1
            story.append(build_table(header, rows))
            continue

        # Blockquotes
        if stripped.startswith(">"):
            flush_paragraph(para_buf); para_buf = []
            bq_lines: List[str] = []
            while i < n and lines[i].lstrip().startswith(">"):
                bq_lines.append(lines[i].lstrip()[1:].lstrip())
                i += 1
            story.append(Paragraph(inline(" ".join(bq_lines)), blockquote))
            continue

        # Bullet list
        if re.match(r"^\s*[-*]\s+", stripped):
            flush_paragraph(para_buf); para_buf = []
            items: List[str] = []
            while i < n and re.match(r"^\s*[-*]\s+", lines[i]):
                item = re.sub(r"^\s*[-*]\s+", "", lines[i])
                # Handle trailing continuation lines (2+ space indent)
                j = i + 1
                while j < n and lines[j].startswith("  ") and lines[j].strip() \
                        and not re.match(r"^\s*[-*]\s+", lines[j]):
                    item += " " + lines[j].strip()
                    j += 1
                items.append(item)
                i = j
            for it in items:
                story.append(Paragraph("&bull;&nbsp;&nbsp;" + inline(it), bullet))
            continue

        # Blank line → paragraph separator
        if not stripped.strip():
            flush_paragraph(para_buf); para_buf = []
            story.append(Spacer(1, 4))
            i += 1
            continue

        # Otherwise accumulate into paragraph buffer
        para_buf.append(stripped)
        i += 1

    flush_paragraph(para_buf)
    return story


def build_table(header: List[str], rows: List[List[str]]):
    ncols = len(header)
    # Column widths: distribute available width; give last column a bit more.
    usable = 17 * cm
    col_w = [usable / ncols] * ncols

    def cell(text: str) -> Paragraph:
        return Paragraph(inline(text), ParagraphStyle(
            "tcell", parent=body, fontSize=8.5, leading=11, spaceAfter=0))

    def hcell(text: str) -> Paragraph:
        return Paragraph(f"<b>{inline(text)}</b>", ParagraphStyle(
            "thcell", parent=body, fontSize=9, leading=12,
            textColor=colors.white, spaceAfter=0))

    data = [[hcell(h) for h in header]]
    for r in rows:
        data.append([cell(c) for c in r])

    t = Table(data, colWidths=col_w, repeatRows=1)
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#19376d")),
        ("GRID", (0, 0), (-1, -1), 0.3, colors.HexColor("#c4c4c4")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("RIGHTPADDING", (0, 0), (-1, -1), 4),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1),
         [colors.white, colors.HexColor("#f5f7fb")]),
    ]))
    return t


def main(md_path: Path, pdf_path: Path) -> None:
    md = md_path.read_text(encoding="utf-8")
    story = parse(md)

    doc = SimpleDocTemplate(
        str(pdf_path), pagesize=A4,
        leftMargin=2 * cm, rightMargin=2 * cm,
        topMargin=1.8 * cm, bottomMargin=1.8 * cm,
        title="fr3wml_industrial_bt — Stage 2 Guide",
        author="industrial_bt_framework",
    )
    doc.build(story)
    print(f"wrote {pdf_path}")


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    md = here / "STAGE2_GUIDE.md"
    pdf = here / "STAGE2_GUIDE.pdf"
    if len(sys.argv) >= 2:
        md = Path(sys.argv[1])
    if len(sys.argv) >= 3:
        pdf = Path(sys.argv[2])
    main(md, pdf)
