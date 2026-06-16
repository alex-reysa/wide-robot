#!/usr/bin/env python3
"""Generate print-ready AprilTag marker sheets for the real-camera pilot."""

from __future__ import annotations

import argparse
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from PIL import Image
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4, letter
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


SOURCE_URL = (
    "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/"
    "master/tag36h11/tag36_11_{tag_id:05d}.png"
)


@dataclass(frozen=True)
class MarkerSpec:
    tag_id: int
    role: str
    total_mm: float
    required: bool

    @property
    def filename_stem(self) -> str:
        safe_role = self.role.replace("/", "_")
        return f"tag36_11_{self.tag_id:05d}_{safe_role}_{int(self.total_mm)}mm"


MARKERS = [
    MarkerSpec(0, "table_world_01", 75, True),
    MarkerSpec(1, "table_world_02", 75, True),
    MarkerSpec(4, "tray_front", 50, True),
    MarkerSpec(5, "tray_inside_or_rim", 50, True),
    MarkerSpec(6, "big_cube_top", 50, True),
    MarkerSpec(7, "big_cube_front", 50, True),
    MarkerSpec(2, "small_cube_top", 35, False),
    MarkerSpec(3, "small_cube_front", 35, False),
]


def download_tag_png(tag_id: int, cache_dir: Path) -> Path:
    cache_dir.mkdir(parents=True, exist_ok=True)
    output = cache_dir / f"tag36_11_{tag_id:05d}.png"
    if output.exists():
        return output

    url = SOURCE_URL.format(tag_id=tag_id)
    with urllib.request.urlopen(url, timeout=30) as response:
        output.write_bytes(response.read())
    return output


def load_tag_grid(png_path: Path) -> list[list[bool]]:
    with Image.open(png_path) as image:
        rgba = image.convert("RGBA")
        if rgba.size != (10, 10):
            raise ValueError(f"{png_path} is {rgba.size}, expected 10x10")
        pixels = rgba.load()
        grid: list[list[bool]] = []
        for y in range(10):
            row = []
            for x in range(10):
                r, g, b, a = pixels[x, y]
                row.append(a > 0 and (r + g + b) / 3 < 128)
            grid.append(row)
        return grid


def write_svg(spec: MarkerSpec, grid: list[list[bool]], svg_dir: Path) -> Path:
    svg_dir.mkdir(parents=True, exist_ok=True)
    output = svg_dir / f"{spec.filename_stem}.svg"
    black_rects = []
    for y, row in enumerate(grid):
        for x, is_black in enumerate(row):
            if is_black:
                black_rects.append(f'  <rect x="{x}" y="{y}" width="1" height="1"/>')

    svg = "\n".join(
        [
            '<?xml version="1.0" encoding="UTF-8"?>',
            (
                f'<svg xmlns="http://www.w3.org/2000/svg" '
                f'width="{spec.total_mm}mm" height="{spec.total_mm}mm" '
                f'viewBox="0 0 10 10" shape-rendering="crispEdges">'
            ),
            '  <rect x="0" y="0" width="10" height="10" fill="#fff"/>',
            '  <g fill="#000">',
            *black_rects,
            "  </g>",
            "</svg>",
            "",
        ]
    )
    output.write_text(svg, encoding="utf-8")
    return output


def draw_marker(
    pdf: canvas.Canvas,
    spec: MarkerSpec,
    grid: list[list[bool]],
    x_mm: float,
    y_mm: float,
) -> None:
    size = spec.total_mm * mm
    x = x_mm * mm
    y = y_mm * mm
    cell = size / 10.0

    pdf.setFillColor(colors.white)
    pdf.rect(x, y, size, size, fill=1, stroke=0)

    pdf.setFillColor(colors.black)
    for row_index, row in enumerate(grid):
        for col_index, is_black in enumerate(row):
            if is_black:
                # PDF coordinates grow upward; image rows grow downward.
                cell_x = x + col_index * cell
                cell_y = y + (9 - row_index) * cell
                pdf.rect(cell_x, cell_y, cell, cell, fill=1, stroke=0)

    pdf.setStrokeColor(colors.Color(0.45, 0.45, 0.45))
    pdf.setLineWidth(0.3)
    pdf.setDash(2, 2)
    pdf.rect(x, y, size, size, fill=0, stroke=1)
    pdf.setDash()

    pdf.setFillColor(colors.black)
    pdf.setFont("Helvetica", 6)
    req = "required" if spec.required else "optional"
    label = f"ID {spec.tag_id} - {spec.role} - {int(spec.total_mm)} mm total - {req}"
    pdf.drawString(x, y - 4.0 * mm, label)


def make_pdf(
    output_path: Path,
    page_size: tuple[float, float],
    grids: dict[int, list[list[bool]]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(str(output_path), pagesize=page_size)
    page_width_mm = page_size[0] / mm
    page_height_mm = page_size[1] / mm

    pdf.setTitle("Sony Object Inside Container AprilTags")
    pdf.setAuthor("wide-robot")
    pdf.setFont("Helvetica-Bold", 12)
    pdf.drawString(12 * mm, (page_height_mm - 14) * mm, "Sony object-inside-container AprilTags")
    pdf.setFont("Helvetica", 8)
    pdf.drawString(
        12 * mm,
        (page_height_mm - 20) * mm,
        "Print at 100% / actual size. Do not fit to page. Cut on dashed boxes.",
    )

    rows = [
        [MARKERS[0], MARKERS[1]],
        [MARKERS[2], MARKERS[3], MARKERS[4]],
        [MARKERS[5], MARKERS[6], MARKERS[7]],
    ]
    top = page_height_mm - 34
    y = top
    for row in rows:
        row_width = sum(marker.total_mm for marker in row) + 12 * (len(row) - 1)
        x = max(12, (page_width_mm - row_width) / 2)
        row_height = max(marker.total_mm for marker in row)
        marker_y = y - row_height
        for marker in row:
            draw_marker(pdf, marker, grids[marker.tag_id], x, marker_y)
            x += marker.total_mm + 12
        y = marker_y - 13

    bar_x = 12 * mm
    bar_y = 14 * mm
    pdf.setStrokeColor(colors.black)
    pdf.setLineWidth(1)
    pdf.line(bar_x, bar_y, bar_x + 50 * mm, bar_y)
    pdf.line(bar_x, bar_y - 2 * mm, bar_x, bar_y + 2 * mm)
    pdf.line(bar_x + 50 * mm, bar_y - 2 * mm, bar_x + 50 * mm, bar_y + 2 * mm)
    pdf.setFont("Helvetica", 8)
    pdf.drawString(bar_x, bar_y + 4 * mm, "50 mm scale check")

    pdf.save()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create exact-size AprilTag SVGs and print sheets for the Sony real-camera pilot."
    )
    parser.add_argument(
        "--asset-dir",
        type=Path,
        default=Path("datasets/sony_object_inside_container_v0/calibration/apriltags/tag36h11"),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("output/pdf"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cache_dir = args.asset_dir / "source_png"
    svg_dir = args.asset_dir / "svg"

    grids: dict[int, list[list[bool]]] = {}
    for spec in MARKERS:
        png_path = download_tag_png(spec.tag_id, cache_dir)
        grids[spec.tag_id] = load_tag_grid(png_path)
        write_svg(spec, grids[spec.tag_id], svg_dir)

    make_pdf(
        args.output_dir / "sony_object_inside_container_v0_apriltags_a4.pdf",
        A4,
        grids,
    )
    make_pdf(
        args.output_dir / "sony_object_inside_container_v0_apriltags_us_letter.pdf",
        letter,
        grids,
    )

    print(f"Wrote SVGs to {svg_dir}")
    print(f"Wrote PDFs to {args.output_dir}")


if __name__ == "__main__":
    main()
