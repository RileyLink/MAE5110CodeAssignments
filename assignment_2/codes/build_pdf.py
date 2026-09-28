"""Restore local figures and build a PDF copy of the source-only report.

Run from the repository root: uv run python assignment_2/codes/build_pdf.py
Requires Pandoc and XeLaTeX (MacTeX); uses the installed MacTeX path if needed.
"""

import re
import shutil
import subprocess
from pathlib import Path

from PIL import Image


def prepare_markdown(text):
    """Restore hidden image references without changing the source report.

    GitHub displays generation instructions; the local PDF uses the original
    figure references stored in LOCAL_FIGURE comments. This function is pure.
    """
    text = re.sub(r"<!-- LOCAL_FIGURE\s*\n(.*?)\n-->", r"\1", text, flags=re.DOTALL)
    text = text.replace("(figures/walker.gif)", "(figures/walker_pdf.png)")
    text = text.replace(
        "![Controlled walking followed by torque-limited standing, shown at half speed.]",
        "![Final frame of the controlled walking and standing animation.]",
    )
    text = text.replace(
        "Open `figures/walker.gif` locally to view the latest generated run; running either example below regenerates its specified output.",
        "The image above is the final frame of the latest generated run. Open the locally generated GIF to view the animation.",
    )
    return text


def main():
    folder = Path(__file__).resolve().parents[1]
    pandoc = shutil.which("pandoc")
    engine = shutil.which("xelatex")
    if engine is None and Path("/Library/TeX/texbin/xelatex").is_file():
        engine = "/Library/TeX/texbin/xelatex"
    if not pandoc or not engine:
        raise SystemExit("This build requires Pandoc and XeLaTeX (MacTeX).")
    with Image.open(folder / "figures/walker.gif") as gif:
        gif.seek(gif.n_frames - 1)
        gif.convert("RGB").save(folder / "figures/walker_pdf.png")
    text = prepare_markdown((folder / "assignment_2.md").read_text())
    (folder / "assignment_2_pdf.md").write_text(text)
    subprocess.run(
        [
            pandoc,
            "assignment_2_pdf.md",
            "--pdf-engine=" + engine,
            "--resource-path=.",
            "--lua-filter=codes/pdf_layout.lua",
            "--include-in-header=codes/pdf_layout.tex",
            "-V",
            "geometry:margin=1in",
            "-V",
            "fontsize=11pt",
            "-o",
            "assignment_2.pdf",
        ],
        cwd=folder,
        check=True,
    )
    print("Created", folder / "assignment_2.pdf")


if __name__ == "__main__":
    main()
