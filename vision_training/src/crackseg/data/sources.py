"""Where the public crack datasets come from and how we fetch them.

Two datasets can be downloaded directly from GitHub (DeepCrack and
CrackForest). Crack500 is only shared through Google Drive by its
authors, so it has to be downloaded by hand; `manual_steps` explains
where to put it so the prepare step can find it.

Raw downloads go to `data/raw/<name>/`. Nothing in here touches the
images themselves, that is the job of `converters.py`.
"""

from __future__ import annotations

import logging
import shutil
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path

from tqdm import tqdm

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class DatasetSource:
    name: str
    url: str | None
    citation: str
    notes: str
    manual_steps: str | None = None


SOURCES: dict[str, DatasetSource] = {
    "deepcrack": DatasetSource(
        name="deepcrack",
        url="https://github.com/yhlleo/DeepCrack/raw/master/dataset/DeepCrack.zip",
        citation="Liu et al., DeepCrack: A deep hierarchical feature learning "
        "architecture for crack segmentation, Neurocomputing 2019.",
        notes="537 images of concrete and asphalt, 544x384, masks are 0/255 PNG. "
        "Comes with an official 300/237 train/test split, which we keep.",
    ),
    "crackforest": DatasetSource(
        name="crackforest",
        url="https://github.com/cuilimeng/CrackForest-dataset/archive/refs/heads/master.zip",
        citation="Shi et al., Automatic road crack detection using random "
        "structured forests, IEEE T-ITS 2016.",
        notes="Urban road surface, 480x320. Only 118 of the 155 images have a "
        "ground truth .mat file; the rest are skipped. Label 2 means crack.",
    ),
    "crack500": DatasetSource(
        name="crack500",
        url=None,
        citation="Yang et al., Feature pyramid and hierarchical boosting network "
        "for pavement crack detection, IEEE T-ITS 2019.",
        notes="Pavement photos taken with a phone, cropped to 640x360 tiles. "
        "Closest to what the robot camera sees, so worth the manual download.",
        manual_steps=(
            "Crack500 is only on Google Drive. Get it from the link in "
            "https://github.com/fyangneil/pavement-crack-detection, unzip it and "
            "copy the traincrop, valcrop and testcrop folders into "
            "data/raw/crack500/. Each folder holds <name>.jpg and <name>_mask.png."
        ),
    ),
}


def download_file(url: str, target: Path) -> Path:
    """Stream a file to disk with a progress bar.

    Writes to a .part file first so an interrupted download never looks
    like a finished one on the next run.
    """
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_suffix(target.suffix + ".part")

    request = urllib.request.Request(url, headers={"User-Agent": "crackseg-downloader"})
    with urllib.request.urlopen(request) as response, open(partial, "wb") as out:
        total = int(response.headers.get("Content-Length", 0)) or None
        with tqdm(total=total, unit="B", unit_scale=True, desc=target.name) as bar:
            while chunk := response.read(1 << 20):
                out.write(chunk)
                bar.update(len(chunk))

    partial.replace(target)
    return target


def fetch(name: str, raw_root: Path, force: bool = False) -> Path:
    """Download and unzip one dataset into raw_root/<name>.

    Returns the folder with the extracted files. If the folder already has
    content we leave it alone unless force is set, since these downloads
    are tens of megabytes and the files never change upstream.
    """
    source = SOURCES[name]
    out_dir = raw_root / name

    if source.url is None:
        if not out_dir.exists() or not any(out_dir.iterdir()):
            raise FileNotFoundError(
                f"{name} has no direct download.\n{source.manual_steps}"
            )
        log.info("%s: using the manually downloaded copy in %s", name, out_dir)
        return out_dir

    if out_dir.exists() and any(out_dir.iterdir()) and not force:
        log.info("%s: already in %s, skipping (use --force to redownload)", name, out_dir)
        return out_dir

    if force and out_dir.exists():
        shutil.rmtree(out_dir)

    archive = raw_root / "_archives" / f"{name}.zip"
    if not archive.exists() or force:
        log.info("%s: downloading %s", name, source.url)
        download_file(source.url, archive)

    log.info("%s: extracting to %s", name, out_dir)
    with zipfile.ZipFile(archive) as zf:
        zf.extractall(out_dir)
    return out_dir
