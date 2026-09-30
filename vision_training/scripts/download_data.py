"""Download the public crack datasets into data/raw/.

    python scripts/download_data.py                 # everything that can be downloaded
    python scripts/download_data.py deepcrack       # just one
    python scripts/download_data.py --list          # show what is available

Crack500 has to be fetched by hand (Google Drive). Run with --list to
see the instructions.
"""

import argparse
import logging

from crackseg.data.sources import SOURCES, fetch
from crackseg.utils import resolve_path, setup_logging

log = logging.getLogger("download")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("names", nargs="*", help="datasets to fetch (default: all with a direct link)")
    parser.add_argument("--raw-root", default="data/raw")
    parser.add_argument("--force", action="store_true", help="download again even if present")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()
    setup_logging()

    if args.list:
        for s in SOURCES.values():
            how = s.url or "manual download"
            print(f"{s.name:12s} {how}\n{'':12s} {s.notes}")
            if s.manual_steps:
                print(f"{'':12s} {s.manual_steps}")
            print(f"{'':12s} Cite: {s.citation}\n")
        return

    names = args.names or [n for n, s in SOURCES.items() if s.url]
    raw_root = resolve_path(args.raw_root)
    failed = []
    for name in names:
        if name not in SOURCES:
            parser.error(f"unknown dataset '{name}', choose from {sorted(SOURCES)}")
        try:
            fetch(name, raw_root, force=args.force)
        except FileNotFoundError as e:
            log.warning("%s", e)
            failed.append(name)

    if failed:
        log.warning("Not available yet: %s", ", ".join(failed))
    log.info("Done. Next step: python scripts/prepare_data.py")


if __name__ == "__main__":
    main()
