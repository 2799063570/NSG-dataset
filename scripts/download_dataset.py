#!/usr/bin/env python3
"""Download and verify the raw NSG CSV snapshot using the Python standard library."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import urllib.request

REPO_ROOT = Path(__file__).resolve().parents[1]
BASE_URL = "https://github.com/2799063570/NSG-dataset/releases/download/dataset-26.05.21/"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(4 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def fetch(name, size, checksum, output_dir, assets_dir):
    target = output_dir / name
    if target.exists():
        if target.stat().st_size == size and sha256(target) == checksum:
            return target
        raise SystemExit(f"Checksum mismatch: {target}")
    partial = target.with_name(target.name + ".part")
    if assets_dir:
        shutil.copyfile(assets_dir / name, partial)
    else:
        request = urllib.request.Request(BASE_URL + name, headers={"User-Agent": "NSG-dataset-downloader"})
        print(f"Downloading {name}...", flush=True)
        with urllib.request.urlopen(request, timeout=120) as response, partial.open("wb") as output:
            shutil.copyfileobj(response, output, length=4 * 1024 * 1024)
    if partial.stat().st_size != size or sha256(partial) != checksum:
        raise SystemExit(f"Download checksum mismatch: {partial}")
    partial.replace(target)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "data/dataset/26.05.21")
    parser.add_argument("--assets-dir", type=Path, help="Use already-downloaded release assets instead of the network")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((REPO_ROOT / "data/dataset-manifest.json").read_text(encoding="utf-8"))
    for entry in manifest["files"]:
        target = args.output_dir / entry["file"]
        if target.exists():
            if target.stat().st_size == entry["bytes"] and sha256(target) == entry["sha256"]:
                print(f"Already verified: {target.name}", flush=True)
                continue
            raise SystemExit(f"Refusing to overwrite a different existing file: {target}")
        compressed = args.output_dir / entry["compressed_file"]
        if not compressed.exists():
            if entry.get("parts"):
                pieces = [fetch(part["file"], part["bytes"], part["sha256"], args.output_dir, args.assets_dir) for part in entry["parts"]]
                partial = compressed.with_name(compressed.name + ".part")
                print(f"Reassembling {compressed.name}...", flush=True)
                with partial.open("wb") as output:
                    for piece in pieces:
                        with piece.open("rb") as source:
                            shutil.copyfileobj(source, output, length=4 * 1024 * 1024)
                if partial.stat().st_size != entry["compressed_bytes"] or sha256(partial) != entry["compressed_sha256"]:
                    raise SystemExit(f"Reassembled checksum mismatch: {partial}")
                partial.replace(compressed)
            else:
                fetch(entry["compressed_file"], entry["compressed_bytes"], entry["compressed_sha256"], args.output_dir, args.assets_dir)
        elif sha256(compressed) != entry["compressed_sha256"]:
            raise SystemExit(f"Compressed checksum mismatch: {compressed}")
        partial = target.with_name(target.name + ".part")
        print(f"Decompressing and verifying {target.name}...", flush=True)
        with gzip.open(compressed, "rb") as source, partial.open("wb") as output:
            shutil.copyfileobj(source, output, length=4 * 1024 * 1024)
        if partial.stat().st_size != entry["bytes"] or sha256(partial) != entry["sha256"]:
            raise SystemExit(f"Raw CSV checksum mismatch: {partial}")
        partial.replace(target)
        print(f"Verified: {target.name} ({entry['bytes']:,} bytes)", flush=True)


if __name__ == "__main__":
    main()
