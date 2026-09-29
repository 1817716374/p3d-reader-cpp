#!/usr/bin/env python3
"""Run p3d-inspect on external samples and inventory unresolved attributes.

No proprietary samples are copied into the repository. Reports are evidence of
the exercised files, not a claim of semantic completeness for the format.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import time


def attribute_audit(document):
    counts = Counter()
    groups = {}
    raw_encodings = {"opaque", "compressed_binary", "uncompressed_binary"}
    for record in document.get("graphics_records", []):
        for ordinal, attribute in enumerate(record.get("attributes", [])):
            decoded = attribute.get("decoded", {})
            encoding = decoded.get("encoding", "missing")
            status = decoded.get("status")
            if decoded.get("decode_error") or attribute.get("decode_error"):
                category = "decode_error"
            elif status in {"partial", "invalid", "unsupported", "unresolved"}:
                category = status
            elif encoding in raw_encodings or encoding == "missing":
                category = "raw_only"
            else:
                category = "structured"
            counts[category] += 1
            # Structured is not synonymous with complete. Inventory every
            # encoding so later revisions can examine unresolved inner fields.
            key = (attribute.get("group"), attribute.get("key"), attribute.get("index"),
                   encoding, category)
            if key not in groups:
                groups[key] = {"group": key[0], "key": key[1], "index": key[2],
                               "encoding": encoding, "category": category,
                               "count": 0, "examples": []}
            row = groups[key]
            row["count"] += 1
            if len(row["examples"]) < 3:
                row["examples"].append({"stream": record.get("stream"),
                                        "record_id": record.get("id"),
                                        "record_offset": record.get("offset"),
                                        "attribute_ordinal": ordinal,
                                        "attribute_offset": attribute.get("offset"),
                                        "payload_bytes": attribute.get("payload", {}).get("bytes")})
    return {"scope": "graphics_records.attributes; structured does not mean complete",
            "total_attributes": sum(counts.values()), "categories": dict(counts),
            "groups": sorted(groups.values(), key=lambda row: (-row["count"], str(row["key"]))) }


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path, help="P3D files or directories (recursive)")
    parser.add_argument("--inspector", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=300)
    args = parser.parse_args()
    files = set()
    for source in args.inputs:
        if not source.exists():
            parser.error(f"input does not exist: {source}")
        if source.is_dir():
            files.update(p.resolve() for p in source.rglob("*")
                         if p.is_file() and p.suffix.lower() == ".p3d")
        else:
            files.add(source.resolve())
    if not files:
        parser.error("no P3D files found")
    inspector = args.inspector.resolve(strict=True)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for number, source in enumerate(sorted(files)):
        folder = output / f"{number:03d}"
        folder.mkdir(exist_ok=True)
        with source.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        command = [str(inspector), str(source), "--threads", str(args.threads),
                   "--native-summary", "--dump", str(folder / "document.json")]
        result = {"source": str(source), "sha256": digest, "bytes": source.stat().st_size,
                  "command": command, "output": str(folder), "execution_status": "pending"}
        start = time.monotonic()
        try:
            with (folder / "stdout.json").open("wb") as stdout, \
                    (folder / "stderr.txt").open("wb") as stderr:
                process = subprocess.run(command, stdout=stdout, stderr=stderr, timeout=args.timeout)
            result["exit_code"] = process.returncode
            if process.returncode:
                result["execution_status"] = "failed"
            else:
                result["summary"] = json.loads((folder / "stdout.json").read_text(encoding="utf-8"))
                document = json.loads((folder / "document.json").read_text(encoding="utf-8"))
                audit = attribute_audit(document)
                write_json(folder / "attribute-audit.json", audit)
                result["attribute_categories"] = audit["categories"]
                result["total_attributes"] = audit["total_attributes"]
                if "display_style_sources" in document:
                    styles = document["display_style_sources"]
                    result["display_style_sources"] = {
                        "tables": len(styles["tables"]),
                        "references": len(styles["references"]),
                        "reference_statuses": dict(Counter(r["status"] for r in styles["references"])),
                        "runtime_resolution": styles["runtime_resolution"],
                    }
                result["execution_status"] = "completed"
                del document
        except subprocess.TimeoutExpired:
            result["execution_status"] = "timeout"
        except (OSError, ValueError) as error:
            result["execution_status"] = "failed"
            result["error"] = str(error)
        result["seconds"] = time.monotonic() - start
        results.append(result)
        write_json(output / "corpus-report.json", {
            "schema_version": 1, "format_completeness": "not_established", "files": results})
        print(f"{number + 1}/{len(files)} {source.name}: {result['execution_status']}", flush=True)
    return int(any(row["execution_status"] != "completed" for row in results))


if __name__ == "__main__":
    raise SystemExit(main())
