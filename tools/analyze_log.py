#!/usr/bin/env python3
import csv
import math
import statistics
import sys
from pathlib import Path


def parse_samples(raw: str) -> list[int]:
    return [int(part) for part in raw.split(",") if part]


def flatten_batches(rows: list[dict[str, str]], key: str) -> list[int]:
    samples: list[int] = []
    for row in rows:
        samples.extend(parse_samples(row[key]))
    return samples


def estimate_sample_rate(rows: list[dict[str, str]]) -> float:
    if len(rows) < 2:
        return 0.0
    sample_count = 0
    for row in rows:
        sample_count += len(parse_samples(row["ir_samples"]))
    start_ms = int(rows[0]["timestamp_ms"])
    end_ms = int(rows[-1]["timestamp_ms"])
    elapsed = (end_ms - start_ms) / 1000.0
    if elapsed <= 0:
        return 0.0
    return sample_count / elapsed


def describe(name: str, samples: list[int]) -> None:
    if not samples:
        print(f"{name}: no samples")
        return
    mean = statistics.fmean(samples)
    minimum = min(samples)
    maximum = max(samples)
    span = maximum - minimum
    stdev = statistics.pstdev(samples)
    print(f"{name}: n={len(samples)} mean={mean:.1f} min={minimum} max={maximum} span={span} stdev={stdev:.2f}")


def highpass(samples: list[int], alpha: float = 0.01) -> list[float]:
    if not samples:
        return []
    dc = float(samples[0])
    filtered: list[float] = []
    smooth = 0.0
    for sample in samples:
        value = float(sample)
        dc += alpha * (value - dc)
        smooth += 0.18 * ((value - dc) - smooth)
        filtered.append(smooth)
    return filtered


def rough_bpm(samples: list[int], sample_rate: float) -> float:
    if sample_rate <= 0 or len(samples) < int(sample_rate * 4):
        return 0.0
    filtered = highpass(samples)
    mean = statistics.fmean(filtered)
    centered = [value - mean for value in filtered]
    rms = math.sqrt(sum(value * value for value in centered) / len(centered))
    threshold = max(24.0, rms * 0.30)
    min_interval = int(sample_rate * 60.0 / 220.0)
    max_interval = int(sample_rate * 60.0 / 38.0)
    peaks: list[int] = []
    last_peak = -max_interval
    for i in range(1, len(centered) - 1):
        if centered[i] <= threshold:
            continue
        if centered[i] <= centered[i - 1] or centered[i] < centered[i + 1]:
            continue
        if i - last_peak < min_interval:
            continue
        if i - last_peak <= max_interval or not peaks:
            peaks.append(i)
            last_peak = i
    if len(peaks) < 3:
        return 0.0
    intervals = [b - a for a, b in zip(peaks, peaks[1:])]
    median_interval = statistics.median(intervals)
    return 60.0 * sample_rate / median_interval


def batch_mean(row: dict[str, str], key: str) -> float:
    samples = parse_samples(row[key])
    if not samples:
        return 0.0
    return statistics.fmean(samples)


def stable_segments(rows: list[dict[str, str]]) -> list[tuple[int, int]]:
    if not rows:
        return []
    means = [batch_mean(row, "ir_samples") for row in rows]
    segments: list[tuple[int, int]] = []
    start = 0
    for i in range(1, len(rows)):
        previous = means[i - 1]
        current = means[i]
        unstable = (
            current < 5000
            or abs(current - previous) > 1200
            or abs(current - previous) / max(current, previous, 1.0) > 0.12
        )
        if unstable:
            if i - start >= 8:
                segments.append((start, i - 1))
            start = i
    if len(rows) - start >= 8:
        segments.append((start, len(rows) - 1))
    return segments


def print_segments(rows: list[dict[str, str]]) -> None:
    segments = stable_segments(rows)
    if not segments:
        print("stable_segments: none")
        return

    print("stable_segments:")
    for index, (start, end) in enumerate(segments, start=1):
        segment_rows = rows[start : end + 1]
        ir = flatten_batches(segment_rows, "ir_samples")
        red = flatten_batches(segment_rows, "red_samples")
        sample_rate = estimate_sample_rate(segment_rows)
        start_ms = int(segment_rows[0]["timestamp_ms"])
        end_ms = int(segment_rows[-1]["timestamp_ms"])
        duration = (end_ms - start_ms) / 1000.0
        print(
            f"  #{index}: batches={start}-{end} duration={duration:.1f}s "
            f"sample_rate={sample_rate:.2f}Hz rough_ir_bpm={rough_bpm(ir, sample_rate):.1f}"
        )
        print(
            f"      IR mean={statistics.fmean(ir):.1f} span={max(ir) - min(ir)} "
            f"stdev={statistics.pstdev(ir):.2f}"
        )
        print(
            f"      RED mean={statistics.fmean(red):.1f} span={max(red) - min(red)} "
            f"stdev={statistics.pstdev(red):.2f}"
        )


def analyze(path: Path) -> int:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle, delimiter="\t"))
    if not rows:
        print(f"file: {path}")
        print("No rows found")
        return 1

    ir = flatten_batches(rows, "ir_samples")
    red = flatten_batches(rows, "red_samples")
    sample_rate = estimate_sample_rate(rows)
    print(f"file: {path}")
    print(f"batches={len(rows)} estimated_sample_rate={sample_rate:.2f} Hz")
    describe("IR", ir)
    describe("RED", red)
    print(f"rough_ir_bpm={rough_bpm(ir, sample_rate):.1f}")
    print_segments(rows)
    return 0


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: analyze_log.py path/to/corbi.tsv [more.tsv ...]", file=sys.stderr)
        return 1

    status = 0
    for index, arg in enumerate(sys.argv[1:]):
        if index > 0:
            print()
        status = max(status, analyze(Path(arg)))
    return status


if __name__ == "__main__":
    raise SystemExit(main())
