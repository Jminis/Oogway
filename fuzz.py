#!/usr/bin/env python3
import os
import sys
import subprocess
import random
import time
from datetime import datetime
from pathlib import Path

IMAGE_NAME = "ros2_libfuzzer"
TIMEOUT_SEC = 2 * 60 * 60  # 2시간

WORKDIR = Path(__file__).resolve().parent
SRC_DIR = WORKDIR / "src" / "ros2_fuzz" / "src"
RESULTS_DIR = WORKDIR / "results"
LOG_DIR = RESULTS_DIR / "logs"
CORPUS_BASE = RESULTS_DIR / "corpus"
CRASH_BASE = RESULTS_DIR / "crashes"
SAVED_CRASH_LIST = RESULTS_DIR / "saved_crash_list.txt"

def list_fuzzers():
    return sorted([f.stem for f in SRC_DIR.glob("fuzz_*.cpp")])

def ensure_dirs(target):
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    (CORPUS_BASE / target).mkdir(parents=True, exist_ok=True)
    (CRASH_BASE / target).mkdir(parents=True, exist_ok=True)
    SAVED_CRASH_LIST.parent.mkdir(parents=True, exist_ok=True)
    SAVED_CRASH_LIST.touch(exist_ok=True)

def docker_image_exists(name):
    result = subprocess.run(["docker", "images", "-q", name], capture_output=True, text=True)
    return bool(result.stdout.strip())

def build_docker_image():
    print(f"[+] Building Docker image '{IMAGE_NAME}'...")
    subprocess.run(["docker", "build", "-t", IMAGE_NAME, "."], check=True)

def load_seen_summaries():
    return set(line.strip() for line in SAVED_CRASH_LIST.read_text(errors='ignore').splitlines())

def append_summary(summary_line):
    with open(SAVED_CRASH_LIST, "a") as f:
        f.write(summary_line + "\n")

def run_fuzzer(target):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = LOG_DIR / f"{target}_{timestamp}.log"
    corpus_dir = CORPUS_BASE / target
    crash_dir = CRASH_BASE / target

    print(f"[+] Running fuzzer: {target} (timeout: {TIMEOUT_SEC // 60} min)")
    with open(log_path, "w") as logfile:
        process = subprocess.Popen([
            "docker", "run", "--rm", "-i",
            "-v", f"{corpus_dir.absolute()}:/corpus",
            "-v", f"{crash_dir.absolute()}:/crashes",
            "-e", "ASAN_OPTIONS=detect_leaks=0:new_delete_type_mismatch=0",
            IMAGE_NAME,
            f"/workspace/install/ros2_fuzz/lib/ros2_fuzz/{target}",
            "-rss_limit_mb=4096",
            "-artifact_prefix=/crashes/",
            "/corpus"
        ], stdout=logfile, stderr=subprocess.STDOUT)

        try:
            process.wait(timeout=TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            print(f"[!] Timeout after {TIMEOUT_SEC // 60} minutes. Killing fuzzer.")
            process.kill()
            logfile.write(f"\n[!] Terminated due to timeout ({TIMEOUT_SEC} sec)\n")

    # 분석: log 저장 여부 판단
    logfile.close()
    log_text = log_path.read_text(errors='ignore')

    # 1. timeout 로그는 삭제
    if "Terminated due to timeout" in log_text:
        print("[-] Timeout-only log. Deleted.")
        log_path.unlink(missing_ok=True)
        return

    # 2. 중복 SUMMARY 검사
    seen = load_seen_summaries()
    for line in log_text.splitlines():
        if line.startswith("SUMMARY: "):
            if line in seen:
                print("[-] Duplicate crash SUMMARY. Log deleted.")
                log_path.unlink(missing_ok=True)
                return
            else:
                print("[+] New crash summary. Saving and recording.")
                append_summary(line)
                return

    print("[+] No crash. Log saved.")

def main():
    fuzzers = list_fuzzers()

    if len(sys.argv) == 2 and sys.argv[1] == "auto":
        print("[!] Running in AUTO mode (infinite fuzz loop)")
        if not docker_image_exists(IMAGE_NAME):
            build_docker_image()
        while True:
            target = random.choice(fuzzers)
            print(f"\n[!] Selected fuzzer: {target}")
            ensure_dirs(target)
            run_fuzzer(target)
            print("[+] Completed. Selecting next fuzzer...")
            time.sleep(2)
        return

    if len(sys.argv) != 2:
        print("Usage:")
        print("  python3 fuzz.py <fuzz_target>")
        print("  python3 fuzz.py auto\n")
        print("Available fuzzers:")
        for fuzzer in fuzzers:
            print(f"  • {fuzzer}")
        sys.exit(1)

    target = sys.argv[1]
    if target not in fuzzers:
        print(f"[!] Unknown fuzzer: {target}")
        print("Available fuzzers:")
        for fuzzer in fuzzers:
            print(f"  • {fuzzer}")
        sys.exit(1)

    ensure_dirs(target)

    if not docker_image_exists(IMAGE_NAME):
        build_docker_image()

    run_fuzzer(target)

if __name__ == "__main__":
    main()
