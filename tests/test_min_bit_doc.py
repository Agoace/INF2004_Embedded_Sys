"""Compare the minimum-bit proof tables in the document with the test output.

Usage: test_min_bit_doc.py <test_min_bit_proof binary> <document>
The document block between the min-bit markers must equal the output of
"test_min_bit_proof --markdown". Run that command and paste its output
between the markers when a pack layout changes.
"""
import difflib
import subprocess
import sys

BEGIN = "<!-- min-bit:begin -->"
END = "<!-- min-bit:end -->"


def main():
    binary, doc = sys.argv[1], sys.argv[2]
    want = subprocess.run([binary, "--markdown"], check=True,
                          capture_output=True, text=True).stdout
    try:
        text = open(doc, encoding="utf-8").read()
    except FileNotFoundError:
        print(f"missing document {doc}")
        return 1
    if text.count(BEGIN) != 1 or text.count(END) != 1:
        print("document needs exactly one begin and one end marker")
        return 1
    have = text.split(BEGIN, 1)[1].split(END, 1)[0].strip("\n") + "\n"
    want = want.strip("\n") + "\n"
    if have == want:
        print("min-bit document matches the proof output")
        return 0
    sys.stdout.writelines(difflib.unified_diff(
        have.splitlines(True), want.splitlines(True), "document", "proof"))
    return 1


if __name__ == "__main__":
    sys.exit(main())
