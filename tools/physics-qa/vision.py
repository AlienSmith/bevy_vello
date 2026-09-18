#!/usr/bin/env python3
"""Send screenshots to a vision model and get a structured QA verdict.

This is the generic, self-contained path: it talks to an OpenAI-compatible
`/chat/completions` endpoint directly. It is intended for batch runs, or as a
fallback when the tuning loop is driven by an agent that can read images itself
(preferable, because then no script ever handles the credential).

The key is read from the environment **only**:

    export VOLC_API_KEY=...        # or any key for your endpoint
    ./vision.py --prompt-file qa_prompt.txt --base-url URL --model NAME shot*.png

Prints a single JSON object on stdout. The exit code is non-zero only on
transport failure, so the caller can distinguish "model says bad" from "call
failed".
"""
import argparse
import base64
import json
import os
import sys
import urllib.request

DEFAULT_BASE_URL = "https://ark.cn-beijing.volces.com/api/plan/v3/chat/completions"
DEFAULT_MODEL = "glm-5.3-flash"
DEFAULT_KEY_ENV = "VOLC_API_KEY"


def api_key(key_env: str) -> str:
    """Read the key from the environment.

    Deliberately no fallback to any credential file: a script that reaches into a
    credential store keeps working after the key is rotated or revoked elsewhere,
    and exposes the secret to anything that can read the workspace.
    """
    key = os.environ.get(key_env)
    if not key:
        raise SystemExit(
            f"{key_env} is not set. Export it, or drive the judge from an agent "
            f"that can read images itself so no script needs the key."
        )
    return key


def data_url(path: str) -> str:
    with open(path, "rb") as fh:
        raw = fh.read()
    ext = os.path.splitext(path)[1].lstrip(".").lower() or "png"
    if ext == "jpg":
        ext = "jpeg"
    return f"data:image/{ext};base64,{base64.b64encode(raw).decode()}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--prompt-file", required=True)
    ap.add_argument("--base-url", default=DEFAULT_BASE_URL)
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--key-env", default=DEFAULT_KEY_ENV,
                    help=f"environment variable holding the API key (default {DEFAULT_KEY_ENV})")
    ap.add_argument("--max-tokens", type=int, default=4000,
                    help="generous by default: reasoning models return empty content when truncated")
    ap.add_argument("images", nargs="+")
    args = ap.parse_args()

    with open(args.prompt_file) as fh:
        prompt = fh.read()

    content = [{"type": "text", "text": prompt}]
    for img in args.images:
        content.append({"type": "image_url", "image_url": {"url": data_url(img)}})

    body = json.dumps(
        {
            "model": args.model,
            "messages": [{"role": "user", "content": content}],
            "max_tokens": args.max_tokens,
            "temperature": 0.0,
        }
    ).encode()

    req = urllib.request.Request(
        args.base_url,
        data=body,
        headers={
            "Authorization": f"Bearer {api_key(args.key_env)}",
            "Content-Type": "application/json",
        },
    )
    try:
        with urllib.request.urlopen(req, timeout=180) as resp:
            payload = json.load(resp)
    except Exception as exc:  # noqa: BLE001 - surface any transport failure verbatim
        print(json.dumps({"error": f"{type(exc).__name__}: {exc}"}))
        return 1

    msg = payload["choices"][0]["message"]
    print(
        json.dumps(
            {
                "model": payload.get("model"),
                "content": msg.get("content"),
                "reasoning": msg.get("reasoning_content"),
                "usage": payload.get("usage"),
            }
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
