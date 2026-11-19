"""Build one portable HTML with local scores and original white-key WAV samples."""
import base64
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def build(output=None):
    songs = [{"name": p.stem, "text": p.read_text(encoding="utf-8-sig")}
             for p in sorted((ROOT / "music_data").glob("*.txt"))]
    # Keep the library intact; strict validation happens before playback.
    demo = "left_arm_move(17, 0.2)\nright_arm_move(24, 0.2)\nsleep(0.2)\n"
    for fingers in ("17", "28", "39", "4x", "17"):
        demo += f"hand(D,{fingers})\nsleep(0.3)\nhand(U,{fingers})\nsleep(0.1)\n"
    songs.insert(0, {"name": "四指试听（备用短谱）", "text": demo})
    samples = {str(m): base64.b64encode((ROOT / "samples" / f"tone ({m - 20}).wav").read_bytes()).decode("ascii")
               for m in range(21, 109) if m % 12 in (0, 2, 4, 5, 7, 9, 11)}
    data = json.dumps({"songs": songs, "samples": samples}, ensure_ascii=False).replace("<", "\\u003c")
    template = (ROOT / "web" / "standalone.html").read_text(encoding="utf-8")
    # Inline the existing MIT-licensed engine as data modules so file:// works
    # offline, without a CDN, a local server or browser CORS exceptions.
    lib = ROOT.parents[1] / "src/openarm_app/web/lib"
    core_url = "data:text/javascript;base64," + base64.b64encode(
        (lib / "three.core.js").read_bytes()).decode("ascii")
    three = (lib / "three.module.js").read_text(encoding="utf-8")
    three = three.replace("'./three.core.js'", json.dumps(core_url))
    three_url = "data:text/javascript;base64," + base64.b64encode(
        three.encode("utf-8")).decode("ascii")
    replacements = {
        "__ASSETS__": data,
        "__STYLE__": "\n".join((ROOT / "web" / name).read_text(encoding="utf-8")
                                for name in ("standalone.css", "standalone-stage.css")),
        "__SCENE__": (ROOT / "web/standalone-stage.js").read_text(encoding="utf-8"),
        "__APP__": (ROOT / "web/standalone-app.js").read_text(encoding="utf-8"),
        "__THREE_URL__": three_url,
        "__THREE_LICENSE__": (lib / "THREE-LICENSE.txt").read_text(encoding="utf-8"),
    }
    for token, content in replacements.items():
        template = template.replace(token, content)
    target = Path(output).resolve() if output else ROOT / "piano_standalone.html"
    target.write_text(template, encoding="utf-8")
    print(f"Built {target}: {len(songs)} songs, {len(samples)} samples, {target.stat().st_size / 1024**2:.1f} MiB")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Optional path for the portable HTML")
    build(parser.parse_args().output)
