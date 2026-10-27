"""Direct-file entry point, also usable from a working directory elsewhere."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from midi_piano.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
