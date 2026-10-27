"""Generate original MIDI encodings of a public-domain melody and a scale."""
from pathlib import Path
import mido


def write_melody(path, sequence, bpm=90):
    midi = mido.MidiFile(type=1, ticks_per_beat=480)
    conductor = mido.MidiTrack()
    conductor.append(mido.MetaMessage("track_name", name="Tempo"))
    conductor.append(mido.MetaMessage("set_tempo", tempo=mido.bpm2tempo(bpm)))
    conductor.append(mido.MetaMessage("time_signature", numerator=4, denominator=4))
    midi.tracks.append(conductor)
    melody = mido.MidiTrack()
    melody.append(mido.MetaMessage("track_name", name=path.stem))
    for pitch, beats in sequence:
        melody.append(mido.Message("note_on", channel=0, note=pitch, velocity=72, time=0))
        melody.append(mido.Message("note_off", channel=0, note=pitch, velocity=0, time=round(beats * 480)))
    midi.tracks.append(melody)
    path.parent.mkdir(parents=True, exist_ok=True)
    midi.save(path)


def main():
    root = Path(__file__).resolve().parent / "examples"
    # Ode to Joy, opening 8 bars, simplified single melody in C major.
    melody = [(64,1),(64,1),(65,1),(67,1),(67,1),(65,1),(64,1),(62,1),
              (60,1),(60,1),(62,1),(64,1),(64,1.5),(62,.5),(62,2),
              (64,1),(64,1),(65,1),(67,1),(67,1),(65,1),(64,1),(62,1),
              (60,1),(60,1),(62,1),(64,1),(62,1.5),(60,.5),(60,2)]
    write_melody(root / "ode_to_joy.mid", melody)
    write_melody(root / "seven_note_scale.mid", [(n,1) for n in (60,62,64,65,67,69,71,69,67,65,64,62,60)], 60)
    print("已生成欢乐颂（开头八小节简化版）和七音上下行音阶。")


if __name__ == "__main__":
    main()
