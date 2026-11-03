"""Archive expected build identity; this is not proof of what is flashed."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def identity(path):
    return dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest(), bytes=path.stat().st_size)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--params', type=Path, required=True, help='actual eIDE temporary builder params used for this build')
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    if a.out.exists():
        raise FileExistsError(a.out)
    cfg = json.loads(a.params.read_text(encoding='utf-8-sig'))
    if Path(cfg['rootDir']).resolve() != ROOT:
        raise ValueError('build root does not match this checkout')
    out = Path(cfg['outDir'])
    if not out.is_absolute():
        out = ROOT / out
    sources = {}
    for group in ('imcalib', 'Core', 'USB_DEVICE', 'X-CUBE-AI/App'):
        for source in (ROOT / group).rglob('*'):
            if source.is_file() and source.suffix.lower() in ('.c', '.h', '.s'):
                sources[str(source.relative_to(ROOT))] = identity(source)
    def git(*args):
        return subprocess.check_output(['git', *args], cwd=ROOT)
    record = dict(format='h7-expected-timing-firmware', version=1, expected_build_only=True,
                  head=git('rev-parse', 'HEAD').decode().strip(),
                  dirty_status=git('status', '--short').decode('utf-8', errors='replace'),
                  dirty_patch_sha256=hashlib.sha256(git('diff', '--binary')).hexdigest(),
                  builder_params=identity(a.params), defines=cfg.get('defines'), sources=sources,
                  artifacts={ext: identity(out / (cfg['name'] + '.' + ext)) for ext in ('elf', 'bin', 'hex')})
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8')
    print(a.out)


if __name__ == '__main__':
    main()
