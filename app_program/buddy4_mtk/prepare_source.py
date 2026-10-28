"""Create a disposable professor-based team build; never patch the source repo."""
from pathlib import Path
import hashlib
import json
import shutil

project = Path(__file__).resolve().parent
repo = project.parent.parent
target = project / 'build' / 'source'
assert (repo / 'include' / 'tk' / 'tkernel.h').is_file(), 'Place this folder in app_program/buddy4_mtk'
expected = json.loads((project / 'base_hashes.json').read_text())
for name, digest in expected.items():
    live = repo / name
    assert hashlib.sha256(live.read_text(encoding='utf-8-sig').encode('utf-8')).hexdigest() == digest, (
        f'Base changed: {name}. Review/rebase this trial overlay before building; source was not modified.')

# Remove only this documented disposable build copy, after validating its path.
resolved = target.resolve()
assert resolved.parent == (project / 'build').resolve() and resolved.name == 'source'
if target.exists():
    assert not target.is_symlink(), 'Unexpected link at disposable build directory'
    shutil.rmtree(target)
target.mkdir(parents=True)
ignore = shutil.ignore_patterns('.git', 'buddy4_mtk', 'build', 'build-host',
    'build-buddy4', 'build-buddy4-host', 'evidence', '*.o', '*.d', '*.elf',
    '*.uf2', '*.map', '*.exe', '.build_profile')
for name in ['app_program', 'build_make', 'config', 'device', 'etc', 'include',
             'kernel', 'lib', 'tools']:
    shutil.copytree(repo / name, target / name, ignore=ignore)
for source in (project / 'overlay').rglob('*'):
    if source.is_file():
        dest = target / source.relative_to(project / 'overlay')
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, dest)
print(f'Prepared trial build copy: {target}')
print('The team startup, kernel/device sources and working IMU project were not changed.')
