#!/usr/bin/env python3
"""Compare every packaged application file with its immutable upstream revision."""
import argparse
import hashlib
import json
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('revision')
    parser.add_argument('component', choices=('agent-core', 'perception', 'actucore'))
    parser.add_argument('image')
    args = parser.parse_args()
    prefixes = {
        'agent-core': ('src/', 'web/', 'tools/', 'resource/', 'deploy/'),
        'perception': ('main.py', 'plugins/', 'utils/', 'tools/', 'config.yaml', 'deploy/'),
        'actucore': ('main.py', 'plugins/', 'config.yaml', 'deploy/'),
    }[args.component]
    tree = subprocess.check_output(['git', '-C', args.source, 'ls-tree', '-r', '--name-only', args.revision, args.component + '/']).decode().splitlines()
    expected = {}
    for path in tree:
        relative = path[len(args.component) + 1:]
        if not any(relative.startswith(p) if p.endswith('/') else relative == p for p in prefixes):
            continue
        destination = '/' + relative if relative.startswith('deploy/') else '/work/' + relative
        content = subprocess.check_output(['git', '-C', args.source, 'show', args.revision + ':' + path])
        expected[destination] = hashlib.sha256(content).hexdigest()
    assert expected, 'no application files found'
    verify = '''import hashlib,json,pathlib,sys
expected=json.load(sys.stdin)
errors=[]
for name,digest in expected.items():
    p=pathlib.Path(name)
    if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
        errors.append(name)
if errors:
    raise SystemExit("source mismatch: "+", ".join(errors))
print("UNMODIFIED_APPLICATION_SOURCE_PASS files="+str(len(expected)))
'''
    subprocess.run(['docker', 'run', '--rm', '-i', '--network', 'none', '--entrypoint', 'python3', args.image, '-c', verify], input=json.dumps(expected).encode(), check=True)


if __name__ == '__main__':
    main()
