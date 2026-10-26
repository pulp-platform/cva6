#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""SRAM inventory of the design as synthesis elaborates it.

Lists every `tc_sram` with its shape. Elaborated with a technology's tc_sram,
whose memory macros are black boxes (--ignore-unknown-modules), it also lists
the macros that tc_sram instantiates for each SRAM and, given the technology's
macro list (--cuts), checks that every SRAM has macros and that every macro is
in the list; a shape that tc_sram does not know reaches its $fatal, an
elaboration error here.

Arguments not listed below go to slang.
"""

import argparse
import json
import os
import sys
from collections import Counter, defaultdict

try:  # pyslang >= 10
    from pyslang import DiagnosticEngine
    from pyslang.ast import SymbolKind as SK
    from pyslang.driver import CommandLineOptions, Driver
except ImportError:  # pyslang 9 (oseda 2025.12)
    from pyslang import CommandLineOptions, DiagnosticEngine, Driver
    from pyslang import SymbolKind as SK

SRAM_DEF = 'tc_sram'
SRAM_PARAMS = ('NumWords', 'DataWidth', 'ByteWidth', 'NumPorts', 'Latency', 'ImplKey')


def param_value(sym):
    """Parameter value as int if integral (e.g. `32'd256`), else as string."""
    val = getattr(sym, 'value', None)
    if val is None:
        return None
    text = str(val).strip()
    if text.startswith('"'):
        return text.strip('"')
    size, tick, rest = text.partition("'")
    if not tick:
        return int(size) if size.lstrip('-').isdigit() else text
    rest = rest.lstrip('sS')
    base = {'d': 10, 'h': 16, 'b': 2, 'o': 8}.get(rest[:1].lower())
    try:
        return int(rest[1:].replace('_', ''), base)
    except (TypeError, ValueError):
        return text


def children(scope):
    """Members that instantiate something: instances, generate blocks, arrays."""
    for member in scope:
        kind = member.kind
        if kind == SK.GenerateBlock and getattr(member, 'isUninstantiated', False):
            continue
        if kind in (SK.Instance, SK.GenerateBlock, SK.GenerateBlockArray, SK.InstanceArray,
                    SK.UninstantiatedDef):
            yield member


def cuts_below(scope, found):
    """Names of the black-box (unknown) modules instantiated below `scope`."""
    for member in children(scope):
        if member.kind == SK.UninstantiatedDef:
            found.append(member.definitionName)
        elif member.kind == SK.Instance:
            cuts_below(member.body, found)
        else:
            cuts_below(member, found)
    return found


def walk(scope, srams):
    for member in children(scope):
        if member.kind == SK.Instance:
            if member.definition.name == SRAM_DEF:
                params = {p.name: param_value(p) for p in member.body.parameters
                          if p.name in SRAM_PARAMS}
                srams.append({'path': member.hierarchicalPath, **params,
                              'cuts': sorted(cuts_below(member.body, []))})
            else:
                walk(member.body, srams)
        elif member.kind != SK.UninstantiatedDef:
            walk(member, srams)


def shape(sram):
    return (sram['NumWords'], sram['DataWidth'], sram['ByteWidth'], sram['NumPorts'],
            sram['Latency'])


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--json', help='write the inventory to this file')
    parser.add_argument('--cuts', default=None,
                        help='the technology\'s memory macros (space-separated); each SRAM must have macros among them')
    args, slang_args = parser.parse_known_args()

    driver = Driver()
    driver.addStandardArgs()
    if not (driver.parseCommandLine(' '.join(['slang'] + slang_args), CommandLineOptions())
            and driver.processOptions() and driver.parseAllSources()):
        sys.exit('error: could not parse the sources')
    comp = driver.createCompilation()
    diags = comp.getAllDiagnostics()
    errors = [d for d in diags if d.isError()]
    if errors:
        print(DiagnosticEngine.reportAll(driver.sourceManager, errors), file=sys.stderr)
    print(f'== elaboration: {len(errors)} errors, {len(diags) - len(errors)} warnings')

    srams = []
    for top in comp.getRoot().topInstances:
        walk(top.body, srams)

    known = set(args.cuts.split()) if args.cuts is not None else None
    problems = []
    shapes = defaultdict(list)
    for sram in srams:
        shapes[shape(sram)].append(sram)
        if known is None:
            continue
        if not sram['cuts']:
            problems.append(f'no memory cut for {sram["path"]} {shape(sram)}')
        else:
            problems += [f'cut {c} of {sram["path"]} is not in the cut list'
                         for c in sram['cuts'] if c not in known]

    print(f'== SRAMs: {len(srams)} instances, {len(shapes)} shapes, '
          f'{sum(len(s["cuts"]) for s in srams)} cuts')
    print(f'{"words":>6} {"width":>6} {"byte":>5} {"ports":>5} {"lat":>4} {"count":>5}  cuts')
    for key, group in sorted(shapes.items(), key=lambda kv: (-kv[0][0] * kv[0][1], kv[0])):
        words, width, byte, ports, lat = key
        cuts = Counter(c for s in group for c in s['cuts'])
        cuts = ', '.join(f'{c} x{n}' for c, n in sorted(cuts.items())) or '(none)'
        print(f'{words:>6} {width:>6} {byte:>5} {ports:>5} {lat:>4} {len(group):>5}  {cuts}')
    if known is not None:
        unused = sorted(known - {c for s in srams for c in s['cuts']})
        print(f'== cut list: {len(known)} cuts, unused by this configuration: {" ".join(unused) or "none"}')

    if args.json:
        tmp = f'{args.json}.{os.getpid()}'  # atomic: runs may share the output
        with open(tmp, 'w', encoding='utf-8') as file:
            json.dump({'srams': srams}, file, indent=2)
        os.replace(tmp, args.json)

    for problem in problems:
        print(f'error: {problem}', file=sys.stderr)
    if errors or problems:
        sys.exit(1)


if __name__ == '__main__':
    main()
